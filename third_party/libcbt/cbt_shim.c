/* Bench-only FFI shim over vendored libcbt (see cbt.h and ../libcbt.REVISION).
 *
 * The shim exposes plain-integer functions so Rust never passes the
 * bitfield `cbt_Node` struct by value across the ABI boundary. Semantics
 * mirror `thessa-rcbt-core::Tree` one to one:
 *   - split takes a leaf,
 *   - merge takes the parent whose two children are leaves
 *     (libcbt merges a pair by clearing the right sibling bit, so the shim
 *     internally addresses the left child),
 *   - reduction/commit happens explicitly via thessa_cbt_reduce, which runs
 *     `cbt_Update` with a no-op callback (a full decode pass plus the sum
 *     reduction, i.e. the honest per-frame commit cost).
 *
 * The shim is compiled WITHOUT OpenMP, so the comparison measures serial
 * bitfield updates against the serial Rust tree, not thread-pool effects.
 * This file is bench/test tooling only and is never linked into the game
 * runtime or server.
 */

#include <stdint.h>

#define CBT_IMPLEMENTATION
#include "cbt.h"

void *thessa_cbt_create(int64_t max_depth, int64_t depth)
{
    return (void *)cbt_CreateAtDepth(max_depth, depth);
}

void thessa_cbt_release(void *tree)
{
    cbt_Release((cbt_Tree *)tree);
}

void thessa_cbt_reset_to_depth(void *tree, int64_t depth)
{
    cbt_ResetToDepth((cbt_Tree *)tree, depth);
}

void thessa_cbt_split(void *tree, uint64_t id, int64_t depth)
{
    cbt_SplitNode((cbt_Tree *)tree, cbt_CreateNode(id, depth));
}

void thessa_cbt_merge_children(void *tree, uint64_t parent_id, int64_t parent_depth)
{
    /* Address the left child: clearing the right sibling bit merges exactly
     * the two children of `parent`, matching Tree::merge(parent). */
    cbt_Node left = cbt_LeftChildNode(cbt_CreateNode(parent_id, parent_depth));
    cbt_MergeNode((cbt_Tree *)tree, left);
}

static void thessa_noop_callback(cbt_Tree *tree, const cbt_Node node, const void *user_data)
{
    (void)tree;
    (void)node;
    (void)user_data;
}

void thessa_cbt_reduce(void *tree)
{
    cbt_Update((cbt_Tree *)tree, &thessa_noop_callback, NULL);
}

int64_t thessa_cbt_node_count(const void *tree)
{
    return cbt_NodeCount((const cbt_Tree *)tree);
}

int thessa_cbt_decode(const void *tree, int64_t index, uint64_t *out_id, int64_t *out_depth)
{
    cbt_Node node = cbt_DecodeNode((const cbt_Tree *)tree, index);
    *out_id = node.id;
    *out_depth = (int64_t)node.depth;
    return 1;
}

int thessa_cbt_is_leaf(const void *tree, uint64_t id, int64_t depth)
{
    return cbt_IsLeafNode((const cbt_Tree *)tree, cbt_CreateNode(id, depth)) ? 1 : 0;
}

int64_t thessa_cbt_heap_bytes(const void *tree)
{
    return cbt_HeapByteSize((const cbt_Tree *)tree);
}

int64_t thessa_cbt_max_depth(const void *tree)
{
    return cbt_MaxDepth((const cbt_Tree *)tree);
}
