/* FFI shim over vendored libcbt (see cbt.h and ../libcbt.REVISION).
 *
 * The shim exposes plain-integer functions so Rust never passes the
 * bitfield `cbt_Node` struct by value across the ABI boundary. Semantics
 * mirror `thessa-rcbt-core::Tree` one to one:
 *   - split takes a leaf,
 *   - merge takes the parent whose two children are leaves
 *     (libcbt merges a pair by clearing the right sibling bit, so the shim
 *     internally addresses the left child),
 *   - reduction/commit happens explicitly via cbt_reduce, which runs
 *     `cbt_Update` with a no-op callback (a full decode pass plus the sum
 *     reduction, i.e. the honest per-frame commit cost).
 *
 * The default build is WITHOUT OpenMP, so the primary comparison measures
 * serial bitfield updates against the serial Rust tree, not thread-pool
 * effects. A second build with THESSA_MT renames every entry point with a
 * `thessa_mt_` prefix and enables OpenMP. The serial entry points are
 * suitable for an explicitly selected runtime backend; the OpenMP entry
 * points are optional and can be selected where a parallel handle is safe.
 */

#include <stdint.h>

#ifdef THESSA_MT
#define API(name) thessa_mt_##name
#ifdef _OPENMP
#include <omp.h>
#endif
#else
#define API(name) thessa_##name
#endif

#define CBT_IMPLEMENTATION
#include "cbt.h"

void *API(cbt_create)(int64_t max_depth, int64_t depth)
{
    return (void *)cbt_CreateAtDepth(max_depth, depth);
}

void API(cbt_release)(void *tree)
{
    cbt_Release((cbt_Tree *)tree);
}

void API(cbt_reset_to_depth)(void *tree, int64_t depth)
{
    cbt_ResetToDepth((cbt_Tree *)tree, depth);
}

void API(cbt_split)(void *tree, uint64_t id, int64_t depth)
{
    cbt_SplitNode((cbt_Tree *)tree, cbt_CreateNode(id, depth));
}

void API(cbt_merge_children)(void *tree, uint64_t parent_id, int64_t parent_depth)
{
    /* Address the left child: clearing the right sibling bit merges exactly
     * the two children of `parent`, matching Tree::merge(parent). */
    cbt_Node left = cbt_LeftChildNode(cbt_CreateNode(parent_id, parent_depth));
    cbt_MergeNode((cbt_Tree *)tree, left);
}

static void API(noop_callback)(cbt_Tree *tree, const cbt_Node node, const void *user_data)
{
    (void)tree;
    (void)node;
    (void)user_data;
}

void API(cbt_reduce)(void *tree)
{
    cbt_Update((cbt_Tree *)tree, &API(noop_callback), NULL);
}

/* Commit without the decode pass: only the sum reduction. Valid when the
 * caller applied a known operation list through the entry points above and
 * just needs the rank/select structure current again. Same translation unit
 * as CBT_IMPLEMENTATION, so the static helper is directly reachable. */
void API(cbt_reduce_only)(void *tree)
{
    cbt__ComputeSumReduction((cbt_Tree *)tree);
}

/* Replay a whole batch across one FFI boundary. kinds[i]: 0 = split the leaf
 * (ids[i], depths[i]); 1 = merge the children of that parent. Returns the
 * number of bit writes issued (validity of each op is the caller's contract,
 * exactly like the single-op entry points). */
int64_t API(cbt_apply_batch)(
    void *tree,
    const uint64_t *ids,
    const int64_t *depths,
    const uint8_t *kinds,
    int64_t count)
{
    cbt_Tree *t = (cbt_Tree *)tree;
    for (int64_t i = 0; i < count; ++i) {
        cbt_Node node = cbt_CreateNode(ids[i], depths[i]);
        if (kinds[i] == 0) {
            cbt_SplitNode(t, node);
        } else {
            cbt_Node left = cbt_LeftChildNode(node);
            cbt_MergeNode(t, left);
        }
    }
    return count;
}

int64_t API(cbt_node_count)(const void *tree)
{
    return cbt_NodeCount((const cbt_Tree *)tree);
}

int API(cbt_decode)(const void *tree, int64_t index, uint64_t *out_id, int64_t *out_depth)
{
    cbt_Node node = cbt_DecodeNode((const cbt_Tree *)tree, index);
    *out_id = node.id;
    *out_depth = (int64_t)node.depth;
    return 1;
}

int API(cbt_is_leaf)(const void *tree, uint64_t id, int64_t depth)
{
    return cbt_IsLeafNode((const cbt_Tree *)tree, cbt_CreateNode(id, depth)) ? 1 : 0;
}

int64_t API(cbt_heap_bytes)(const void *tree)
{
    return cbt_HeapByteSize((const cbt_Tree *)tree);
}

int64_t API(cbt_max_depth)(const void *tree)
{
    return cbt_MaxDepth((const cbt_Tree *)tree);
}

void API(cbt_set_threads)(int n)
{
#ifdef _OPENMP
    omp_set_num_threads(n);
#else
    (void)n;
#endif
}
