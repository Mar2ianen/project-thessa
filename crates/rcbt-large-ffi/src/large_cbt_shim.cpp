#include <cstdlib>
#include <cstdint>
#include <bit>

#include "ocbt_128k.h"
#include "ocbt_256k.h"
#include "ocbt_512k.h"
#include "ocbt_1m.h"

std::uint32_t countbits(std::uint32_t value)
{
    return static_cast<std::uint32_t>(std::popcount(value));
}

std::uint32_t countbits(std::uint64_t value)
{
    return static_cast<std::uint32_t>(std::popcount(value));
}

void __handle_fail(const char*, const char*, int)
{
    std::abort();
}

struct LargeCbtHandle
{
    std::uint32_t variant;
    CBT* tree;
};

template <typename Tree>
LargeCbtHandle* make_tree(std::uint32_t variant)
{
    return new LargeCbtHandle { variant, new Tree() };
}

extern "C"
{

void* thessa_large_cbt_create(std::uint32_t variant)
{
    switch (variant)
    {
    case 0: return make_tree<UPCBT_128k>(variant);
    case 1: return make_tree<UPCBT_256k>(variant);
    case 2: return make_tree<UPCBT_512k>(variant);
    case 3: return make_tree<UPCBT_1M>(variant);
    default: return nullptr;
    }
}

void thessa_large_cbt_destroy(void* opaque)
{
    auto* handle = static_cast<LargeCbtHandle*>(opaque);
    if (handle == nullptr)
        return;
    switch (handle->variant)
    {
    case 0: delete static_cast<UPCBT_128k*>(handle->tree); break;
    case 1: delete static_cast<UPCBT_256k*>(handle->tree); break;
    case 2: delete static_cast<UPCBT_512k*>(handle->tree); break;
    case 3: delete static_cast<UPCBT_1M*>(handle->tree); break;
    default: break;
    }
    delete handle;
}

static CBT* tree(void* opaque)
{
    return static_cast<LargeCbtHandle*>(opaque)->tree;
}

static const CBT* tree_const(const void* opaque)
{
    return static_cast<const LargeCbtHandle*>(opaque)->tree;
}

std::uint32_t thessa_large_cbt_num_elements(const void* opaque)
{
    return tree_const(opaque)->num_elements();
}

std::uint32_t thessa_large_cbt_max_depth(const void* opaque)
{
    return tree_const(opaque)->max_depth();
}

std::uint32_t thessa_large_cbt_last_level_size(const void* opaque)
{
    return tree_const(opaque)->last_level_size();
}

std::uint32_t thessa_large_cbt_memory_footprint(const void* opaque)
{
    return tree_const(opaque)->memory_footprint();
}

std::uint32_t thessa_large_cbt_buffer_size(const void* opaque, std::uint32_t index)
{
    return tree_const(opaque)->buffer_size(index);
}

std::uint32_t thessa_large_cbt_element_size(const void* opaque, std::uint32_t index)
{
    return tree_const(opaque)->element_size(index);
}

const std::uint8_t* thessa_large_cbt_buffer(const void* opaque, std::uint32_t index)
{
    return reinterpret_cast<const std::uint8_t*>(tree_const(opaque)->raw_buffer(index));
}

void thessa_large_cbt_set_bit(void* opaque, std::uint32_t bit, std::uint32_t state)
{
    tree(opaque)->set_bit(bit, state != 0);
}

std::uint32_t thessa_large_cbt_get_bit(const void* opaque, std::uint32_t bit)
{
    return tree_const(opaque)->get_bit(bit);
}

std::uint32_t thessa_large_cbt_bit_count(const void* opaque)
{
    return tree_const(opaque)->bit_count();
}

std::uint32_t thessa_large_cbt_decode_bit(const void* opaque, std::uint32_t handle)
{
    return tree_const(opaque)->decode_bit(handle);
}

std::uint32_t thessa_large_cbt_decode_bit_complement(const void* opaque, std::uint32_t handle)
{
    return tree_const(opaque)->decode_bit_complement(handle);
}

std::uint32_t thessa_large_cbt_heap_element(const void* opaque, std::uint32_t id)
{
    return tree_const(opaque)->get_heap_element(id);
}

void thessa_large_cbt_reduce(void* opaque)
{
    tree(opaque)->reduce();
}

void thessa_large_cbt_clear(void* opaque)
{
    tree(opaque)->clear();
}

}
