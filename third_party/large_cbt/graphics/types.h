#pragma once

#include <cstdint>

// The upstream CBT headers use these opaque graphics handles only to describe
// the optional GPU mirror. The Rust binding does not expose or compile the
// upstream DX12 backend; these fixed-width placeholders keep the CPU OCBT
// layout source self-contained.
using RenderWindow = std::uint64_t;
using GraphicsDevice = std::uint64_t;
using CommandQueue = std::uint64_t;
using CommandBuffer = std::uint64_t;
using GraphicsBuffer = std::uint64_t;

