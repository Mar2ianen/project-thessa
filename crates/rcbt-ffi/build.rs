//! Compiles the vendored libcbt shim for bench/test comparison only.
//!
//! The shim is built WITHOUT OpenMP so the comparison measures serial
//! bitfield updates against the serial Rust tree, not thread-pool effects.

fn main() {
    println!("cargo::rerun-if-changed=../../third_party/libcbt/cbt_shim.c");
    println!("cargo::rerun-if-changed=../../third_party/libcbt/cbt.h");
    cc::Build::new()
        .file("../../third_party/libcbt/cbt_shim.c")
        .include("../../third_party/libcbt")
        .std("c11")
        .opt_level(3)
        .compile("thessa_cbt_shim");
}
