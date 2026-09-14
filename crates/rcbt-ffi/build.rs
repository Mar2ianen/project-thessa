//! Compiles the vendored libcbt shim for bench/test comparison only.
//!
//! Two variants from the same upstream source: serial (primary honest
//! baseline) and OpenMP (thread-scaling experiments only). The upstream
//! header emits its helpers as global symbols, so the scaling TU is built
//! from a build-time renamed copy (`cbt_` -> `cbtmt_`) in OUT_DIR; the
//! vendored file on disk is never modified. Neither archive is ever linked
//! into the game runtime.

use std::path::PathBuf;

fn main() {
    println!("cargo::rerun-if-changed=../../third_party/libcbt/cbt_shim.c");
    println!("cargo::rerun-if-changed=../../third_party/libcbt/cbt.h");
    println!("cargo::rerun-if-changed=build.rs");

    // Serial baseline: no OpenMP, so the primary comparison measures
    // data-structure cost, not thread-pool effects.
    cc::Build::new()
        .file("../../third_party/libcbt/cbt_shim.c")
        .include("../../third_party/libcbt")
        .define("CBT_STATIC", None)
        .std("c11")
        .opt_level(3)
        .compile("thessa_cbt_shim");

    // Scaling variant: same source with upstream symbols renamed so both
    // archives link into one binary without clashing. OpenMP is attempted,
    // but a toolchain without it falls back to serial code: the scaling
    // bench then reports a flat line instead of breaking the build on any
    // platform.
    let out = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR"));
    let header: String = std::fs::read_to_string("../../third_party/libcbt/cbt.h")
        .expect("vendored cbt.h")
        .replace("cbt_", "cbtmt_");
    let shim: String = std::fs::read_to_string("../../third_party/libcbt/cbt_shim.c")
        .expect("vendored shim")
        .replace("cbt_", "cbtmt_")
        .replace("\"cbt.h\"", "\"cbtmt.h\"")
        // The API() macro adds the thessa_mt_ prefix itself; keep its
        // argument on the original spelling so entry points come out as
        // thessa_mt_cbt_* while upstream types stay cbtmt_*.
        .replace("API(cbtmt_", "API(cbt_");
    std::fs::write(out.join("cbtmt.h"), header).expect("renamed header");
    std::fs::write(out.join("cbt_shim_mt.c"), shim).expect("renamed shim");

    let mut mt = cc::Build::new();
    mt.file(out.join("cbt_shim_mt.c"))
        .include(&out)
        .define("CBT_STATIC", None)
        .std("c11")
        .opt_level(3)
        .define("THESSA_MT", "1");
    let compiler = mt.get_compiler();
    let want_openmp = compiler.is_like_gnu() || compiler.is_like_clang();
    if want_openmp {
        mt.flag("-fopenmp");
    }
    // try_compile first: Apple Clang and MSVC-style toolchains reject
    // -fopenmp, and the fallback keeps every platform green.
    if want_openmp && mt.try_compile("thessa_cbt_shim_mt").is_ok() {
        println!("cargo::rustc-link-lib=gomp");
    } else {
        cc::Build::new()
            .file(out.join("cbt_shim_mt.c"))
            .include(&out)
            .define("CBT_STATIC", None)
            .std("c11")
            .opt_level(3)
            .define("THESSA_MT", "1")
            .compile("thessa_cbt_shim_mt");
    }
}
