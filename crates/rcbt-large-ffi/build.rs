fn main() {
    println!("cargo::rerun-if-changed=../../third_party/large_cbt");
    println!("cargo::rerun-if-changed=src/large_cbt_shim.cpp");

    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("c++20")
        .include("../../third_party/large_cbt")
        .file("../../third_party/large_cbt/ocbt_128k.cpp")
        .file("../../third_party/large_cbt/ocbt_256k.cpp")
        .file("../../third_party/large_cbt/ocbt_512k.cpp")
        .file("../../third_party/large_cbt/ocbt_1m.cpp")
        .file("src/large_cbt_shim.cpp")
        .compile("thessa_large_cbt");
}
