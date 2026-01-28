//! Build script for speaker-os
//! Configures linker for RP2350

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Copy memory.x to OUT_DIR for the linker to find
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();

    // Tell linker where to find memory.x
    println!("cargo:rustc-link-search={}", out.display());

    // Set linker arguments
    println!("cargo:rustc-link-arg=--nmagic");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    // Rebuild if memory.x changes
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}
