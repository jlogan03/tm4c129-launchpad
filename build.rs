use std::env;
use std::fs::copy;
use std::path::PathBuf;

fn main() {
    // Copy the memory.x 

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap_or_default());
    let in_file: PathBuf = PathBuf::from("src/memory.x");
    let out_file: PathBuf = out_dir.join("memory.x");
    let _ = copy(in_file, out_file).unwrap_or_default();

    println!("cargo:rustc-link-search={}", out_dir.display());
}
