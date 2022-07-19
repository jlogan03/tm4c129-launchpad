//! Functions required by rustc/LLVM

#[cfg(debug_assertions)]
use super::board;

#[cfg(debug_assertions)]
use core::panic::PanicInfo;

/// Required by the compiler.
#[no_mangle]
pub extern "C" fn __aeabi_unwind_cpp_pr0() -> () {
}

/// Required by the compiler.
#[no_mangle]
pub extern "C" fn __aeabi_unwind_cpp_pr1() -> () {
}

/// Required by modules that haven't been build with panic = "abort"
#[allow(non_snake_case)]
#[no_mangle]
pub extern "C" fn _Unwind_Resume() -> () {
}

#[cfg(debug_assertions)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    board::safe();
}
