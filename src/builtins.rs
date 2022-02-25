//! Functions required by rustc/LLVM

use board;

#[cfg(debug_assertions)]
use core::panic::PanicInfo;

/// Required by the compiler.
#[no_mangle]
pub extern "C" fn __aeabi_unwind_cpp_pr0() -> () {
    board::safe();
}

/// Required by the compiler.
#[no_mangle]
pub extern "C" fn __aeabi_unwind_cpp_pr1() -> () {
    board::safe();
}

/// Required by modules that haven't been build with panic = "abort"
#[allow(non_snake_case)]
#[no_mangle]
pub extern "C" fn _Unwind_Resume() -> () {
    board::safe();
}

#[cfg(debug_assertions)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    board::panic();
}
