//! Functions required by rustc/LLVM

use board;
use core::panic::PanicInfo;

/// Required by the compiler.
#[no_mangle]
pub extern "C" fn __aeabi_unwind_cpp_pr0() -> ! {
    board::panic();
}

/// Required by the compiler.
#[no_mangle]
pub extern "C" fn __aeabi_unwind_cpp_pr1() -> ! {
    board::panic();
}

/// Required by modules that haven't been build with panic = "abort"
#[allow(non_snake_case)]
#[no_mangle]
pub extern "C" fn _Unwind_Resume() -> ! {
    board::panic();
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    board::panic();
}
