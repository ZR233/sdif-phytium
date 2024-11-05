#![no_std]
#![no_main]

use core::ptr::NonNull;

use register::SdRegister;

mod register;

pub struct Sdif {
    reg: NonNull<SdRegister>,
}

unsafe impl Send for Sdif {}

impl Sdif {
    pub fn new(base: NonNull<u8>) -> Self {
        Sdif { reg: base.cast() }
    }

    fn reg(&self) -> &SdRegister {
        unsafe { self.reg.as_ref() }
    }

    /// 卡是否在位
    pub fn card_detect(&self) -> bool {
        self.reg().card_detect()
    }
}
