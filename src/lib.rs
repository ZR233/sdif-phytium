#![no_std]
#![no_main]

use core::ptr::NonNull;

use err::Result;
use log::{debug, info};
use register::SdRegister;

pub mod err;
mod register;

pub struct Sdif {
    reg: NonNull<SdRegister>,
}

unsafe impl Send for Sdif {}

impl Sdif {
    pub fn new(base: NonNull<u8>) -> Result<Self> {
        let s = Sdif { reg: base.cast() };
        s.reset()?;
        Ok(s)
    }

    fn reg(&self) -> &SdRegister {
        unsafe { self.reg.as_ref() }
    }

    /// 卡是否在位
    pub fn card_detect(&self) -> bool {
        self.reg().card_detect()
    }

    pub fn reset(&self) -> Result {
        self.reg().reset()?;
        info!("Reset hardware done !!!");
        Ok(())
    }
}
