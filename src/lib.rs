#![no_std]
#![no_main]

use core::ptr::NonNull;

use err::Result;
use log::info;
use register::SdRegister;

pub mod err;
mod register;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransMode {
    DMA,
    /// NO-DMA trans by read/write Fifo
    PIO,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub trans_mode: TransMode,
    pub non_removeable: bool,
}

pub struct Sdif {
    reg: NonNull<SdRegister>,
    config: Config,
}

unsafe impl Send for Sdif {}

impl Sdif {
    pub const COMPATIBLE: &'static str = "phytium,mci";

    pub fn new(base: NonNull<u8>, config: Config) -> Result<Self> {
        let s = Sdif {
            reg: base.cast(),
            config,
        };
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
        self.reg().reset(self.config)?;
        info!("Reset hardware done !!!");
        Ok(())
    }
}
