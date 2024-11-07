#![no_std]
#![no_main]

use core::{
    marker::PhantomData,
    ptr::NonNull,
    sync::atomic::{fence, Ordering},
    time::Duration,
};

use define::{CmdDataFlag, SdifCmdData};
use err::{Error, Result};
use log::{error, info};
use register::{SdRegister, MAX_FIFO_CNT};
use tock_registers::interfaces::Readable;

mod define;
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

pub struct Sdif<K: KFun> {
    reg: NonNull<SdRegister>,
    config: Config,
    _phantom: PhantomData<K>,
}

unsafe impl<K: KFun> Send for Sdif<K> {}

impl<K: KFun> Sdif<K> {
    pub const COMPATIBLE: &'static str = "phytium,mci";

    pub fn new(base: NonNull<u8>, config: Config) -> Result<Self> {
        let s = Sdif {
            reg: base.cast(),
            config,
            _phantom: PhantomData,
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
    pub fn poi_transfer(&self, cmd_data: &SdifCmdData) -> Result {
        let read = cmd_data.flag.is_set(CmdDataFlag::READ_DATA);

        if self.config.trans_mode != TransMode::PIO {
            error!("device is not configure in PIO transfer mode.");
            return Err(Error::InvalidState);
        }

        /* for removable media, check if card exists */
        if (!self.config.non_removeable) && (!self.card_detect()) {
            return Err(Error::NoCard);
        }

        self.reg().poll_wait_busy_card::<K>()?;

        self.reg().reset_fifo_and_not_use_dma()?;

        unsafe {
            if let Some(data) = cmd_data.data_p.as_mut() {
                if data.datalen > MAX_FIFO_CNT {
                    error!("data length is too long.");
                    return Err(Error::NotSupport);
                }

                self.reg().set_trans_bytes(data.datalen);
                self.reg().set_block_size(data.blksz);

                if !read {
                    fence(Ordering::SeqCst);
                    self.reg().pio_write_data(data)?;
                }
            }
        }

        self.reg().transfer_cmd(cmd_data)?;

        Ok(())
    }
    pub fn reset(&self) -> Result {
        self.reg().reset(self.config)?;
        info!("Reset hardware done !!!");
        Ok(())
    }
}

pub trait KFun {
    fn sleep(duration: Duration);
}
