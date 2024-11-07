#![no_std]
#![no_main]
#![feature(custom_test_frameworks)]
#![test_runner(bare_test::test_runner)]
#![reexport_test_harness_main = "test_main"]

extern crate bare_test;

#[bare_test::entry]
fn main() {
    test_main();
}

use core::hint::spin_loop;

use bare_test::{iomap, println};
use log::error;
use sdmmc_phytium::{Config, KFun, Sdif, TransMode};

struct KFunImpl;

impl KFun for KFunImpl {
    fn sleep(duration: core::time::Duration) {
        spin_loop();
    }
}

#[test_case]
fn test_base() {
    // map uart data register for using.
    let base_addr = iomap(0x28000000.into(), 0x1000);

    let config = Config {
        trans_mode: TransMode::PIO,
        non_removeable: false,
    };

    let sd = Sdif::<KFunImpl>::new(base_addr, config)
        .inspect_err(|e| {
            error!("{:?}", e);
        })
        .unwrap();

    // let is_cart_detect = sd.card_detect();

    // assert!(is_cart_detect);
    println!("test passed!");
}
