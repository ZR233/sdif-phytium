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

use bare_test::{iomap, println};
use sdmmc_phytium::Sdif;

#[test_case]
fn test_base() {
    // map uart data register for using.
    let base_addr = iomap(0x28000000.into(), 0x1000);

    let sd = Sdif::new(base_addr).unwrap();

    // let is_cart_detect = sd.card_detect();

    // assert!(is_cart_detect);
    println!("test passed!");
}
