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

#[test_case]
fn test_base() {
    // map uart data register for using.
    let base_addr = iomap(28000000.into(), 0x1000);

    println!("uart test passed!");
}
