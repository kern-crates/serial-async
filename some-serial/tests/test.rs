#![no_std]
#![no_main]
#![feature(used_with_arg)]
#![cfg(target_os = "none")]

extern crate alloc;
extern crate bare_test;

#[bare_test::tests]
mod tests {
    use core::sync::atomic::AtomicBool;

    use super::*;
    use alloc::sync::Arc;
    use bare_test::{
        GetIrqConfig,
        globals::{PlatformInfoKind, global_val},
        irq::{IrqHandleResult, IrqInfo, IrqParam},
        mem::iomap,
    };
    use log::{debug, info};
    use some_serial::pl011;

    #[test]
    fn test_pl011() {
        info!("test uart");

        let (addr, size, irq_info) = get_uart(&["arm,pl011"]);

        let base = iomap(addr.into(), size);

        debug!("base address: {:p}, size: {:#x}", base, size);

        let mut uart = pl011::new(base.as_ptr() as _);

        info!("Starting Test");

        uart.enable();

        info!("UART enabled");

        let mut sender = uart.try_take_tx().unwrap();
        let receiver = uart.try_take_rx().unwrap();

        sender.write_all_blocking(b"Hello, world!\n").unwrap();
    }

    fn get_uart(cmp: &[&str]) -> (usize, usize, IrqInfo) {
        let PlatformInfoKind::DeviceTree(fdt) = &global_val().platform_info;
        let fdt = fdt.get();

        let chosen = fdt.chosen().unwrap().debugcon().unwrap();

        for node in fdt.find_compatible(cmp) {
            if node.name() == chosen.name() {
                continue;
            }

            let addr = node.reg().unwrap().next().unwrap();

            let size = addr.size.unwrap_or(0x1000);

            let irq_info = node.irq_info().unwrap();

            return (addr.address as usize, size, irq_info);
        }

        panic!("No matching UART node found");
    }
}
