#![no_std]
#![no_main]
#![feature(used_with_arg)]
#![cfg(target_os = "none")]

extern crate alloc;
extern crate bare_test;

#[bare_test::tests]
mod tests {
    use super::*;
    use bare_test::{
        GetIrqConfig,
        globals::{PlatformInfoKind, global_val},
        irq::IrqInfo,
        mem::iomap,
    };
    use log::{debug, info};
    use serial_async::{DataBits, FlowControl, Parity, StopBits, UartConfig};
    use some_serial::pl011;

    #[test]
    fn test_pl011_loopback() {
        info!("test uart loopback");

        let (addr, size, clk_freq, _irq_info) = get_uart(&["arm,pl011"]);
        // let addr = 0x2800d000;
        let base = iomap(addr.into(), size);

        debug!("base address: {base:p}, size: {size:#x}, clk {clk_freq}",);

        let mut uart = pl011::new(base, clk_freq);

        info!("Starting Loopback Test");
        uart.configure(&UartConfig {
            baud_rate: 115200,
            data_bits: DataBits::Eight,
            stop_bits: StopBits::One,
            parity: Parity::None,
            flow_control: FlowControl::None,
        })
        .unwrap();
        uart.enable();

        uart.loopback_enable();

        // 检查控制寄存器的值
        let cr_value = uart.registers().read_control_register();
        info!("Control Register after setup: 0x{:08x}", cr_value);
        info!(
            "UARTEN: {}, TXE: {}, RXE: {}, LBE: {}",
            (cr_value & 1) != 0,
            (cr_value & (1 << 8)) != 0,
            (cr_value & (1 << 9)) != 0,
            (cr_value & (1 << 7)) != 0
        );

        info!("UART loopback enabled");

        let mut sender = uart.try_take_tx().unwrap();
        let mut receiver = uart.try_take_rx().unwrap();

        // 测试数据
        let test_data = b"LOOPBACK_TEST_123";

        info!("Sending test data: {:?}", core::str::from_utf8(test_data));

        // 逐字节发送并立即尝试接收
        let mut received_data = [0u8; 17];
        for (i, &byte_to_send) in test_data.iter().enumerate() {
            sender.write_all_blocking(&[byte_to_send]).unwrap();
            info!(
                "Sent byte {}: {} ('{}')",
                i, byte_to_send, byte_to_send as char
            );

            // 等待并接收这个字节
            let mut wait_count = 0;

            let mut buff = [0u8; 1];

            while receiver.read(&mut buff).unwrap() == 0 {
                wait_count += 1;
                if wait_count > 100000 {
                    panic!("Timeout waiting for byte {}", i);
                }
            }

            received_data[i] = buff[0];
            info!(
                "Received byte {}: {} ('{}')",
                i, received_data[i], received_data[i] as char
            );
        }

        info!("Received data: {:?}", core::str::from_utf8(&received_data));

        // 验证发送和接收的数据是否相同
        assert_eq!(test_data, &received_data);

        info!("Loopback test passed!");

        // 禁用回环模式
        uart.loopback_disable();

        info!("UART loopback disabled");
    }

    fn get_uart(cmp: &[&str]) -> (usize, usize, usize, IrqInfo) {
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

            let mut clk = 0;
            for node_clk in node.clocks() {
                info!("UART clock: {:?}", node_clk.clock_frequency);
                if let Some(f) = node_clk.clock_frequency {
                    clk = f;
                    break;
                }
            }

            return (addr.address as usize, size, clk as _, irq_info);
        }

        panic!("No matching UART node found");
    }
}
