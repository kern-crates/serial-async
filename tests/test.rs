use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
    thread::sleep,
    time::Duration,
};

use uart_async::{Registers, Uart};

const FIFO_MAX: usize = 4;

#[derive(Default)]
struct FakeRegisters {
    tx_fifo: VecDeque<u8>,
    rx_fifo: Vec<u8>,
}

static OUT_DATA: Mutex<Vec<u8>> = Mutex::new(Vec::new());

#[derive(Clone)]
struct FakeUart {
    registers: Arc<Mutex<FakeRegisters>>,
}

impl FakeUart {
    fn new() -> Self {
        let regs: Arc<Mutex<FakeRegisters>> = Default::default();
        std::thread::spawn({
            let regs = regs.clone();
            move || {
                let mut tick = 0;
                loop {
                    sleep(Duration::from_millis(100));
                    let mut regs = regs.lock().unwrap();
                    if let Some(b) = regs.tx_fifo.pop_front() {
                        OUT_DATA.lock().unwrap().push(b);
                    }

                    tick += 1;
                }
            }
        });

        Self { registers: regs }
    }
}

impl Registers for FakeUart {
    fn can_put(&self) -> bool {
        let regs = self.registers.lock().unwrap();
        regs.tx_fifo.len() < FIFO_MAX
    }

    fn put(&self, c: u8) -> Result<(), uart_async::ErrorKind> {
        let mut regs = self.registers.lock().unwrap();
        regs.tx_fifo.push_back(c);
        if regs.tx_fifo.len() == FIFO_MAX {
            // TODO: trigger irq
        }

        Ok(())
    }

    fn can_get(&self) -> bool {
        todo!()
    }

    fn get(&self) -> Result<u8, uart_async::ErrorKind> {
        todo!()
    }

    fn set_irq_enable(&self, enable: bool) {
        todo!()
    }

    fn get_irq_enable(&self) -> bool {
        todo!()
    }

    fn get_irq_event(&self) -> uart_async::IrqEvent {
        todo!()
    }

    fn clean_irq_event(&self, event: uart_async::IrqEvent) {
        todo!()
    }
}

#[test]
fn test_take() {
    let mut uart = Uart::new(FakeUart::new());
    let tx = uart.try_take_tx().unwrap();
    println!("take: {}", uart.is_tx_taken());
    let e = uart.try_take_tx();
    drop(tx);

    assert!(e.is_none())
}
