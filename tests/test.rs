use std::{
    collections::VecDeque,
    sync::{Arc, Mutex, OnceLock},
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
                    sleep(Duration::from_millis(20));
                    let mut is_irq = false;
                    let mut regs = regs.lock().unwrap();
                    if let Some(b) = regs.tx_fifo.pop_front() {
                        OUT_DATA.lock().unwrap().push(b);
                        println!("tx: {b}");
                    }
                    if regs.tx_fifo.is_empty() {
                        is_irq = true;
                    }

                    drop(regs);
                    if is_irq {
                        irq_handle();
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
        if regs.tx_fifo.len() > FIFO_MAX {
            panic!("tx fifo overflow");
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

static UART: OnceLock<Uart<FakeUart>> = OnceLock::new();

fn irq_handle() {
    unsafe {
        UART.get().unwrap().handle_irq();
    }
}

#[test]
fn test_take() {
    let mut uart = Uart::new(FakeUart::new());
    let tx = uart.try_take_tx().unwrap();
    let e = uart.try_take_tx();
    assert!(e.is_none());
    drop(tx);

    assert!(uart.try_take_tx().is_some());
}

#[test]
fn test_tx() {
    let mut uart = Uart::new(FakeUart::new());
    let mut tx = uart.try_take_tx().unwrap();

    let data = (0..12).collect::<Vec<u8>>();

    let mut buf = &data[..];

    while !buf.is_empty() {
        let n = tx.write(buf).unwrap();
        if n > 0 {
            println!("writen: {}", n);
        }
        buf = &buf[n..];
    }
    sleep(Duration::from_millis(100));
    assert_eq!(OUT_DATA.lock().unwrap().as_slice(), &data);
}
