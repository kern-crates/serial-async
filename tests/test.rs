use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
    thread::sleep,
    time::Duration,
};

use uart_async::{IrqEvent, Registers, Uart};

const FIFO_MAX: usize = 4;

#[derive(Default)]
struct FakeRegisters {
    tx_fifo: VecDeque<u8>,
    is_tx_empty: bool,
    is_rx_full: bool,
    rx_fifo: VecDeque<u8>,
    out_data: Vec<u8>,
    in_data: VecDeque<u8>,
}

type IrqFn = Arc<Mutex<Option<Box<dyn Fn() + Send + 'static>>>>;

#[derive(Clone)]
struct FakeUart {
    irq_fn: IrqFn,
    registers: Arc<Mutex<FakeRegisters>>,
}

impl FakeUart {
    fn new(in_data: &[u8]) -> Self {
        let irq_fn: IrqFn = Arc::new(Mutex::new(None));
        let regs = Arc::new(Mutex::new(FakeRegisters {
            in_data: VecDeque::from_iter(in_data.iter().cloned()),
            ..Default::default()
        }));

        std::thread::spawn({
            let irq_fn = irq_fn.clone();
            let regs = regs.clone();
            move || {
                loop {
                    sleep(Duration::from_millis(20));
                    let mut regs = regs.lock().unwrap();
                    if let Some(b) = regs.tx_fifo.pop_front() {
                        regs.out_data.push(b);
                        println!("tx: {b}, fifo: {:?}", regs.tx_fifo.len());
                        if regs.tx_fifo.is_empty() {
                            regs.is_tx_empty = true;
                        }
                    }

                    if let Some(b) = regs.in_data.pop_front() {
                        if regs.rx_fifo.len() == FIFO_MAX {
                            panic!("rx fifo overflow");
                        }
                        regs.rx_fifo.push_back(b);
                        if regs.rx_fifo.len() == FIFO_MAX {
                            regs.is_rx_full = true;
                        }
                    }

                    let is_irq = regs.is_tx_empty || regs.is_rx_full;
                    drop(regs);
                    if is_irq {
                        let g = irq_fn.lock().unwrap();
                        if let Some(irq_fn) = g.as_ref() {
                            println!("IRQ!");
                            irq_fn();
                        }
                    }
                }
            }
        });

        Self {
            registers: regs,
            irq_fn,
        }
    }
}

impl Registers for FakeUart {
    fn can_put(&self) -> bool {
        let regs = self.registers.lock().unwrap();
        regs.tx_fifo.len() < FIFO_MAX
    }

    fn put(&self, c: u8) -> Result<(), uart_async::ErrorKind> {
        let mut regs = self.registers.lock().unwrap();
        if regs.tx_fifo.len() == FIFO_MAX {
            panic!("tx fifo overflow");
        }
        regs.tx_fifo.push_back(c);

        Ok(())
    }

    fn can_get(&self) -> bool {
        let regs = self.registers.lock().unwrap();
        !regs.rx_fifo.is_empty()
    }

    fn get(&self) -> Result<u8, uart_async::ErrorKind> {
        let mut regs = self.registers.lock().unwrap();
        regs.rx_fifo
            .pop_front()
            .ok_or(uart_async::ErrorKind::Overrun)
    }

    fn get_irq_event(&self) -> uart_async::IrqEvent {
        let regs = self.registers.lock().unwrap();

        IrqEvent {
            can_get: regs.is_rx_full,
            can_put: regs.is_tx_empty,
        }
    }

    fn clean_irq_event(&self, event: uart_async::IrqEvent) {
        println!("clean_irq_event");
        let mut regs = self.registers.lock().unwrap();
        if event.can_put {
            regs.is_tx_empty = false;
        }
        if event.can_get {
            regs.is_rx_full = false;
        }
    }
}

#[test]
fn test_take() {
    let mut uart = Uart::new(FakeUart::new(&[]));
    let tx = uart.try_take_tx().unwrap();
    let e = uart.try_take_tx();
    assert!(e.is_none());
    drop(tx);

    assert!(uart.try_take_tx().is_some());
}

#[test]
fn test_tx() {
    let fake = FakeUart::new(&[]);

    let fake_data = fake.registers.clone();

    let mut uart = Uart::new(fake);

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
    assert_eq!(fake_data.lock().unwrap().out_data.as_slice(), &data);
}

#[tokio::test]
async fn test_tx_async() {
    let fake = FakeUart::new(&[]);
    let irq_fn = fake.irq_fn.clone();
    let fake_data = fake.registers.clone();

    let mut uart = Uart::new(fake);
    let handler = uart.irq_handler.take().unwrap();
    {
        let mut g = irq_fn.lock().unwrap();
        g.replace(Box::new(move || unsafe {
            handler.handle_irq();
        }));
    }

    let mut tx = uart.try_take_tx().unwrap();

    let data = (0..12).collect::<Vec<u8>>();

    tokio::spawn(async {
        loop {
            tokio::time::sleep(Duration::from_millis(1)).await;
        }
    });

    tx.write_all(&data).await.unwrap();

    tokio::time::sleep(Duration::from_millis(500)).await;

    assert_eq!(fake_data.lock().unwrap().out_data.as_slice(), &data);
}

#[tokio::test]
async fn test_rx_async() {
    let in_data = (0..12).collect::<Vec<u8>>();

    let fake = FakeUart::new(&in_data);
    let irq_fn = fake.irq_fn.clone();

    let mut uart = Uart::new(fake);
    let handler = uart.irq_handler.take().unwrap();
    {
        let mut g = irq_fn.lock().unwrap();
        g.replace(Box::new(move || unsafe {
            handler.handle_irq();
        }));
    }

    let mut rx = uart.try_take_rx().unwrap();

    let mut data = vec![0u8; in_data.len()];

    for _ in 0..1000 {
        tokio::spawn(async {
            loop {
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
        });
    }

    rx.read_all(&mut data).await.unwrap();

    assert_eq!(&in_data, &data);
}
