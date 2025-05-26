#![cfg_attr(not(test), no_std)]

extern crate alloc;

use alloc::{boxed::Box, sync::Arc};
use core::{
    sync::atomic::{AtomicBool, Ordering},
    task::Poll,
};
use futures::{FutureExt, task::AtomicWaker};
use rdif_serial::ErrorBase;
pub use rdif_serial::{DriverGeneric, Interface, SerialError};

#[derive(Debug, Clone, Copy, Default)]
pub struct IrqEvent {
    pub can_get: bool,
    pub can_put: bool,
}

pub trait Registers: Clone + 'static {
    fn can_put(&self) -> bool;
    fn put(&self, c: u8) -> Result<(), SerialError>;
    fn can_get(&self) -> bool;
    fn get(&self) -> Result<u8, SerialError>;
    fn get_irq_event(&self) -> IrqEvent;
    fn clean_irq_event(&self, event: IrqEvent);
}

pub struct Serial<R: Registers> {
    registers: R,
    tx: ChData,
    rx: ChData,
    pub irq_handler: Option<IrqHandler<R>>,
}

impl<R: Registers> Serial<R> {
    pub fn new(registers: R) -> Self {
        let tx = ChData::new();
        let rx = ChData::new();

        Self {
            registers: registers.clone(),
            tx: tx.clone(),
            rx: rx.clone(),
            irq_handler: Some(IrqHandler { registers, tx, rx }),
        }
    }

    pub fn try_take_tx(&mut self) -> Option<Sender<R>> {
        self.tx.try_take()?;

        Some(Sender {
            registers: self.registers.clone(),
            data: self.tx.clone(),
        })
    }

    pub fn try_take_rx(&mut self) -> Option<Receiver<R>> {
        self.rx.try_take()?;

        Some(Receiver {
            registers: self.registers.clone(),
            data: self.rx.clone(),
        })
    }

    /// Returns the irq state of this [`Serial`].
    ///
    /// # Safety
    ///
    /// Only used in interrupt handler.
    pub unsafe fn get_irq_event(&self) -> IrqEvent {
        self.registers.get_irq_event()
    }

    /// Cleans the irq state of this [`Serial`].
    ///
    /// # Safety
    ///
    /// Only used in interrupt handler.
    pub unsafe fn clean_irq_event(&self, state: IrqEvent) {
        self.registers.clean_irq_event(state);
    }
}

impl<R: Registers> DriverGeneric for Serial<R> {
    fn open(&mut self) -> Result<(), ErrorBase> {
        Ok(())
    }

    fn close(&mut self) -> Result<(), ErrorBase> {
        Ok(())
    }
}

impl<R: Registers> Interface for Serial<R> {
    fn handle_irq(&mut self) {
        unsafe { self.irq_handler.as_mut().unwrap().handle_irq() };
    }

    fn take_tx(&mut self) -> Option<Box<dyn rdif_serial::Sender>> {
        Some(Box::new(self.try_take_tx()?))
    }

    fn take_rx(&mut self) -> Option<Box<dyn rdif_serial::Reciever>> {
        Some(Box::new(self.try_take_rx()?))
    }
}

impl<R: Registers> rdif_serial::Sender for Sender<R> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, SerialError> {
        Sender::write(self, buf)
    }

    fn write_all<'a>(
        &'a mut self,
        buf: &'a [u8],
    ) -> rdif_serial::LocalBoxFuture<'a, Result<(), SerialError>> {
        Sender::write_all(self, buf).boxed_local()
    }
}

impl<R: Registers> rdif_serial::Reciever for Receiver<R> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, SerialError> {
        Receiver::read(self, buf)
    }

    fn read_all<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> rdif_serial::LocalBoxFuture<'a, Result<(), SerialError>> {
        Receiver::read_all(self, buf).boxed_local()
    }
}

pub struct IrqHandler<R: Registers> {
    registers: R,
    tx: ChData,
    rx: ChData,
}

unsafe impl<R: Registers> Sync for IrqHandler<R> {}

impl<R: Registers> IrqHandler<R> {
    /// Handle interrupt
    ///
    /// #  Safety
    /// Only used in interrupt handler.
    pub unsafe fn handle_irq(&self) {
        let state = self.registers.get_irq_event();

        if state.can_get {
            self.rx.waker.wake();
        }
        if state.can_put {
            self.tx.waker.wake();
        }

        self.registers.clean_irq_event(state);
    }
}

unsafe impl<R: Registers> Send for Serial<R> {}
unsafe impl<R: Registers> Send for Sender<R> {}
unsafe impl<R: Registers> Send for Receiver<R> {}

#[derive(Clone)]
struct ChData {
    taken: Arc<AtomicBool>,
    waker: Arc<AtomicWaker>,
}

impl ChData {
    fn new() -> Self {
        Self {
            taken: Arc::new(AtomicBool::new(false)),
            waker: Arc::new(AtomicWaker::new()),
        }
    }

    fn try_take(&self) -> Option<()> {
        match self
            .taken
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        {
            Ok(taken) => {
                if taken {
                    None
                } else {
                    Some(())
                }
            }
            Err(_) => None,
        }
    }
}

pub struct Sender<R: Registers> {
    registers: R,
    data: ChData,
}

impl<R: Registers> Sender<R> {
    pub fn write(&mut self, buf: &[u8]) -> Result<usize, SerialError> {
        let mut written = 0;
        for &byte in buf {
            if !self.registers.can_put() {
                break;
            }
            self.registers.put(byte)?;
            written += 1;
        }
        Ok(written)
    }

    pub fn can_put(&self) -> bool {
        self.registers.can_put()
    }

    pub fn write_all<'a>(
        &'a mut self,
        buf: &'a [u8],
    ) -> impl Future<Output = Result<(), SerialError>> + 'a {
        WaitForWriteAll {
            waiter: self.data.waker.clone(),
            sender: self,
            buf,
        }
    }
}

impl<R: Registers> Drop for Sender<R> {
    fn drop(&mut self) {
        self.data.taken.store(false, Ordering::Release);
    }
}

pub struct Receiver<R: Registers> {
    registers: R,
    data: ChData,
}

impl<R: Registers> Receiver<R> {
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, SerialError> {
        let mut read = 0;
        for byte in buf {
            if !self.registers.can_get() {
                break;
            }
            *byte = self.registers.get()?;
            read += 1;
        }
        Ok(read)
    }

    pub fn read_all<'a>(
        &'a mut self,
        buf: &'a mut [u8],
    ) -> impl Future<Output = Result<(), SerialError>> + 'a {
        WaitForReadAll {
            waiter: self.data.waker.clone(),
            rx: self,
            buf,
            i: 0,
        }
    }
}

impl<R: Registers> Drop for Receiver<R> {
    fn drop(&mut self) {
        self.data.taken.store(false, Ordering::Release);
    }
}

struct WaitForWriteAll<'a, R: Registers> {
    waiter: Arc<AtomicWaker>,
    sender: &'a mut Sender<R>,
    buf: &'a [u8],
}

impl<R: Registers> Future for WaitForWriteAll<'_, R> {
    type Output = Result<(), SerialError>;

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        self.waiter.register(cx.waker());

        let buf = self.buf;
        match self.sender.write(buf) {
            Ok(n) => {
                self.buf = &buf[n..];
                if n < buf.len() {
                    Poll::Pending
                } else {
                    Poll::Ready(Ok(()))
                }
            }
            Err(e) => Poll::Ready(Err(e)),
        }
    }
}

struct WaitForReadAll<'a, R: Registers> {
    waiter: Arc<AtomicWaker>,
    rx: &'a mut Receiver<R>,
    buf: &'a mut [u8],
    i: usize,
}

impl<R: Registers> Future for WaitForReadAll<'_, R> {
    type Output = Result<(), SerialError>;

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        self.waiter.register(cx.waker());
        let begin = self.i;
        for i in begin..self.buf.len() {
            if self.rx.registers.can_get() {
                match self.rx.registers.get() {
                    Ok(b) => {
                        self.buf[i] = b;
                        self.i += 1;
                    }
                    Err(e) => {
                        return Poll::Ready(Err(e));
                    }
                }
            } else {
                return Poll::Pending;
            }
        }
        Poll::Ready(Ok(()))
    }
}
