#![cfg_attr(not(test), no_std)]

extern crate alloc;

use alloc::sync::Arc;
use core::{
    sync::atomic::{AtomicBool, Ordering},
    task::Poll,
};
use futures::task::AtomicWaker;

#[derive(Debug, Clone, Copy, Default)]
pub struct IrqEvent {
    pub can_get: bool,
    pub can_put: bool,
}

/// Serial error kind.
///
/// This represents a common set of serial operation errors. HAL implementations are
/// free to define more specific or additional error types. However, by providing
/// a mapping to these common serial errors, generic code can still react to them.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[non_exhaustive]
pub enum ErrorKind {
    /// The peripheral receive buffer was overrun.
    Overrun,
    /// Received data does not conform to the peripheral configuration.
    /// Can be caused by a misconfigured device on either end of the serial line.
    FrameFormat,
    /// Parity check failed.
    Parity,
    /// Serial line is too noisy to read valid data.
    Noise,
    /// Device was closed.
    Closed,
    /// A different error occurred. The original error may contain more information.
    Other,
}

impl core::error::Error for ErrorKind {}

impl core::fmt::Display for ErrorKind {
    #[inline]
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Overrun => write!(f, "The peripheral receive buffer was overrun"),
            Self::Parity => write!(f, "Parity check failed"),
            Self::Noise => write!(f, "Serial line is too noisy to read valid data"),
            Self::FrameFormat => write!(
                f,
                "Received data does not conform to the peripheral configuration"
            ),
            Self::Closed => write!(f, "Device was closed"),
            Self::Other => write!(
                f,
                "A different error occurred. The original error may contain more information"
            ),
        }
    }
}

pub trait Registers: Clone + 'static {
    fn can_put(&self) -> bool;
    fn put(&self, c: u8) -> Result<(), ErrorKind>;
    fn can_get(&self) -> bool;
    fn get(&self) -> Result<u8, ErrorKind>;
    fn set_irq_enable(&self, enable: bool);
    fn get_irq_enable(&self) -> bool;
    fn get_irq_event(&self) -> IrqEvent;
    fn clean_irq_event(&self, event: IrqEvent);
}

pub struct Uart<R: Registers> {
    registers: R,
    tx: ChData,
    rx: ChData,
    pub irq_handler: Option<IrqHandler<R>>,
}

impl<R: Registers> Uart<R> {
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

    /// Returns the irq state of this [`Uart`].
    ///
    /// # Safety
    ///
    /// Only used in interrupt handler.
    pub unsafe fn get_irq_event(&self) -> IrqEvent {
        self.registers.get_irq_event()
    }

    /// Cleans the irq state of this [`Uart`].
    ///
    /// # Safety
    ///
    /// Only used in interrupt handler.
    pub unsafe fn clean_irq_event(&self, state: IrqEvent) {
        self.registers.clean_irq_event(state);
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

unsafe impl<R: Registers> Send for Uart<R> {}
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
    pub fn write(&mut self, buf: &[u8]) -> Result<usize, ErrorKind> {
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

    pub fn write_all(&mut self, buf: &[u8]) -> impl Future<Output = Result<(), ErrorKind>> {
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
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, ErrorKind> {
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

    pub fn read_all(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<(), ErrorKind>> {
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

struct WaitForWriteAll<'a, 'b, R: Registers> {
    waiter: Arc<AtomicWaker>,
    sender: &'a mut Sender<R>,
    buf: &'b [u8],
}

impl<'a, 'b, R: Registers> Future for WaitForWriteAll<'a, 'b, R> {
    type Output = Result<(), ErrorKind>;

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

struct WaitForReadAll<'a, 'b, R: Registers> {
    waiter: Arc<AtomicWaker>,
    rx: &'a mut Receiver<R>,
    buf: &'b mut [u8],
    i: usize,
}

impl<'a, 'b, R: Registers> Future for WaitForReadAll<'a, 'b, R> {
    type Output = Result<(), ErrorKind>;

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
