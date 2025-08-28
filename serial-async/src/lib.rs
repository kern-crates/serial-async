#![cfg_attr(target_os = "none", no_std)]

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

/// 数据位数
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataBits {
    Five,
    Six,
    Seven,
    Eight,
}

/// 停止位数
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    One,
    Two,
}

/// 奇偶校验
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    None,
    Odd,
    Even,
    Mark,
    Space,
}

/// 流控制
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowControl {
    None,
    RtsCts,
    XonXoff,
}

/// UART配置
#[derive(Debug, Clone, Copy)]
pub struct UartConfig {
    pub baud_rate: usize,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
    pub flow_control: FlowControl,
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baud_rate: 115200,
            data_bits: DataBits::Eight,
            stop_bits: StopBits::One,
            parity: Parity::None,
            flow_control: FlowControl::None,
        }
    }
}

pub trait Registers: Send + Clone + 'static {
    fn can_put(&self) -> bool;
    fn put(&self, c: u8) -> Result<(), SerialError>;
    fn can_get(&self) -> bool;
    fn get(&self) -> Result<u8, SerialError>;
    fn get_irq_event(&self) -> IrqEvent;
    fn clean_irq_event(&self, event: IrqEvent);

    fn enable(&self);
    fn disable(&self);

    fn tx_enable(&self);
    fn tx_disable(&self);
    fn rx_enable(&self);
    fn rx_disable(&self);

    fn loopback_enable(&self) {}
    fn loopback_disable(&self) {}

    /// 配置UART参数
    fn configure(&self, config: &UartConfig) -> Result<(), SerialError> {
        self.set_baud_rate(config.baud_rate)?;
        self.set_data_bits(config.data_bits)?;
        self.set_stop_bits(config.stop_bits)?;
        self.set_parity(config.parity)?;
        self.set_flow_control(config.flow_control)?;
        Ok(())
    }

    /// 设置波特率
    fn set_baud_rate(&self, baud_rate: usize) -> Result<(), SerialError>;

    /// 设置数据位数
    fn set_data_bits(&self, data_bits: DataBits) -> Result<(), SerialError>;

    /// 设置停止位数
    fn set_stop_bits(&self, stop_bits: StopBits) -> Result<(), SerialError>;

    /// 设置奇偶校验
    fn set_parity(&self, parity: Parity) -> Result<(), SerialError>;

    /// 设置流控制
    fn set_flow_control(&self, flow_control: FlowControl) -> Result<(), SerialError>;

    /// 启用DMA接收
    fn dma_rx_enable(&self) {}

    /// 禁用DMA接收
    fn dma_rx_disable(&self) {}

    /// 启用DMA发送
    fn dma_tx_enable(&self) {}

    /// 禁用DMA发送
    fn dma_tx_disable(&self) {}

    /// 检查DMA接收是否启用
    fn dma_rx_enabled(&self) -> bool {
        false
    }

    /// 检查DMA发送是否启用
    fn dma_tx_enabled(&self) -> bool {
        false
    }

    /// 启用错误时禁用DMA
    fn dma_on_error_enable(&self) {}

    /// 禁用错误时禁用DMA
    fn dma_on_error_disable(&self) {}

    /// 检查错误时禁用DMA是否启用
    fn dma_on_error_enabled(&self) -> bool {
        false
    }
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
        self.registers.tx_enable();

        Some(Sender {
            registers: self.registers.clone(),
            data: self.tx.clone(),
        })
    }

    pub fn try_take_rx(&mut self) -> Option<Receiver<R>> {
        self.rx.try_take()?;
        self.registers.rx_enable();

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

    pub fn loopback_enable(&self) {
        self.registers.loopback_enable();
    }

    pub fn loopback_disable(&self) {
        self.registers.loopback_disable();
    }

    pub fn enable(&self) {
        self.registers.enable();
    }

    pub fn disable(&self) {
        self.registers.disable();
    }

    /// 用于调试的方法：尝试清空接收FIFO
    pub fn flush_rx(&self) -> usize {
        let mut count = 0;
        while self.registers.can_get() && count < 100 {
            if self.registers.get().is_ok() {
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    /// 用于调试的方法：获取原始寄存器访问
    pub fn registers(&self) -> &R {
        &self.registers
    }

    /// 配置UART参数
    pub fn configure(&self, config: &UartConfig) -> Result<(), SerialError> {
        self.registers.configure(config)
    }

    /// 设置波特率
    pub fn set_baud_rate(&self, baud_rate: usize) -> Result<(), SerialError> {
        self.registers.set_baud_rate(baud_rate)
    }

    /// 设置数据位数
    pub fn set_data_bits(&self, data_bits: DataBits) -> Result<(), SerialError> {
        self.registers.set_data_bits(data_bits)
    }

    /// 设置停止位数
    pub fn set_stop_bits(&self, stop_bits: StopBits) -> Result<(), SerialError> {
        self.registers.set_stop_bits(stop_bits)
    }

    /// 设置奇偶校验
    pub fn set_parity(&self, parity: Parity) -> Result<(), SerialError> {
        self.registers.set_parity(parity)
    }

    /// 设置流控制
    pub fn set_flow_control(&self, flow_control: FlowControl) -> Result<(), SerialError> {
        self.registers.set_flow_control(flow_control)
    }

    /// 启用DMA接收
    pub fn dma_rx_enable(&self) {
        self.registers.dma_rx_enable();
    }

    /// 禁用DMA接收
    pub fn dma_rx_disable(&self) {
        self.registers.dma_rx_disable();
    }

    /// 启用DMA发送
    pub fn dma_tx_enable(&self) {
        self.registers.dma_tx_enable();
    }

    /// 禁用DMA发送
    pub fn dma_tx_disable(&self) {
        self.registers.dma_tx_disable();
    }

    /// 检查DMA接收是否启用
    pub fn dma_rx_enabled(&self) -> bool {
        self.registers.dma_rx_enabled()
    }

    /// 检查DMA发送是否启用
    pub fn dma_tx_enabled(&self) -> bool {
        self.registers.dma_tx_enabled()
    }

    /// 启用错误时禁用DMA
    pub fn dma_on_error_enable(&self) {
        self.registers.dma_on_error_enable();
    }

    /// 禁用错误时禁用DMA
    pub fn dma_on_error_disable(&self) {
        self.registers.dma_on_error_disable();
    }

    /// 检查错误时禁用DMA是否启用
    pub fn dma_on_error_enabled(&self) -> bool {
        self.registers.dma_on_error_enabled()
    }
}

impl<R: Registers> DriverGeneric for Serial<R> {
    fn open(&mut self) -> Result<(), ErrorBase> {
        self.enable();
        Ok(())
    }

    fn close(&mut self) -> Result<(), ErrorBase> {
        self.disable();
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

    pub fn write_all_blocking(&mut self, buf: &[u8]) -> Result<(), SerialError> {
        let mut written = 0;
        while written < buf.len() {
            let n = self.write(&buf[written..])?;
            written += n;
        }
        Ok(())
    }
}

impl<R: Registers> Drop for Sender<R> {
    fn drop(&mut self) {
        self.registers.tx_disable();
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

    pub fn can_get(&self) -> bool {
        self.registers.can_get()
    }

    pub fn get(&mut self) -> Result<u8, SerialError> {
        self.registers.get()
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
        self.registers.rx_disable();
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
