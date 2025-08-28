use serial_async::*;

use crate::Mmio;

bitflags::bitflags! {
    struct Interrupts: u32 {
        /// Overrun error interrupt.
        const OEI = 1 << 10;
        /// Break error interrupt.
        const BEI = 1 << 9;
        /// Parity error interrupt.
        const PEI = 1 << 8;
        /// Framing error interrupt.
        const FEI = 1 << 7;
        /// Receive timeout interrupt.
        const RTI = 1 << 6;
        /// Transmit interrupt.
        const TXI = 1 << 5;
        /// Receive interrupt.
        const RXI = 1 << 4;
        /// nUARTDSR modem interrupt.
        const DSRMI = 1 << 3;
        /// nUARTDCD modem interrupt.
        const DCDMI = 1 << 2;
        /// nUARTCTS modem interrupt.
        const CTSMI = 1 << 1;
        /// nUARTRI modem interrupt.
        const RIMI = 1 << 0;
    }
}

pub type Ns16550 = Serial<Impl>;

pub fn new(base: usize) -> Ns16550 {
    Ns16550::new(Impl { base })
}

#[derive(Clone)]
pub struct Impl {
    base: usize,
}

impl Impl {
    fn read(&self, offset: usize) -> u8 {
        #[cfg(target_arch = "x86_64")]
        unsafe {
            x86_64::instructions::port::Port::<u8>::new((self.base + offset) as _).read()
        }
        #[cfg(not(target_arch = "x86_64"))]
        Mmio(self.base).read(offset)
    }

    fn write(&self, offset: usize, val: u8) {
        #[cfg(target_arch = "x86_64")]
        unsafe {
            x86_64::instructions::port::Port::<u8>::new((self.base + offset) as _).write(val)
        }
        #[cfg(not(target_arch = "x86_64"))]
        Mmio(self.base).write(offset, val)
    }

    fn modify(&self, offset: usize, f: impl FnOnce(u8) -> u8) {
        let v = self.read(offset);
        self.write(offset, f(v));
    }

    fn sts(&self) -> u8 {
        self.read(5)
    }
}

impl Registers for Impl {
    fn can_put(&self) -> bool {
        // Xmitter empty
        const LSR_TEMT: u8 = 1 << 6;
        self.sts() & LSR_TEMT != 0
    }

    fn put(&self, c: u8) -> Result<(), SerialError> {
        self.write(0, c);
        Ok(())
    }

    fn can_get(&self) -> bool {
        const LSR_DR: u8 = 1;
        self.sts() & LSR_DR != 0
    }

    fn get(&self) -> Result<u8, SerialError> {
        Ok(self.read(0))
    }

    fn get_irq_event(&self) -> IrqEvent {
        let sts = self.read(2);
        let mut event = IrqEvent::default();

        if sts & 1 != 0 {
            event.can_get = true;
        }

        if sts & 1 << 1 != 0 {
            event.can_put = true;
        }

        event
    }

    fn clean_irq_event(&self, event: IrqEvent) {}

    fn enable(&self) {
        todo!()
    }

    fn disable(&self) {
        todo!()
    }

    fn tx_enable(&self) {
        todo!()
    }

    fn tx_disable(&self) {
        todo!()
    }

    fn rx_enable(&self) {
        todo!()
    }

    fn rx_disable(&self) {
        todo!()
    }

    fn set_baud_rate(&self, baud_rate: usize) -> Result<(), ConfigError> {
        todo!()
    }

    fn set_data_bits(&self, data_bits: DataBits) -> Result<(), ConfigError> {
        todo!()
    }

    fn set_stop_bits(&self, stop_bits: StopBits) -> Result<(), ConfigError> {
        todo!()
    }

    fn set_parity(&self, parity: Parity) -> Result<(), ConfigError> {
        todo!()
    }

    fn set_flow_control(&self, flow_control: FlowControl) -> Result<(), ConfigError> {
        todo!()
    }
}
