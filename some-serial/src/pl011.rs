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

// const IMSC: usize = 0x038 / 4;
const RIS: usize = 0x03C / 4;
const MIS: usize = 0x040 / 4;
const ICR: usize = 0x044 / 4;

pub type Pl011 = Serial<Impl>;

pub fn new(mmio: usize) -> Pl011 {
    Pl011::new(Impl { mmio: Mmio(mmio) })
}

#[derive(Clone)]
pub struct Impl {
    mmio: Mmio,
}

impl Registers for Impl {
    fn can_put(&self) -> bool {
        const TXFF: u8 = 1 << 5;
        self.mmio.read::<u8>(0x18) & TXFF == 0
    }

    fn put(&self, c: u8) -> Result<(), SerialError> {
        self.mmio.write(0, c);
        Ok(())
    }

    fn can_get(&self) -> bool {
        const RXFE: u8 = 0x10;
        self.mmio.read::<u8>(0x18) & RXFE == 0
    }

    fn get(&self) -> Result<u8, SerialError> {
        let data: u32 = self.mmio.read(0);

        if data & 0xFFFFFF00 != 0 {
            // Clear the error
            self.mmio.write(1, 0xFFFFFFFFu32);
            return Err(SerialError::Other);
        }

        Ok(data as _)
    }

    fn get_irq_event(&self) -> IrqEvent {
        let mut event = IrqEvent::default();

        let ris: u32 = self.mmio.read(RIS);
        let mis: u32 = self.mmio.read(MIS);

        let sts = Interrupts::from_bits_retain(ris & mis);

        if sts.contains(Interrupts::RXI) {
            event.can_get = true;
        }

        if sts.contains(Interrupts::TXI) {
            event.can_put = true;
        }
        event
    }

    fn clean_irq_event(&self, event: IrqEvent) {
        let mut irqs = Interrupts::empty();
        if event.can_get {
            irqs |= Interrupts::RXI
        }

        if event.can_put {
            irqs |= Interrupts::TXI
        }

        self.mmio.write(ICR, irqs.bits());
    }

    fn enable(&self) {}

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

    fn loopback_enable(&self) {}
    fn loopback_disable(&self) {}
}
