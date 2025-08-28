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

const RIS: usize = 0x03C / 4;
const MIS: usize = 0x040 / 4;
const ICR: usize = 0x044 / 4;
const CR: usize = 0x030 / 4;
const UARTDR: usize = 0x000 / 4;
const UARTECR: usize = 0x004 / 4;
const UARTFR: usize = 0x018 / 4;
const UARTIBRD: usize = 0x024 / 4; // Integer Baud Rate Register
const UARTFBRD: usize = 0x028 / 4; // Fractional Baud Rate Register
const UARTLCR_H: usize = 0x02C / 4; // Line Control Register

pub type Pl011 = Serial<Impl>;

pub fn new(mmio: usize, clk_freq: usize) -> Pl011 {
    Pl011::new(Impl {
        clk_freq,
        mmio: Mmio(mmio),
    })
}

#[derive(Clone)]
pub struct Impl {
    clk_freq: usize,
    mmio: Mmio,
}

impl Registers for Impl {
    fn can_put(&self) -> bool {
        const TXFF: u32 = 1 << 5;
        self.mmio.read::<u32>(UARTFR) & TXFF == 0
    }

    fn put(&self, c: u8) -> Result<(), SerialError> {
        self.mmio.write(UARTDR, c as u32);
        Ok(())
    }

    fn can_get(&self) -> bool {
        const RXFE: u32 = 0x10;
        self.mmio.read::<u32>(UARTFR) & RXFE == 0
    }

    fn get(&self) -> Result<u8, SerialError> {
        let data: u32 = self.mmio.read(UARTDR);

        if data & 0xFFFFFF00 != 0 {
            // Clear the error
            self.mmio.write(UARTECR, 0xFFFFFFFFu32);
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

    fn enable(&self) {
        // 设置UARTEN位来启用UART
        let mut cr: u32 = self.mmio.read(CR);
        cr |= 1 << 0; // 设置UARTEN位
        self.mmio.write(CR, cr);
    }

    fn disable(&self) {
        // 清除UARTEN位来禁用UART
        let mut cr: u32 = self.mmio.read(CR);
        cr &= !(1 << 0); // 清除UARTEN位
        self.mmio.write(CR, cr);
    }

    fn tx_enable(&self) {
        // 设置TXE位来启用发送
        let mut cr: u32 = self.mmio.read(CR);
        cr |= 1 << 8; // 设置TXE位
        self.mmio.write(CR, cr);
    }

    fn tx_disable(&self) {
        // 清除TXE位来禁用发送
        let mut cr: u32 = self.mmio.read(CR);
        cr &= !(1 << 8); // 清除TXE位
        self.mmio.write(CR, cr);
    }

    fn rx_enable(&self) {
        // 设置RXE位来启用接收
        let mut cr: u32 = self.mmio.read(CR);
        cr |= 1 << 9; // 设置RXE位
        self.mmio.write(CR, cr);
    }

    fn rx_disable(&self) {
        // 清除RXE位来禁用接收
        let mut cr: u32 = self.mmio.read(CR);
        cr &= !(1 << 9); // 清除RXE位
        self.mmio.write(CR, cr);
    }

    fn loopback_enable(&self) {
        // 设置LBE位来启用环回模式
        let mut cr: u32 = self.mmio.read(CR);
        cr |= 1 << 7; // 设置LBE位
        self.mmio.write(CR, cr);
    }

    fn loopback_disable(&self) {
        // 清除LBE位来禁用环回模式
        let mut cr: u32 = self.mmio.read(CR);
        cr &= !(1 << 7); // 清除LBE位
        self.mmio.write(CR, cr);
    }

    fn set_baud_rate(&self, baud_rate: BaudRate) -> Result<(), SerialError> {
        // PL011 的波特率计算公式：
        // BAUDDIV = (FUARTCLK / (16 * Baud rate))
        // IBRD = integer(BAUDDIV)
        // FBRD = integer((BAUDDIV - IBRD) * 64 + 0.5)
        //
        // 假设 FUARTCLK = 24MHz (常见的时钟频率)
        let uart_clk = self.clk_freq as u32;
        let baud = baud_rate.as_u32();

        // 计算整数和小数部分
        let bauddiv = uart_clk / (16 * baud);
        let fbrd = ((uart_clk % (16 * baud)) * 64 + (16 * baud / 2)) / (16 * baud);

        if bauddiv == 0 || bauddiv > 0xFFFF {
            return Err(SerialError::Other);
        }

        self.mmio.write(UARTIBRD, bauddiv);
        self.mmio.write(UARTFBRD, fbrd);

        Ok(())
    }

    fn set_data_bits(&self, data_bits: DataBits) -> Result<(), SerialError> {
        let mut lcr_h: u32 = self.mmio.read(UARTLCR_H);
        lcr_h &= !(0b11 << 5); // 清除WLEN位 [6:5]

        let wlen = match data_bits {
            DataBits::Five => 0b00,
            DataBits::Six => 0b01,
            DataBits::Seven => 0b10,
            DataBits::Eight => 0b11,
        };

        lcr_h |= wlen << 5;
        self.mmio.write(UARTLCR_H, lcr_h);

        Ok(())
    }

    fn set_stop_bits(&self, stop_bits: StopBits) -> Result<(), SerialError> {
        let mut lcr_h: u32 = self.mmio.read(UARTLCR_H);

        match stop_bits {
            StopBits::One => lcr_h &= !(1 << 3), // 清除STP2位
            StopBits::Two => lcr_h |= 1 << 3,    // 设置STP2位
        }

        self.mmio.write(UARTLCR_H, lcr_h);

        Ok(())
    }

    fn set_parity(&self, parity: Parity) -> Result<(), SerialError> {
        let mut lcr_h: u32 = self.mmio.read(UARTLCR_H);
        lcr_h &= !(0b111 << 1); // 清除PEN, EPS, SPS位 [3:1]

        match parity {
            Parity::None => {
                // PEN = 0, 无奇偶校验
            }
            Parity::Odd => {
                lcr_h |= 1 << 1; // PEN = 1
                // EPS = 0 (奇校验)
            }
            Parity::Even => {
                lcr_h |= 1 << 1; // PEN = 1
                lcr_h |= 1 << 2; // EPS = 1 (偶校验)
            }
            Parity::Mark => {
                lcr_h |= 1 << 1; // PEN = 1
                lcr_h |= 1 << 3; // SPS = 1
                // EPS = 0 (奇校验)
            }
            Parity::Space => {
                lcr_h |= 1 << 1; // PEN = 1
                lcr_h |= 1 << 2; // EPS = 1 (偶校验)
                lcr_h |= 1 << 3; // SPS = 1
            }
        }

        self.mmio.write(UARTLCR_H, lcr_h);

        Ok(())
    }

    fn set_flow_control(&self, flow_control: FlowControl) -> Result<(), SerialError> {
        let mut cr: u32 = self.mmio.read(CR);
        cr &= !(0b11 << 14); // 清除CTSEn和RTSEn位 [15:14]

        match flow_control {
            FlowControl::None => {
                // 无流控制，位已清除
            }
            FlowControl::RtsCts => {
                cr |= 1 << 14; // RTSEn = 1
                cr |= 1 << 15; // CTSEn = 1
            }
            FlowControl::XonXoff => {
                // PL011不直接支持XON/XOFF，需要在软件层实现
                return Err(SerialError::Other);
            }
        }

        self.mmio.write(CR, cr);

        Ok(())
    }
}

impl Impl {
    pub fn read_control_register(&self) -> u32 {
        self.mmio.read(CR)
    }

    /// 完整配置UART（会暂时禁用UART）
    pub fn configure_uart(&self, config: &UartConfig) -> Result<(), SerialError> {
        // 根据ARM文档的建议配置流程：
        // 1. 禁用UART
        let mut cr: u32 = self.mmio.read(CR);
        let original_enable = cr & 1; // 保存原始使能状态
        cr &= !1; // 禁用UART
        self.mmio.write(CR, cr);

        // 2. 等待当前字符传输完成
        while self.mmio.read::<u32>(UARTFR) & (1 << 3) != 0 {
            // BUSY位
            core::hint::spin_loop();
        }

        // 3. 刷新发送FIFO（通过设置FEN=0）
        let mut lcr_h: u32 = self.mmio.read(UARTLCR_H);
        lcr_h &= !(1 << 4); // FEN = 0
        self.mmio.write(UARTLCR_H, lcr_h);

        // 4. 配置各项参数
        self.set_baud_rate(config.baud_rate)?;
        self.set_data_bits(config.data_bits)?;
        self.set_stop_bits(config.stop_bits)?;
        self.set_parity(config.parity)?;

        // 5. 重新启用FIFO
        lcr_h = self.mmio.read(UARTLCR_H);
        lcr_h |= 1 << 4; // FEN = 1
        self.mmio.write(UARTLCR_H, lcr_h);

        // 6. 配置流控制
        self.set_flow_control(config.flow_control)?;

        // 7. 恢复UART使能状态
        if original_enable != 0 {
            cr = self.mmio.read(CR);
            cr |= 1; // 重新启用UART
            self.mmio.write(CR, cr);
        }

        Ok(())
    }
}
