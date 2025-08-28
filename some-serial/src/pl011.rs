use serial_async::*;

use tock_registers::{interfaces::*, register_bitfields, register_structs, registers::*};

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

register_bitfields! [
    u32,

    /// Data Register
    UARTDR [
        DATA OFFSET(0) NUMBITS(8) [],
        FE OFFSET(8) NUMBITS(1) [],
        PE OFFSET(9) NUMBITS(1) [],
        BE OFFSET(10) NUMBITS(1) [],
        OE OFFSET(11) NUMBITS(1) []
    ],

    /// Receive Status Register / Error Clear Register
    UARTRSR_ECR [
        FE OFFSET(0) NUMBITS(1) [],
        PE OFFSET(1) NUMBITS(1) [],
        BE OFFSET(2) NUMBITS(1) [],
        OE OFFSET(3) NUMBITS(1) []
    ],

    /// Flag Register
    UARTFR [
        CTS OFFSET(0) NUMBITS(1) [],
        DSR OFFSET(1) NUMBITS(1) [],
        DCD OFFSET(2) NUMBITS(1) [],
        BUSY OFFSET(3) NUMBITS(1) [],
        RXFE OFFSET(4) NUMBITS(1) [],
        TXFF OFFSET(5) NUMBITS(1) [],
        RXFF OFFSET(6) NUMBITS(1) [],
        TXFE OFFSET(7) NUMBITS(1) [],
        RI OFFSET(8) NUMBITS(1) []
    ],

    /// Integer Baud Rate Register
    UARTIBRD [
        BAUD_DIVINT OFFSET(0) NUMBITS(16) []
    ],

    /// Fractional Baud Rate Register
    UARTFBRD [
        BAUD_DIVFRAC OFFSET(0) NUMBITS(6) []
    ],

    /// Line Control Register
    UARTLCR_H [
        BRK OFFSET(0) NUMBITS(1) [],
        PEN OFFSET(1) NUMBITS(1) [],
        EPS OFFSET(2) NUMBITS(1) [],
        STP2 OFFSET(3) NUMBITS(1) [],
        FEN OFFSET(4) NUMBITS(1) [],
        WLEN OFFSET(5) NUMBITS(2) [
            FiveBit = 0,
            SixBit = 1,
            SevenBit = 2,
            EightBit = 3
        ],
        SPS OFFSET(7) NUMBITS(1) []
    ],

    /// Control Register
    UARTCR [
        UARTEN OFFSET(0) NUMBITS(1) [],
        SIREN OFFSET(1) NUMBITS(1) [],
        SIRLP OFFSET(2) NUMBITS(1) [],
        LBE OFFSET(7) NUMBITS(1) [],
        TXE OFFSET(8) NUMBITS(1) [],
        RXE OFFSET(9) NUMBITS(1) [],
        DTR OFFSET(10) NUMBITS(1) [],
        RTS OFFSET(11) NUMBITS(1) [],
        OUT1 OFFSET(12) NUMBITS(1) [],
        OUT2 OFFSET(13) NUMBITS(1) [],
        RTSEN OFFSET(14) NUMBITS(1) [],
        CTSEN OFFSET(15) NUMBITS(1) []
    ],

    /// Interrupt FIFO Level Select Register
    UARTIFLS [
        TXIFLSEL OFFSET(0) NUMBITS(3) [],
        RXIFLSEL OFFSET(3) NUMBITS(3) []
    ],

    /// Interrupt Mask Set/Clear Register
    UARTIMSC [
        RIMIM OFFSET(0) NUMBITS(1) [],
        CTSMIM OFFSET(1) NUMBITS(1) [],
        DCDMIM OFFSET(2) NUMBITS(1) [],
        DSRMIM OFFSET(3) NUMBITS(1) [],
        RXIM OFFSET(4) NUMBITS(1) [],
        TXIM OFFSET(5) NUMBITS(1) [],
        RTIM OFFSET(6) NUMBITS(1) [],
        FEIM OFFSET(7) NUMBITS(1) [],
        PEIM OFFSET(8) NUMBITS(1) [],
        BEIM OFFSET(9) NUMBITS(1) [],
        OEIM OFFSET(10) NUMBITS(1) []
    ],

    /// Raw Interrupt Status Register
    UARTRIS [
        RIRMIS OFFSET(0) NUMBITS(1) [],
        CTSRMIS OFFSET(1) NUMBITS(1) [],
        DCDRMIS OFFSET(2) NUMBITS(1) [],
        DSRRMIS OFFSET(3) NUMBITS(1) [],
        RXRIS OFFSET(4) NUMBITS(1) [],
        TXRIS OFFSET(5) NUMBITS(1) [],
        RTRIS OFFSET(6) NUMBITS(1) [],
        FERIS OFFSET(7) NUMBITS(1) [],
        PERIS OFFSET(8) NUMBITS(1) [],
        BERIS OFFSET(9) NUMBITS(1) [],
        OERIS OFFSET(10) NUMBITS(1) []
    ],

    /// Masked Interrupt Status Register
    UARTMIS [
        RIMMIS OFFSET(0) NUMBITS(1) [],
        CTSMIS OFFSET(1) NUMBITS(1) [],
        DCDMIS OFFSET(2) NUMBITS(1) [],
        DSRMIS OFFSET(3) NUMBITS(1) [],
        RXMIS OFFSET(4) NUMBITS(1) [],
        TXMIS OFFSET(5) NUMBITS(1) [],
        RTMIS OFFSET(6) NUMBITS(1) [],
        FEMIS OFFSET(7) NUMBITS(1) [],
        PEMIS OFFSET(8) NUMBITS(1) [],
        BEMIS OFFSET(9) NUMBITS(1) [],
        OEMIS OFFSET(10) NUMBITS(1) []
    ],

    /// Interrupt Clear Register
    UARTICR [
        RIMIC OFFSET(0) NUMBITS(1) [],
        CTSMIC OFFSET(1) NUMBITS(1) [],
        DCDMIC OFFSET(2) NUMBITS(1) [],
        DSRMIC OFFSET(3) NUMBITS(1) [],
        RXIC OFFSET(4) NUMBITS(1) [],
        TXIC OFFSET(5) NUMBITS(1) [],
        RTIC OFFSET(6) NUMBITS(1) [],
        FEIC OFFSET(7) NUMBITS(1) [],
        PEIC OFFSET(8) NUMBITS(1) [],
        BEIC OFFSET(9) NUMBITS(1) [],
        OEIC OFFSET(10) NUMBITS(1) []
    ],

    /// DMA Control Register
    UARTDMACR [
        RXDMAE OFFSET(0) NUMBITS(1) [],
        TXDMAE OFFSET(1) NUMBITS(1) [],
        DMAONERR OFFSET(2) NUMBITS(1) []
    ]
];

register_structs! {
    pub Pl011Registers {
        (0x000 => uartdr: ReadWrite<u32, UARTDR::Register>),
        (0x004 => uartrsr_ecr: ReadWrite<u32, UARTRSR_ECR::Register>),
        (0x008 => _reserved1),
        (0x018 => uartfr: ReadOnly<u32, UARTFR::Register>),
        (0x01c => _reserved2),
        (0x020 => uartilpr: ReadWrite<u32>),
        (0x024 => uartibrd: ReadWrite<u32, UARTIBRD::Register>),
        (0x028 => uartfbrd: ReadWrite<u32, UARTFBRD::Register>),
        (0x02c => uartlcr_h: ReadWrite<u32, UARTLCR_H::Register>),
        (0x030 => uartcr: ReadWrite<u32, UARTCR::Register>),
        (0x034 => uartifls: ReadWrite<u32, UARTIFLS::Register>),
        (0x038 => uartimsc: ReadWrite<u32, UARTIMSC::Register>),
        (0x03c => uartris: ReadOnly<u32, UARTRIS::Register>),
        (0x040 => uartmis: ReadOnly<u32, UARTMIS::Register>),
        (0x044 => uarticr: WriteOnly<u32, UARTICR::Register>),
        (0x048 => uartdmacr: ReadWrite<u32, UARTDMACR::Register>),
        (0x04c => _reserved3),
        (0x1000 => @END),
    }
}

// SAFETY: PL011 寄存器访问是原子的，硬件保证了内存映射寄存器的线程安全
unsafe impl Sync for Pl011Registers {}

pub type Pl011 = Serial<Impl>;

pub fn new(mmio: usize, clk_freq: usize) -> Pl011 {
    Pl011::new(Impl {
        clk_freq,
        registers: unsafe { &*(mmio as *const Pl011Registers) },
    })
}

#[derive(Clone)]
pub struct Impl {
    clk_freq: usize,
    registers: &'static Pl011Registers,
}

// SAFETY: PL011 寄存器访问是原子的，可以安全地在线程间共享
unsafe impl Sync for Impl {}

impl Registers for Impl {
    fn can_put(&self) -> bool {
        use tock_registers::interfaces::Readable;
        !self.registers.uartfr.is_set(UARTFR::TXFF)
    }

    fn put(&self, c: u8) -> Result<(), SerialError> {
        use tock_registers::interfaces::Writeable;
        self.registers.uartdr.write(UARTDR::DATA.val(c as u32));
        Ok(())
    }

    fn can_get(&self) -> bool {
        use tock_registers::interfaces::Readable;
        !self.registers.uartfr.is_set(UARTFR::RXFE)
    }

    fn get(&self) -> Result<u8, SerialError> {
        use tock_registers::interfaces::{Readable, Writeable};
        let data = self.registers.uartdr.get();

        if data & 0xFFFFFF00 != 0 {
            // Clear the error
            self.registers.uartrsr_ecr.set(0xFFFFFFFF);
            return Err(SerialError::Other);
        }

        Ok(data as u8)
    }

    fn get_irq_event(&self) -> IrqEvent {
        let mut event = IrqEvent::default();

        let ris = self.registers.uartris.get();
        let mis = self.registers.uartmis.get();

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

        self.registers.uarticr.set(irqs.bits());
    }

    fn enable(&self) {
        // 设置UARTEN位来启用UART

        self.registers.uartcr.modify(UARTCR::UARTEN::SET);
    }

    fn disable(&self) {
        // 清除UARTEN位来禁用UART

        self.registers.uartcr.modify(UARTCR::UARTEN::CLEAR);
    }

    fn tx_enable(&self) {
        // 设置TXE位来启用发送

        self.registers.uartcr.modify(UARTCR::TXE::SET);
    }

    fn tx_disable(&self) {
        // 清除TXE位来禁用发送

        self.registers.uartcr.modify(UARTCR::TXE::CLEAR);
    }

    fn rx_enable(&self) {
        // 设置RXE位来启用接收

        self.registers.uartcr.modify(UARTCR::RXE::SET);
    }

    fn rx_disable(&self) {
        // 清除RXE位来禁用接收

        self.registers.uartcr.modify(UARTCR::RXE::CLEAR);
    }

    fn loopback_enable(&self) {
        // 设置LBE位来启用环回模式

        self.registers.uartcr.modify(UARTCR::LBE::SET);
    }

    fn loopback_disable(&self) {
        // 清除LBE位来禁用环回模式

        self.registers.uartcr.modify(UARTCR::LBE::CLEAR);
    }

    fn set_baud_rate(&self, baud_rate: usize) -> Result<(), SerialError> {
        // PL011 的波特率计算公式：
        // BAUDDIV = (FUARTCLK / (16 * Baud rate))
        // IBRD = integer(BAUDDIV)
        // FBRD = integer((BAUDDIV - IBRD) * 64 + 0.5)
        //
        // 假设 FUARTCLK = 24MHz (常见的时钟频率)
        let uart_clk = self.clk_freq as u32;
        let baud = baud_rate as u32;

        // 计算整数和小数部分
        let bauddiv = uart_clk / (16 * baud);
        let fbrd = ((uart_clk % (16 * baud)) * 64 + (16 * baud / 2)) / (16 * baud);

        if bauddiv == 0 || bauddiv > 0xFFFF {
            return Err(SerialError::Other);
        }

        self.registers
            .uartibrd
            .write(UARTIBRD::BAUD_DIVINT.val(bauddiv));
        self.registers
            .uartfbrd
            .write(UARTFBRD::BAUD_DIVFRAC.val(fbrd));

        Ok(())
    }

    fn set_data_bits(&self, data_bits: DataBits) -> Result<(), SerialError> {
        let wlen = match data_bits {
            DataBits::Five => UARTLCR_H::WLEN::FiveBit,
            DataBits::Six => UARTLCR_H::WLEN::SixBit,
            DataBits::Seven => UARTLCR_H::WLEN::SevenBit,
            DataBits::Eight => UARTLCR_H::WLEN::EightBit,
        };

        self.registers.uartlcr_h.modify(wlen);
        Ok(())
    }

    fn set_stop_bits(&self, stop_bits: StopBits) -> Result<(), SerialError> {
        match stop_bits {
            StopBits::One => self.registers.uartlcr_h.modify(UARTLCR_H::STP2::CLEAR),
            StopBits::Two => self.registers.uartlcr_h.modify(UARTLCR_H::STP2::SET),
        }
        Ok(())
    }

    fn set_parity(&self, parity: Parity) -> Result<(), SerialError> {
        match parity {
            Parity::None => {
                // PEN = 0, 无奇偶校验
                self.registers.uartlcr_h.modify(UARTLCR_H::PEN::CLEAR);
            }
            Parity::Odd => {
                // PEN = 1, EPS = 0 (奇校验), SPS = 0
                self.registers
                    .uartlcr_h
                    .modify(UARTLCR_H::PEN::SET + UARTLCR_H::EPS::CLEAR + UARTLCR_H::SPS::CLEAR);
            }
            Parity::Even => {
                // PEN = 1, EPS = 1 (偶校验), SPS = 0
                self.registers
                    .uartlcr_h
                    .modify(UARTLCR_H::PEN::SET + UARTLCR_H::EPS::SET + UARTLCR_H::SPS::CLEAR);
            }
            Parity::Mark => {
                // PEN = 1, SPS = 1, EPS = 0 (奇校验)
                self.registers
                    .uartlcr_h
                    .modify(UARTLCR_H::PEN::SET + UARTLCR_H::EPS::CLEAR + UARTLCR_H::SPS::SET);
            }
            Parity::Space => {
                // PEN = 1, EPS = 1 (偶校验), SPS = 1
                self.registers
                    .uartlcr_h
                    .modify(UARTLCR_H::PEN::SET + UARTLCR_H::EPS::SET + UARTLCR_H::SPS::SET);
            }
        }

        Ok(())
    }

    fn set_flow_control(&self, flow_control: FlowControl) -> Result<(), SerialError> {
        match flow_control {
            FlowControl::None => {
                // 禁用流控制
                self.registers
                    .uartcr
                    .modify(UARTCR::RTSEN::CLEAR + UARTCR::CTSEN::CLEAR);
            }
            FlowControl::RtsCts => {
                // 启用RTS/CTS流控制
                self.registers
                    .uartcr
                    .modify(UARTCR::RTSEN::SET + UARTCR::CTSEN::SET);
            }
            FlowControl::XonXoff => {
                // PL011不直接支持XON/XOFF，需要在软件层实现
                return Err(SerialError::Other);
            }
        }

        Ok(())
    }

    fn dma_rx_enable(&self) {
        // 启用接收DMA：设置RXDMAE位
        self.registers.uartdmacr.modify(UARTDMACR::RXDMAE::SET);
    }

    fn dma_rx_disable(&self) {
        // 禁用接收DMA：清除RXDMAE位
        self.registers.uartdmacr.modify(UARTDMACR::RXDMAE::CLEAR);
    }

    fn dma_tx_enable(&self) {
        // 启用发送DMA：设置TXDMAE位
        self.registers.uartdmacr.modify(UARTDMACR::TXDMAE::SET);
    }

    fn dma_tx_disable(&self) {
        // 禁用发送DMA：清除TXDMAE位
        self.registers.uartdmacr.modify(UARTDMACR::TXDMAE::CLEAR);
    }

    fn dma_rx_enabled(&self) -> bool {
        // 检查RXDMAE位是否设置
        use tock_registers::interfaces::Readable;
        self.registers.uartdmacr.is_set(UARTDMACR::RXDMAE)
    }

    fn dma_tx_enabled(&self) -> bool {
        // 检查TXDMAE位是否设置
        use tock_registers::interfaces::Readable;
        self.registers.uartdmacr.is_set(UARTDMACR::TXDMAE)
    }

    fn dma_on_error_enable(&self) {
        // 启用错误时禁用DMA：设置DMAONERR位
        self.registers.uartdmacr.modify(UARTDMACR::DMAONERR::SET);
    }

    fn dma_on_error_disable(&self) {
        // 禁用错误时禁用DMA：清除DMAONERR位
        self.registers.uartdmacr.modify(UARTDMACR::DMAONERR::CLEAR);
    }

    fn dma_on_error_enabled(&self) -> bool {
        // 检查DMAONERR位是否设置
        use tock_registers::interfaces::Readable;
        self.registers.uartdmacr.is_set(UARTDMACR::DMAONERR)
    }
}

impl Impl {
    pub fn read_control_register(&self) -> u32 {
        use tock_registers::interfaces::Readable;
        self.registers.uartcr.get()
    }

    /// 读取DMA控制寄存器的值
    pub fn read_dma_control_register(&self) -> u32 {
        use tock_registers::interfaces::Readable;
        self.registers.uartdmacr.get()
    }

    /// 完整配置UART（会暂时禁用UART）
    pub fn configure_uart(&self, config: &UartConfig) -> Result<(), SerialError> {
        use tock_registers::interfaces::Readable;

        // 根据ARM文档的建议配置流程：
        // 1. 禁用UART
        let original_enable = self.registers.uartcr.is_set(UARTCR::UARTEN); // 保存原始使能状态
        self.registers.uartcr.modify(UARTCR::UARTEN::CLEAR); // 禁用UART

        // 2. 等待当前字符传输完成
        while self.registers.uartfr.is_set(UARTFR::BUSY) {
            // BUSY位
            core::hint::spin_loop();
        }

        // 3. 刷新发送FIFO（通过设置FEN=0）
        self.registers.uartlcr_h.modify(UARTLCR_H::FEN::CLEAR);

        // 4. 配置各项参数
        self.set_baud_rate(config.baud_rate)?;
        self.set_data_bits(config.data_bits)?;
        self.set_stop_bits(config.stop_bits)?;
        self.set_parity(config.parity)?;

        // 5. 重新启用FIFO
        self.registers.uartlcr_h.modify(UARTLCR_H::FEN::SET);

        // 6. 配置流控制
        self.set_flow_control(config.flow_control)?;

        // 7. 恢复UART使能状态
        if original_enable {
            self.registers.uartcr.modify(UARTCR::UARTEN::SET); // 重新启用UART
        }

        Ok(())
    }
}
