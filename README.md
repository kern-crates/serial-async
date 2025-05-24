# Uart Async

A lightweight and flexible UART driver library that provides register-level access through generic interfaces. By implementing these interfaces, users can create asynchronous UART drivers for various platforms with ease.

## 📌 Features

- **Register-Level Abstraction**: Offers low-level register-like control while maintaining usability.
- **Asynchronous Read/Write Support**: Enables non-blocking data transmission and reception using interrupts or DMA.
- **Highly Portable**: Abstracted hardware layer allows easy adaptation to different MCUs or SoCs.
- **Modular Design**: Clean architecture suitable for integration into existing embedded projects.
- **Full-Duplex Communication**: Supports simultaneous sending and receiving, automatically controlling the lifecycle of the transmitter and receiver.

## 🧩 Core Interface Overview

To use this library, implement the following device-specific interface functions:

```rust
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

Then you can use it like this:

```rust
static HANDLER: OnceCell<IrqHandler> = OnceCell::new();

// Your interrupt handler
fn handle_irq() {
    HANDLER.get().unwrap().handle_irq();
}

async fn use() {
    let mut uart = Uart::new(your_registers_impl);

    let handler = uart.irq_handler.take().unwrap();
    HANDLER.set(handler).unwrap();

    // Will fail if the TX is taken.
    let mut tx = uart.try_take_tx().unwrap();

    tx.write_all(b"hello!").await.unwrap();

    // Tx will give back to uart.
    drop(tx);

    // Can take tx again.
    let mut tx = uart.try_take_tx().unwrap();
}
```

## 🤝 Contributing

Contributions are welcome! 

## 📜 License

This project is licensed under the MIT License.
