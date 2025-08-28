# Serial Async

A lightweight and flexible serial driver library that provides register-level access through generic interfaces. By implementing these interfaces, users can create asynchronous serial drivers for various platforms with ease.

## 📌 Features

- **Register-Level Abstraction**: Offers low-level register-like control while maintaining usability.
- **Asynchronous Read/Write Support**: Enables non-blocking data transmission and reception using interrupts and device FIFO.
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
    fn get_irq_event(&self) -> IrqEvent;
    fn clean_irq_event(&self, event: IrqEvent);
}
```

Then you can use it like this:

```rust
static HANDLER: OnceCell<IrqHandler> = OnceCell::new();

// Your interrupt handler
fn handle_irq() {
    HANDLER.get().unwrap().handle_irq();
}

async fn use() {
    let mut serial = Serial::new(your_registers_impl);

    let handler = serial.irq_handler.take().unwrap();
    HANDLER.set(handler).unwrap();

    // If the TX has already been taken, it will fail.
    let mut tx = serial.try_take_tx().unwrap();
    let mut rx = serial.try_take_rx().unwrap();

    tx.write_all(b"hello!").await.unwrap();

    // The TX will be given back to serial.
    drop(tx);

    // Can take tx again.
    let mut tx = serial.try_take_tx().unwrap();
}
```

## 🤝 Contributing

Contributions are welcome! 

## 📜 License

This project is licensed under the MIT License.
