# HP RedEye IR Receiver (RP2040 → UART)

This repository contains an implementation of the **HP RedEye infrared protocol (receiver)** using the **RP2040** microcontroller.  
The firmware receives IR data from an HP calculator, decodes the RedEye protocol, and forwards the decoded data to **UART0**.

The project was developed and tested with an **HP 28C** calculator, but it may work with other HP devices that use the RedEye IR protocol.

---

## Overview

The system works as follows:

1. An IR signal is transmitted by the calculator (e.g. HP 28C).
2. A **TSMP58000** IR receiver module captures the modulated IR signal.
3. The RP2040 decodes the HP RedEye protocol in firmware.
4. The decoded bytes are transmitted via **UART0**.

This effectively converts HP RedEye IR communication into a standard UART data stream, making it easy to interface with:

- PCs (via USB-UART bridge)
- Logic analyzers
- Embedded systems
- Data logging setups

---

## Hardware

### Microcontroller
- RP2040 (e.g. Raspberry Pi Pico or compatible board)

### IR Receiver
- TSMP58000 (33 kHz IR Photo detector and preamplifier in one package)

### Connections (Typical)

- TSMP58000 OUT → RP2040 GPIO (configurable)
- TSMP58000 VCC → 3.3V
- TSMP58000 GND → GND
- RP2040 UART0 TX → External UART device RX
- RP2040 GND → Common ground

Pin assignments can be changed in firmware if needed.

---

## Protocol

This project implements the **HP RedEye IR protocol**, which was used in several classic HP calculators for:

- Printer communication
- Data transfer
- Peripheral connectivity

The firmware performs:

- Pulse timing capture
- Frame synchronization
- Bit decoding
- Byte reconstruction
- UART transmission

The UART output contains decoded RedEye payload bytes in real time.

---

## Project Status

This is a **hobbyist project** created for experimentation and personal use.

It was developed to interface with an HP 28C calculator but may require modification for other HP devices or timing variations.

---

## Disclaimer

This project is provided **as-is**, without any warranty.

- It is an amateur implementation of the HP RedEye protocol.
- Timing accuracy may depend on clock configuration and firmware settings.
- There is no guarantee of compatibility with all RedEye devices.
- You are responsible for verifying electrical compatibility before connecting hardware.
- Improper wiring may damage your RP2040 board or IR module.

---

## Motivation

Many classic HP calculators use the RedEye IR protocol, but modern computers do not natively support it.  
This project bridges that gap by converting legacy IR communication into a simple UART stream that can be easily captured and analyzed.
I also implemented graphics mode, so you could "print" the whole LCD and transfer using UART.

---

Contributions, testing reports, and protocol improvements are welcome.
