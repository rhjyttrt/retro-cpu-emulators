# ATmega32 Intel 4004 Hardware Emulator

An embedded, cycle accurate hardware emulator for the historic **Intel 4004 4 bit Microprocessor**, powered by an **AVR ATmega32** running at **20 MHz**. This project maps the bus interface, timing clocks, and control lines of the 4004 directly to physical I/O pins on the ATmega32 DIP 40 package.

---

## Features

* **Core Emulation:** Replicates the Intel 4004 instruction set, internal registers, 12 bit PC/Stack, and 4 bit accumulator logic.
* **Pin Level Interface:** Real time generation of phase clocks ($\Phi_1$, $\Phi_2$), SYNC output, and control lines for interfacing with physical peripheral hardware or RAM/ROM chips.
* **20 MHz System Clock:** Utilizes the maximum specified clock rate of the ATmega32 to ensure adequate instruction execution headroom for emulating the ~740 kHz 4004 clock cycles.

---

## Hardware Pinout & Wiring Guide

Below is the pin mapping between the **Intel 4004 signal definitions** and the **ATmega32**:

| Signal / Register Name | i4004 Function | ATmega32 Pin Code | ATmega32 Physical Pin # |
| :--- | :--- | :--- | :--- |
| **D0 – D3** | 4 Bit Data/Address Bus | `PD0 – PD3` | **14, 15, 16, 17** |
| **$\Phi_1$ Clock** | Phase 1 Clock Output | `PB0` | **18** |
| **$\Phi_2$ Clock** | Phase 2 Clock Output | `PB1` | **19** |
| **SYNC** | Machine Cycle Sync | `PB2` | **20** |
| **RESET** | Master Reset Line | `PB3` | **21** |
| **TEST** | Test Input Line | `PB4` | **22** |
| **CM ROM** | ROM Bank Select Line | `PC0` | **23** |
| **CM RAM0 – CM RAM3** | RAM Bank Select Lines | `PC1 – PC4` | **24, 25, 26, 27** |

> **Note on Pin Mapping:** Pins PC0 through PC4 correspond to physical DIP 40 pins 23 through 27 (PORTC[0:4]). Ensure external pull up resistors or level shifting are implemented if you are interfacing with original 12V/15V PMOS Intel 4000 series logic components.

---

## Getting Started

### 1. Fuses Configuration
Set the AVR high/low fuse bits to enable the external 20 MHz high frequency crystal and disable JTAG (to prevent interference with PORTC lines):

* **Low Fuse:** `0xFF` (External Crystal, Fast Power Up)
* **High Fuse:** `0xD9` (JTAG Disabled, SPI Enabled)

Example using `avrdude`:
```bash
avrdude -c usbasp -p m32 -U lfuse:w:0xff:m -U hfuse:w:0xd9:m
