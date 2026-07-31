# Intel 4040 ATmega32 Emulator

An accurate hardware emulator for the Intel 4040 4 bit central processing unit running on Atmel AVR ATmega32 microcontrollers.

## Overview

This project implements the complete Intel 4040 architecture using bit banging routines. It drives physical input and output lines to match the original 8 subcycle bus timing protocol.

## Pin Mapping

* PA0 to PA3: Bidirectional multiplexed 4 bit address and data bus
* PA4: SYNC signal output pin
* PA5 and PA6: CM ROM bank selection outputs
* PC0 to PC7: CM RAM bank selection lines
* PD0: Interrupt input pin
* PD2: Phase 1 clock output pin
* PD3: Phase 2 clock output pin
* PD4: TEST input pin

## Features

* Cycle accurate clock phase generation
* Full support for all 46 instructions
* 24 index registers across two banks
* 8 level hardware stack
* Page 0 and current page indirect ROM fetches

## Building

Compile with AVR GCC or Arduino IDE targeting an ATmega32 running at 20 MHz.
