# meFDCAN

A CAN bus Arduino library for the stm32G473 FDCAN.  

## Description

meFDCAN is an Arduino library for the stm32G4 microcontroller family, especially the stm32G473, which allows communication with other CAN devices using the FDCAN module. The library provides a simple and intuitive interface for sending and receiving CAN messages, making it easy to integrate CAN communication into your stm32G473-based projects.

The library is designed to work with the STM32 (official) Arduino Core, so you can easily use it with the Arduino Integrated Development Environment (IDE).

## Features

- Simple and intuitive API for sending and receiving CAN messages.
- Supports Receive Interrupts.
- Supports message filtering.
- Classic CAN only.

## Requirements

- An stm32G473 microcontroller board
- A CAN transceiver, such as the TJA1050 or the SN65HVD230.
- The Arduino IDE, version 1.8.x or higher.
- STM32 Core 2.4.0

## Usage

To use meFDCAN in your sketch, include the library header file:

```c++
#include <meFDCAN.h>```

Two H files are also required in your project folder.  Check out the examples. you can get copies from there.

- build_opt.h
- hal_conf_extra.h
