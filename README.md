# meFDCAN

A CAN bus library for the stm32G473 and Arduino boards.

## Description

meFDCAN is a library for the stm32G473 microcontroller, which allows communication with other CAN devices using the FDCAN module. The library provides a simple and intuitive interface for sending and receiving CAN messages, making it easy to integrate CAN communication into your stm32G473-based projects.

The library is designed to work with Arduino boards, so you can easily use it with the Arduino Integrated Development Environment (IDE).

## Features

- Simple and intuitive API for sending and receiving CAN messages.
- Supports 11-bit and 29-bit identifier lengths.
- Supports normal, extended and mixed identifier formats.
- Supports normal and remote transmission frames.
- Supports different message buffer configurations.
- Supports bus error management.
- Supports bit timing calculation.
- Supports message filtering.

## Requirements

- An stm32G473 microcontroller board
- A CAN transceiver, such as the TJA1050 or the SN65HVD230.
- An Arduino board, such as the Arduino Uno or the Arduino Mega.
- The Arduino IDE, version 1.8.x or higher.

## Installation

To install meFDCAN, follow these steps:

1. Download the latest release of meFDCAN from the [releases](https://github.com/[YOUR_GITHUB_USERNAME]/meFDCAN/releases) page.
2. Open the Arduino IDE and go to `Sketch` > `Include Library` > `Add .ZIP Library`.
3. Select the `meFDCAN.zip` file you just downloaded and click the `Open` button.
4. The library should now be installed and you should be able to use it in your sketches.

## Usage

To use meFDCAN in your sketch, include the library header file:

```c++
#include <meFDCAN.h>
