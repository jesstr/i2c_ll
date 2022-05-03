## I2C driver for STM32 L0xx

Based on the STM32 Low-Layer library (LL).

Features:

- Polling and interrupt modes are available;
- Repeated Start condition support.

Driver support polling and interrupt modes, but not in the same time. So you should include in your project either `_poll` or `_it` version of the library, but not both of them.

Interrupt version depends on the OS primitives and so requires FreeRTOS-enabled system.
Polling version is OS-free, but no multithreading is supported out-of-the-box.