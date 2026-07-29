# Wheel2Controller

Controller for `stm32f411cue6` microcontroller board.

Specs:
1. 128KB sram - Starts at 0x2000 0000
2. 512KB flash - Starts at 0x0800 0000

## Documentations
1. [GNU Linker Script](https://sourceware.org/binutils/docs/ld/Simple-Example.html)
2. [Reference Manual](ReferenceManual.pdf)
3. [Programming Manual](programming%20manual.pdf)
## Upload using DFU


## Pin Layout:
SDA2 SCL2 -> Multiplexer
SDA1 SCL1 -> MPU6050


## Flashing
Using onboard usb:
```
dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D cmake-build-debug/BipedalV1.bin
```

Using st-link:
```
cmake --build cmake-build-debug/ --target BipedalV1 && st-flash write cmake-build-debug/BipedalV1.bin 0x08000000
```
