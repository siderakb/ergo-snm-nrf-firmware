# Mitosis-like

[ErgoSNM Rev 3.x keyboard](https://github.com/siderakb/ergo-snm-keyboard) wireless [mitosis](https://github.com/reversebias/mitosis)-like edition nRF52840 (E73-2G4M08S1C) firmware.

- nRF52 Central: [`central/`](/central/)
- nRF52 Left: [`peripheral/`](/peripheral/) without `PMW3360_ENABLE` and `RIGHT` symbol
- nRF52 Right: [`peripheral/`](/peripheral/) with `PMW3360_ENABLE` and `RIGHT` symbol

## Pin Map

| Name             | Left  | Right | Central |
| ---------------- | ----- | ----- | ------- |
| Key Col-0        | P0.10 | P0.09 |         |
| Key Col-1        | P0.09 | P1.13 |         |
| Key Col-2        | P1.11 | P0.02 |         |
| Key Col-3        | P1.10 | P0.30 |         |
| Key Col-4        | P0.03 | P0.31 |         |
| Key Col-5        | P0.28 | P0.29 |         |
| Key Col-6        | P1.04 | P0.26 |         |
| Key Col-7        | P0.13 | P0.17 |         |
| Key Row-0        | P0.30 | P0.10 |         |
| Key Row-1        | P0.31 | P0.28 |         |
| Key Row-2        | P0.29 | P0.03 |         |
| Key Row-3        | P0.02 | P1.10 |         |
| Key Row-4        | P1.13 | P1.11 |         |
| PMW3360 SPI CS   |       | P1.06 |         |
| PMW3360 SPI SCLK |       | P0.13 |         |
| PMW3360 SPI MOSI |       | P1.09 |         |
| PMW3360 SPI MISO |       | P0.05 |         |
| PMW3360 Motion   |       | P1.04 |         |
| QMK UART Tx      |       |       | P1.02   |
| QMK UART Rx      |       |       | P1.01   |
| Debug UART Tx    | P0.06 | P0.06 | P0.06   |
| Debug UART Rx    | P0.08 | P0.08 | P0.08   |

## Baudrate

- QMK UART: `57600`
- Debug UART: `115200`
