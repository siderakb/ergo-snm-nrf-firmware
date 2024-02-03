# ErgoSNM nRF Firmware

[ErgoSNM Rev 3.x](https://github.com/siderakb/ergo-snm-keyboard)  wireless keyboard nRF firmware, based on nRF52840 SoC and [NCS](https://www.nordicsemi.com/Products/Development-software/nrf-connect-sdk).

- [`mitosis-like/`](/mitosis-like/): Main firmware


```mermaid
flowchart TD
    A[PC]---|USB HID| B["ATmega32U4
                        QMK"]
    B --- |UART| C[nRF52 Central]
    C -.- |Gazell 2.4GHz| D[nRF52 Left]
    C -.- |Gazell 2.4GHz| E[nRF52 Righ]
    E --- |SPI| F[PMW3360]
```

- [`three-mod/`](/three-mod/): Experimental firmware
