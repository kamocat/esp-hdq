# esp-hdq
Battery monitoring using TI's HDQ one-wire protocol

I had previously implemented this using avr asm, but it would freeze up if the battery was disconnected.
Using the uart method (described in the [HDQ overview](https://www.ti.com/lit/an/slua408a/slua408a.pdf)) and an RTOS, there is built-in support for timeouts. Furthermore, esp32 is the platform that the batteries are powering in the first place.

For details on individual commands and data available, see the [bq27541 datasheet](https://www.ti.com/lit/ds/symlink/bq27541-g1.pdf).
