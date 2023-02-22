# HUB75 Led Matrix Panel component for ESPHome

This is custom component that allows the use of common Led Matrix Panels
as a display. Multiple displays can be linked together to create a larger
area.

## Before getting started

This component is essentially a port of the
[PxMatrix](https://github.com/2dom/PxMatrix/) Arduino library, albeit
slightly stripped down.

  * Only BINARY and STRAIGHT mux patterns are supported (No SHIFTREG ones).
  * Only SHIFT driver chips are supported (No FM61 chips).

Before attempting to use this component, first confirm that you can
get your display working natively with PxMatrix.

This works for both ESP8266 and ESP32.

## Hardware wiring

Wire it first following the PxMatrix instructions, then you can use the
same pin patterns here.

## Configuration

```
# Turn on the display after setup.
esphome:
  ...
  on_boot:
    priority: 250
    then:
      - hub75.turn_on:
          id: ledmatrix

external_components:
  - source: github://buxtronix/esphome-components
    components: [ hub75 ]

# Turn off the display when OTA begins (otherwise will crash)
ota:
  on_begin:
    then:
      - hub75.turn_off:
          id: ledmatrix

spi:
  # Display CLK pin.
  clk_pin: 18
  # Display R1/RD1 pin.
  mosi_pin: 23

display:
  - platform: hub75
    id: ledmatrix
    # Width/height of total panel assembly.
    width: 32
    height: 16
    # 4, 8, 16, 32 row scanning layout.
    row_scan: 4
    # Scan pattern. LINE, ZIGZAG,ZZAGG, ZAGGIZ, WZAGZIG, VZAG, ZAGZIG, WZAGZIG2
    scan: ZAGGIZ
    # Colour order. RRGGBB, RRBBGG, GGRRBB, GGBBRR, BBRRGG, BBGGRR
    color_order: RRGGBB
    # Mux Patterns. BINARY, STRAIGHT
    mux_pattern: BINARY
    # Block patterns. ABCD DBCA
    block_pattern: ABCD
    # How many physical panels chained wide.
    panels_wide: 1
    # Brightest, can't (yet) control dimming.
    fastupdate: true
    # Remaining pins on the panel.
    pin_a: 22
    pin_b: 21
    pin_c: 5
    # pin_d: 5
    # pin_e: 5
    stb_pin: 26
    # Brightness 0-100. Less effective if fastupdate is True.
    brightness: 100
    enable_pin: 16
      # Standard display lambda, add colours, fonts, etc.
    lambda: |-
        it.print(0, 0, id(digit_font), "Hello, world!");
```

## Automations

### turn_on

### turn_off

### set_brightness

## Licence/Copyright

For the ESPHome component parts:

Copyright (c) 2023, Ben Buxton

Licenced under the GPL v3.

For the PxMatrix code:

```
Copyright (c) 2018, Dominic Buchstaller
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

