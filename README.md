# esphome-components
This repository contains a collection of my custom components
for [ESPHome](https://esphome.io/).

## 1. Usage

Use latest [ESPHome](https://esphome.io/) (at least v1.18.0)
with external components and add this to your `.yaml` definition:

```yaml
external_components:
  - source: github://stanek0j/esphome-components
```
## 2. Components

### 2.1 rubicson

Component that attaches to remote_receiver and decodes Rubicson 433 MHz OOK-PPM thermometers.
Based on [rtl_433](https://github.com/merbanan/rtl_433) Rubicson-Temperature decoder.

#### 2.1.1 Example

```yaml
esp32:
  variant: esp32c3
  flash_size: 4MB
  framework:
    type: esp-idf

spi:
  miso_pin: GPIO7
  mosi_pin: GPIO8
  clk_pin: GPIO9

cc1101:
  cs_pin: GPIO6
  gdo0_pin:
    number: GPIO4
    allow_other_uses: true
  frequency: 433.92MHz
  modulation_type: ASK/OOK
  filter_bandwidth: 203kHz

remote_receiver:
  pin:
    number: GPIO4
    allow_other_uses: true
  filter: 4us
  tolerance: 25%
  idle: 8ms
  buffer_size: 4kb

external_components:
  - source: github://stanek0j/esphome-components
    components: [ rubicson ]
    refresh: always

rubicson:
  temperature:
    name: "Temperature"
  battery_low:
    name: "Battery"
```
