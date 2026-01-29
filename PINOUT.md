# OtterAmp_DSP Pinout Reference

RP2350A GPIO mapping for the OtterAmp DSP smart speaker platform.

## GPIO Pin Assignments

| GPIO | Pin | Signal       | Direction | Description                              |
|------|-----|--------------|-----------|------------------------------------------|
| 0    | 2   | I2C0.SCL     | I/O       | I2C bus 0 clock (OLED displays)          |
| 1    | 3   | I2C0.SDA     | I/O       | I2C bus 0 data (OLED displays)           |
| 2    | 4   | SPDIF_TX     | Output    | S/PDIF transmit (27Ω series resistor)    |
| 3    | 5   | SPDIF_RX     | Input     | S/PDIF receive                           |
| 4    | 7   | SPDIF_SEL    | Output    | S/PDIF mux select (active-low for RX)    |
| 5    | 8   | EXT_GPIO     | I/O       | External GPIO expansion                  |
| 6    | 9   | UART_TX      | Output    | Debug UART transmit                      |
| 7    | 10  | UART_RX      | Input     | Debug UART receive                       |
| 8    | 12  | ENC.BTN      | Input     | Rotary encoder push button (10k pull-up) |
| 9    | 13  | ENC.A        | Input     | Rotary encoder channel A (10k pull-up)   |
| 10   | 14  | ENC.B        | Input     | Rotary encoder channel B (10k pull-up)   |
| 11   | 15  | LED1         | Output    | Status LED 1 (active-high, 1k resistor)  |
| 12   | 16  | LED0         | Output    | Status LED 0 (active-high, 1k resistor)  |
| 13   | 17  | I2C1.SCL     | I/O       | I2C bus 1 clock (TAS5830 amplifier)      |
| 14   | 18  | I2C1.SDA     | I/O       | I2C bus 1 data (TAS5830 amplifier)       |
| 15   | 19  | I2C1.INT     | Input     | TAS5830 interrupt/fault                  |
| 16   | 27  | AMP0.FLT     | Input     | Amplifier fault indicator                |
| 17   | 28  | AMP0.PDN     | Output    | Amplifier power down (active-low)        |
| 18   | 29  | AMP0.MUTE    | Output    | Amplifier mute control                   |
| 19   | 31  | AMP0.BCLK    | Output    | I2S bit clock to amplifier (27Ω series)  |
| 20   | 32  | AMP0.WCLK    | Output    | I2S word clock to amplifier (27Ω series) |
| 21   | 33  | AMP0.DATA    | Output    | I2S data to amplifier (27Ω series)       |
| 22   | 34  | AMP0.RTN     | Input     | I2S return/feedback from amplifier       |
| 23   | 35  | ADC0.BCLK    | Output    | I2S bit clock to ADC (27Ω series)        |
| 24   | 36  | ADC0.WCLK    | Output    | I2S word clock to ADC (27Ω series)       |
| 25   | 37  | ADC0.DATA    | Input     | I2S data from ADC                        |
| 26   | 40  | ADC0         | Analog    | Analog input channel 0                   |
| 27   | 41  | ADC1         | Analog    | Analog input channel 1                   |
| 28   | 42  | ADC2         | Analog    | Analog input channel 2                   |
| 29   | 43  | ADC3         | Analog    | Analog input channel 3                   |

## USB Interface

| Signal   | Pin | Description                                |
|----------|-----|--------------------------------------------|
| USB_DP   | 51  | USB D+ (27Ω series, USBLC6 ESD protection) |
| USB_DM   | 52  | USB D- (27Ω series, USBLC6 ESD protection) |
| USB_VBUS | -   | VBUS sense via resistor divider (5.1k/10k) |

## Peripheral Buses

### I2C0 (OLED Displays)
- Pull-ups: 5.1kΩ to +3V3 (R62, R63)
- Devices:
  - J8: 1.3" OLED display
  - J2: 0.96" OLED display

### I2C1 (TAS5830 Class-D Amplifier)
- Pull-ups: 5.1kΩ to +3V3 (R43, R47)
- Interrupt line on GPIO15 for fault/status reporting

### I2S Audio (PIO, RP2350 controller)
- **AMP0 (TAS5830)**: I2S output to Class-D amplifier (GPIOs 19-22)
  - Configured via I2C1
  - Supports return channel for diagnostics
- **ADC0 (PCM1822)**: I2S input from audio ADC (GPIOs 23-25)
  - Hardwired configuration: 32-bit, 2-channel, I2S slave
  - No software configuration required
- All clock/data lines have 27Ω series termination

### S/PDIF
- Input: J7A with 75Ω termination, AC-coupled
- Output: J7B with 120Ω termination
- Signal conditioning: dual 74LVC1GU04 inverters
- Mux: 74LVC1G157 for RX/TX loopback selection

## Rust HAL Pin Configuration

```rust
use rp235x_hal::gpio::Pins;

pub struct OtterAmpPins {
    // I2C0 - OLED displays
    pub i2c0_scl: gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionI2C>,
    pub i2c0_sda: gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionI2C>,
    
    // S/PDIF
    pub spdif_tx: gpio::Pin<gpio::bank0::Gpio2, gpio::FunctionPio0>,
    pub spdif_rx: gpio::Pin<gpio::bank0::Gpio3, gpio::FunctionPio0>,
    pub spdif_sel: gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionSioOutput>,
    
    // Expansion
    pub ext_gpio: gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionSioInput>,
    
    // Debug UART
    pub uart_tx: gpio::Pin<gpio::bank0::Gpio6, gpio::FunctionUart>,
    pub uart_rx: gpio::Pin<gpio::bank0::Gpio7, gpio::FunctionUart>,
    
    // Rotary encoder
    pub enc_btn: gpio::Pin<gpio::bank0::Gpio8, gpio::FunctionSioInput>,
    pub enc_a: gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionSioInput>,
    pub enc_b: gpio::Pin<gpio::bank0::Gpio10, gpio::FunctionSioInput>,
    
    // Status LEDs
    pub led1: gpio::Pin<gpio::bank0::Gpio11, gpio::FunctionSioOutput>,
    pub led0: gpio::Pin<gpio::bank0::Gpio12, gpio::FunctionSioOutput>,
    
    // I2C1 - TAS5830 amplifier
    pub i2c1_scl: gpio::Pin<gpio::bank0::Gpio13, gpio::FunctionI2C>,
    pub i2c1_sda: gpio::Pin<gpio::bank0::Gpio14, gpio::FunctionI2C>,
    pub i2c1_int: gpio::Pin<gpio::bank0::Gpio15, gpio::FunctionSioInput>,
    
    // Amplifier control
    pub amp_flt: gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSioInput>,
    pub amp_pdn: gpio::Pin<gpio::bank0::Gpio17, gpio::FunctionSioOutput>,
    pub amp_mute: gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionSioOutput>,
    
    // I2S output to TAS5830 (PIO, controller mode)
    pub amp_bclk: gpio::Pin<gpio::bank0::Gpio19, gpio::FunctionPio1>,
    pub amp_wclk: gpio::Pin<gpio::bank0::Gpio20, gpio::FunctionPio1>,
    pub amp_data: gpio::Pin<gpio::bank0::Gpio21, gpio::FunctionPio1>,
    pub amp_rtn: gpio::Pin<gpio::bank0::Gpio22, gpio::FunctionPio1>,
    
    // I2S input from PCM1822 (PIO, controller mode, 32-bit stereo)
    pub adc_bclk: gpio::Pin<gpio::bank0::Gpio23, gpio::FunctionPio1>,
    pub adc_wclk: gpio::Pin<gpio::bank0::Gpio24, gpio::FunctionPio1>,
    pub adc_data: gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionPio1>,
}
```

## Notes

- RP2350 is I2S controller for both TAS5830 and PCM1822
- PCM1822 ADC is hardwired (no I2C): 32-bit word length, 2-channel, I2S format
- All GPIO active at 3.3V logic levels
- Series resistors (27Ω) on high-speed signals for EMI/ringing suppression
- LEDs active-high with 1kΩ current limiting resistors
- Encoder uses external 10kΩ pull-ups to +3V3
- USB uses USBLC6-2SC6 for ESD protection
- VBUS detection via voltage divider for USB device mode sensing

---

## Hardware Errata & Revisions

### Rev 1.0 Hardware Bodges

The RP2350 has fixed I2C pin function assignments that differ from the original schematic:

| Issue | Schematic | RP2350 Actual | Bodge Fix |
|-------|-----------|---------------|-----------|
| I2C0 SDA/SCL swapped | GPIO0=SCL, GPIO1=SDA | GPIO0=SDA, GPIO1=SCL | Swap GPIO0↔GPIO1 traces |
| I2C1 SCL wrong pin | GPIO13=I2C1.SCL | GPIO13=I2C0.SCL only | Bridge GPIO13→GPIO15 |
| UART TX/RX swapped | GPIO6=TX, GPIO7=RX | GPIO6=RX, GPIO7=TX | Swap GPIO6↔GPIO7 traces |

**Rev 1.0 Bodge Instructions:**
1. **I2C0**: Cut traces to GPIO0/GPIO1 at OLED connector, cross-wire (GPIO0→SDA pad, GPIO1→SCL pad)
2. **I2C1**: Add wire bridge from GPIO13 pad to GPIO15 pad (GPIO15/INT function sacrificed)
3. **UART** (optional): It's fucked.

### Rev 1.1 Planned Fixes

| Signal | Rev 1.0 GPIO | Rev 1.1 GPIO | Notes |
|--------|--------------|--------------|-------|
| I2C0.SDA | 0 (bodged→1) | 0 | Correct per RP2350 |
| I2C0.SCL | 1 (bodged→0) | 1 | Correct per RP2350 |
| I2C1.SDA | 14 | 14 | Already correct |
| I2C1.SCL | 13 (bodged→15) | 15 | Move to GPIO15 |
| I2C1.INT | 15 (sacrificed) | 13 | Swap with SCL |
| UART.TX | 6 (bodged) | 4 | Move to correct pin |
| UART.RX | 7 (bodged) | 5 | Move to correct pin |

**RP2350 I2C Pin Function Reference:**
- I2C0: SDA = even pins (0,4,8,12,16,20,24,28), SCL = odd pins (1,5,9,13,17,21,25,29)
- I2C1: SDA = even pins (2,6,10,14,18,22,26), SCL = odd pins (3,7,11,15,19,23,27)
