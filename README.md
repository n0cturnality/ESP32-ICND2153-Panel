# ESP32-ICND2153-Panel

An ESP32 driver for HUB75-style RGB LED panels built on **ChipOne ICND2153** PWM driver chips.

These panels have the familiar HUB75 connector, but they aren't ordinary shift-register panels. The ICND2153 stores the image in its own memory, generates brightness internally, and needs a command protocol plus a continuous clock to show anything. The popular ESP32 HUB75 libraries don't support them. This library does.

- **Hardware refresh.** The whole scan runs from DMA memory through the ESP32's I2S peripheral: no CPU time, perfectly even timing, no flicker.
- **Tear-free updates.** Draw a frame, then flip it. Uploads happen in the background while the panel stays lit.
- **Adafruit GFX drawing.** Text, shapes, bitmaps, fonts and rotation all work as usual.
- **Same colours as ESP32-HUB75-MatrixPanel-DMA.** The colour mapping matches that library's 6-bit CIE mapping exactly, so a project can mix both panel types.
- **Familiar API.** `begin()`, `clearScreen()`, `flipDMABuffer()`, `setBrightness8()` and `color565()` work like they do in ESP32-HUB75-MatrixPanel-DMA, so existing sketches port with few changes.

> **Status: 0.1.0, single panel.** Built and tested on one panel model (below) with a classic ESP32. Panel size, colour order and orientation are currently fixed to that panel; chaining and other layouts are planned. Reports from other ICND2153 panels are very welcome.

## Tested hardware

| | |
|---|---|
| Panel | `NCSD-P2.5-96X48(2121)-24S-V2.1`: 96×48 pixels, P2.5, 1/24 scan |
| Column drivers | ChipOne ICND2153AP (6 per data line) |
| Row drivers | RUC7258D |
| Input buffer | MW245B |
| Controller | ESP32 DevKit V1 (classic ESP32, ESP32-WROOM-32) |

## Installation

1. Install **Adafruit GFX Library** from the Arduino Library Manager.
2. Download this repository as a ZIP, then in the Arduino IDE choose **Sketch → Include Library → Add .ZIP Library…**.
3. Open **File → Examples → ESP32-ICND2153-Panel → HelloWorld**.

## Wiring

The default pins match ESP32-HUB75-MatrixPanel-DMA's defaults, so existing HUB75 adapter wiring works unchanged.

| HUB75 | ESP32 | HUB75 | ESP32 |
|---|---|---|---|
| R1 | 25 | G1 | 26 |
| B1 | 27 | GND | GND |
| R2 | 14 | G2 | 12 |
| B2 | 13 | E | 33 |
| A | 23 | B | 19 |
| C | 5 | D | 17 |
| CLK | 16 | LAT | 4 |
| OE | 15 | GND | GND |

- **Power the panel from its own 5 V supply** and connect its ground to the ESP32's ground. The tested panel draws about 0.72 A at full white, full brightness.
- **GPIO12 is an ESP32 boot strapping pin.** If the board ever fails to boot with the panel connected, move G2 to a free pin such as 21 and pass custom pins (below).
- **Data, LAT, OE and CLK** can be any output-capable GPIO; they're routed through the GPIO matrix.

Custom pins:

```cpp
static const ICND2153Pins pins = {
  25, 26, 27, 14, 21, 13,   // R1 G1 B1 R2 G2 B2
  23, 19, 5, 17, 33,        // A B C D E
  4, 15, 16                 // LAT OE CLK
};
ICND2153PanelDMA panel(pins);
```

## Usage

```cpp
#include <ICND2153PanelDMA.h>

ICND2153PanelDMA panel;

void setup() {
  panel.begin();
  panel.setBrightness8(128);
}

void loop() {
  panel.clearScreen();
  panel.setCursor(4, 8);
  panel.setTextColor(panel.color565(255, 255, 255));
  panel.print("Hello");
  panel.flipDMABuffer();   // returns at once; the upload happens in the background
  delay(100);
}
```

### API

| Call | What it does |
|---|---|
| `begin()` | Allocates buffers, starts the DMA refresh and the upload task, shows a black frame. Returns `false` if out of DMA memory. |
| Adafruit GFX calls | Draw into an off-screen buffer: `drawPixel`, `fillRect`, `print`, `drawRGBBitmap`, `setFont`, `setRotation`, and so on. |
| `flipDMABuffer()` | Hands the finished drawing to the panel and returns immediately. Safe to call from timer callbacks. If frames arrive faster than they can be uploaded, the newest one wins. |
| `show()` | Same as `flipDMABuffer()`, but waits until the new frame is on screen. |
| `clearScreen()` | `fillScreen(0)`. |
| `setBrightness8(0-255)` | Sets brightness and re-sends the frame on screen. |
| `setBrightness(0-255)` | Sets brightness for the next flip. |
| `color565(r, g, b)` | Packs 8-bit RGB into the 16-bit colour Adafruit GFX uses. |
| `lastUploadUs()` | How long the last upload took. |
| `setRegister(i, r, g, b)` / `getRegister(i, c)` | Advanced: chip configuration registers, see below. |
| `setGrayFloor(n)` | Advanced: minimum level for lit LEDs (default 0 = off). |

### Settings (set before `begin()`)

| Setting | Default | Notes |
|---|---|---|
| `busMHz` | 20 | I2S word clock: 5, 10 or 20 MHz. The panel's GCLK runs at half this. |
| `uploadParts` | 4 | The image is uploaded in this many slices through one reused buffer. 1 is fastest but needs about 100 KB of DMA memory. |
| `doubleUpload` | true | Writes both of the chips' image buffers. Needed for reliable updates. |
| `vsyncOnUpdate` | false | Sends VSYNC at the start of each update. On makes animations tear. |
| `clkInvert`, `swapWords` | false, true | Low-level I2S options. Leave as is unless the image is garbled. |
| `taskCore`, `taskPriority` | 0, 1 | Where the background upload task runs. |

## Resource use

- **Peripherals:** I2S1 and its DMA.
- **RAM:** about 65 KB. That's the DMA buffers and descriptors (~34 KB), the drawing buffer and two frame snapshots (~27 KB), and the upload task's stack (4 KB).
- **CPU:** the refresh uses none. Each update builds and uploads the frame in a few milliseconds on core 0, at low priority.

## How it works

The ICND2153 is a "first group" SPWM driver in board707's classification: brightness timing (PWM) and row switching are both driven by pulses on the OE pin, which acts as the chip's grayscale clock (GCLK).

- **Refresh:** one full scan frame lives in DMA memory and loops forever. Per row, it sets the A–E row address, waits a few clocks, then sends 138 GCLK pulses. Every pin change is one 16-bit word streamed by I2S in parallel (LCD) mode, so timing is exact.
- **Updates:** `flipDMABuffer()` copies the drawing into a pending buffer and wakes the upload task. The task swaps that frame in whole, converts it to the chips' 14-bit grayscale words, and builds the upload in slices. Each slice is spliced into the refresh loop at a frame boundary and played once by the DMA, with GCLK pulses woven in so the panel stays lit. The CLK pin is the I2S word clock, so it runs continuously; that's harmless, because nothing is latched without LAT.
- **Colours:** RGB565 is expanded the same way as ESP32-HUB75-MatrixPanel-DMA does it, then each channel goes through the CIE 1931 lightness curve into 64 steps. The table is identical to that library's `lumConvTab_6bit`; any channel value of 19/255 or more lights its LED. Brightness then scales everything evenly.

## What we learned about the ICND2153

The public datasheet leaves most of this out, so here's everything this driver relies on. "Tested" means verified on the panel above.

### Commands

Commands are sent by holding LAT high for a number of CLK rising edges:

| LAT clocks | Command |
|---|---|
| 1 | Data latch (last bit of each 16-bit word) |
| 2, 4, 6, 8, 10 | Write configuration register 1–5 (LAT high for the last N clocks of the 16-bit-per-chip write) |
| 3 | VSYNC |
| 12 | Enable outputs |
| 14 | Pre-activate (sent before each register write) |

The setup sequence is: pre-activate, enable outputs, VSYNC, then each register preceded by pre-activate. Registers are written in the order 2, 4, 6, 8, 10 latch clocks.

### Registers

| Write (LAT clocks) | Value | Notes |
|---|---|---|
| 2 | `0x0008` | From board707's DMD_STM32 |
| 4 | `0x1770` | Bits 8–12 = scan lines − 1 (`0x17` = 24) |
| 6 | `0x67FF` | **Tested.** High byte bits 0–1: `11` normal current range, `01` low range (about 35% as bright), `00` off. The `0x40` bit must stay set (clearing it gives a full-on fault state). Low byte bits 2–7: LED current trim; `FF` gave 0.72 A at full white versus 0.53 A at `07`. High byte bits 2–5 showed no measurable effect. |
| 8 | `0x40F7` | No measurable effect from the variations tried |
| 10 | `0x0040` | No measurable effect from the variations tried |

### Timing and data

- **138 GCLK pulses per row are required.** 96 and 80 lose row sync. That fits 128 brightness clocks plus 10 for row switching.
- **Grayscale is 14-bit.** The top 2 bits of each 16-bit word must be 0.
- **Data is shifted MSB first,** one 16-bit word per output channel. The image is sent row by row; within a row, channel by channel, with one word per chip in the chain for each channel. The last bit of the last chip's word gets the 1-clock data latch.
- **GCLK must not have gaps within a frame.** Jittery software-generated GCLK flickers visibly, which is why this driver uses DMA.
- **Updates:** two upload passes, each ending with VSYNC, with no VSYNC at the start of an update. The very first upload after power-on does include the start VSYNC. Other combinations froze the image, showed frames out of order, or tore during animations.
- **The lowest grayscale levels flicker slightly** on the tested panel. A minimum level for lit LEDs (`setGrayFloor`) hides that, but it also stops the darkest shades from lighting, so it's off by default.

### Tested-panel specifics (currently fixed in code)

- **Colour pins are rotated:** the R pin drives blue LEDs, G drives red, and B drives green.
- **The image is mounted upside down,** and the 16 channels run in reverse within each chip.
- **Rows use plain binary A–E addressing,** with 24 scan lines and two data groups (top and bottom half).
- **Peak brightness is about two-thirds** of a comparable Waveshare HUB75 panel. That's set by the panel's current resistors, not the driver.

## Known limitations

- Single 96×48 panel with the layout above; chaining and other panel sizes are planned.
- Classic ESP32 only (I2S parallel mode). ESP32-S3 support would need the LCD_CAM peripheral.
- A brief splash stutter can appear while Wi-Fi starts up, mostly right after flashing. Wi-Fi's start-up work briefly delays the upload task on core 0.

## Credits

This driver builds on other people's reverse-engineering work:

- **[board707/DMD_STM32](https://github.com/board707/DMD_STM32):** the FM6353/ICN2153 protocol, register values and 138-GCLK timing.
- **[CamelCaseName/HUB75Enano](https://github.com/CamelCaseName/HUB75Enano)** and its [ICND2153 write-up](https://gist.github.com/CamelCaseName/59bd58b002521df8daf4efca69d01e2e): the first public notes on driving these chips.
- **The SPWM panel discussions in [hzeller/rpi-rgb-led-matrix](https://github.com/hzeller/rpi-rgb-led-matrix/issues/1866):** datasheets, register maps and captures for many SPWM drivers.
- **[ESP32-HUB75-MatrixPanel-DMA](https://github.com/mrcodetastic/ESP32-HUB75-MatrixPanel-DMA):** the reference for the ESP32 I2S parallel-mode setup and the CIE colour mapping this library matches.

## License

MIT, see [LICENSE](LICENSE).
