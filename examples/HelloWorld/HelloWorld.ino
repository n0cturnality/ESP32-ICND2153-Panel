/*
  ESP32-ICND2153-Panel - Hello World

  Minimal example: a 96x48 ICND2153 panel on a classic ESP32 (DevKit V1), drawn with the
  usual Adafruit GFX calls. Wiring and power: see the library README.

  Draw into the panel like any Adafruit GFX display, then call flipDMABuffer() (returns at
  once, the upload happens in the background) or show() (waits until it's on screen).
*/
#include <ICND2153PanelDMA.h>

ICND2153PanelDMA panel;   // default pins (same as ESP32-HUB75-MatrixPanel-DMA's defaults)

void setup() {
  Serial.begin(115200);
  if (!panel.begin()) {
    Serial.println("panel.begin() failed (out of DMA memory?)");
    while (true) delay(1000);
  }
  panel.setBrightness8(128);   // 0-255
}

void loop() {
  static uint32_t n = 0;
  panel.clearScreen();
  panel.drawRect(0, 0, panel.width(), panel.height(), panel.color565(255, 0, 0));
  panel.setTextColor(panel.color565(255, 255, 255));
  panel.setCursor(4, 8);
  panel.print("Hello,");
  panel.setCursor(4, 20);
  panel.print("ICND2153!");
  panel.setTextColor(panel.color565(0, 200, 255));
  panel.setCursor(4, 34);
  panel.print(n++);
  panel.flipDMABuffer();
  delay(1000);
}
