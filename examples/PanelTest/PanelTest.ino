/*
  ESP32-ICND2153-Panel - PanelTest

  Test and tuning screens for an ICND2153 panel, controlled from the Serial Monitor
  (115200 baud, line ending "Newline"). Wiring and power: see the library README.

    m0      demo screen: colour bars, brightness ramp, text, moving square
    m1      dim ramp: the darkest third of the range, in white/red/green/blue
    m2      orange shades at 100% down to 10%
    m3      full white (for comparing peak brightness or measuring current)
    b<n>    brightness, 0-255
    f<n>    minimum level for lit LEDs, 0-4095 (default 0 = off)
    ?       show current settings

  Register experiments (see the README for what's known about each register):
    g<hex>                register 2 high byte for all colours (67 = normal current range)
    r<i>=<hex>            set register i (0-4) for all colours, e.g. r2=67FF (default)
    r<i>=<hR>,<hG>,<hB>   set register i per LED colour
*/
#include <ICND2153PanelDMA.h>

ICND2153PanelDMA panel;

int mode = 0;
uint8_t brightness = 96;

uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) { return panel.color565(r, g, b); }

void drawDemo(uint32_t seconds, int frame) {
  panel.fillScreen(0);

  // Color bars across the top
  const uint16_t bars[8] = {rgb(255, 0, 0),   rgb(0, 255, 0),   rgb(0, 0, 255),   rgb(255, 255, 0),
                            rgb(0, 255, 255), rgb(255, 0, 255), rgb(255, 255, 255), rgb(255, 128, 0)};
  for (int i = 0; i < 8; i++) panel.fillRect(i * 12, 1, 12, 8, bars[i]);

  // Brightness ramp: should fade from black (left) to white (right)
  for (int x = 0; x < 96; x++) {
    uint8_t v = x * 255 / 95;
    panel.drawFastVLine(x, 10, 6, rgb(v, v, v));
  }

  panel.setTextSize(1);
  panel.setTextColor(rgb(255, 255, 255));
  panel.setCursor(2, 19);
  panel.print("ICND2153");

  panel.setTextColor(rgb(0, 200, 255));
  panel.setCursor(2, 29);
  panel.print("count ");
  panel.print(seconds);

  // Sliding square: redrawn 20 times a second, so any blink on updates is obvious
  int x = frame % (96 - 6 - 2) + 1;
  panel.fillRect(x, 39, 6, 6, rgb(255, 160, 0));

  panel.drawRect(0, 0, 96, 48, rgb(255, 0, 0));
}

// Four 12-row bands ramping over input levels 0..95 (the dark third of 0..255)
void drawDimTest() {
  panel.fillScreen(0);
  for (int x = 0; x < 96; x++) {
    uint8_t v = x;  // 0..95
    panel.drawFastVLine(x, 0, 12, rgb(v, v, v));
    panel.drawFastVLine(x, 12, 12, rgb(v, 0, 0));
    panel.drawFastVLine(x, 24, 12, rgb(0, v, 0));
    panel.drawFastVLine(x, 36, 12, rgb(0, 0, v));
  }
}

// Orange FF9F1C at 100% down to 10%: checks that dark shades keep their colour
void drawOrangeTest() {
  panel.fillScreen(0);
  for (int i = 0; i < 10; i++) {
    int pct = 100 - i * 10;
    uint16_t c = rgb(255 * pct / 100, 159 * pct / 100, 28 * pct / 100);
    panel.fillRect(1 + i * 9, 2, 8, 20, c);
  }
  // a solid orange block with a darker 1-pixel bevel
  panel.fillRect(30, 28, 36, 16, rgb(255, 159, 28));
  panel.drawRect(30, 28, 36, 16, rgb(92, 58, 12));
}

void printSettings() {
  Serial.printf("mode=%d  floor=%u  brightness=%u  last upload %lu us\n", mode, panel.getGrayFloor(),
                brightness, (unsigned long)panel.lastUploadUs());
  Serial.printf("  GCLK per row: %d\n", panel.gclkPerRow);
  for (int i = 0; i < 5; i++)
    Serial.printf("  reg%d  R %04X  G %04X  B %04X\n", i, panel.getRegister(i, 0), panel.getRegister(i, 1),
                  panel.getRegister(i, 2));
}

// r<i>=<hex> or r<i>=<hR>,<hG>,<hB>
void parseRegister(const String &line) {
  int eq = line.indexOf('=');
  if (eq < 2) { Serial.println("format: r2=6707  or  r2=7F07,6707,5F07"); return; }
  int idx = line.substring(1, eq).toInt();
  String vals = line.substring(eq + 1);
  uint16_t v[3];
  int c1 = vals.indexOf(','), c2 = c1 >= 0 ? vals.indexOf(',', c1 + 1) : -1;
  if (c1 < 0) {
    v[0] = v[1] = v[2] = strtol(vals.c_str(), nullptr, 16);
  } else if (c2 > c1) {
    v[0] = strtol(vals.substring(0, c1).c_str(), nullptr, 16);
    v[1] = strtol(vals.substring(c1 + 1, c2).c_str(), nullptr, 16);
    v[2] = strtol(vals.substring(c2 + 1).c_str(), nullptr, 16);
  } else { Serial.println("format: r2=7F07,6707,5F07"); return; }
  panel.setRegister(idx, v[0], v[1], v[2]);
}

void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { line += c; continue; }
    line.trim();
    if (line.length()) {
      long v = line.substring(1).toInt();
      switch (line[0]) {
        case 'm': mode = constrain(v, 0, 3); break;
        case 'f': panel.setGrayFloor(constrain(v, 0, 4095)); break;
        case 'b': brightness = constrain(v, 0, 255); panel.setBrightness(brightness); break;
        case '?': break;
        case 'g': {
          uint16_t hi = strtol(line.substring(1).c_str(), nullptr, 16) & 0xFF;
          uint16_t r2 = (hi << 8) | (panel.getRegister(2, 0) & 0xFF);
          panel.setRegister(2, r2, r2, r2);
          break;
        }
        case 'r': parseRegister(line); break;
        default: Serial.println("commands: m0 m1 m2 m3 f<n> b<n> g<hex> r<i>=<hex> ?"); break;
      }
      printSettings();
    }
    line = "";
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);


  if (!panel.begin()) {
    Serial.println("panel.begin() failed");
    while (true) delay(1000);
  }
  panel.setBrightness(brightness);
  Serial.println("Panel running. Commands: m0 m1 m2 m3 f<n> b<n> g<hex> r<i>=<hex> ?");
  printSettings();
}

void loop() {
  static int frame = 0;
  handleSerial();

  if (mode == 0) drawDemo(millis() / 1000, frame++);
  else if (mode == 1) drawDimTest();
  else if (mode == 2) drawOrangeTest();
  else panel.fillScreen(rgb(255, 255, 255));
  panel.show();

  delay(50);
}
