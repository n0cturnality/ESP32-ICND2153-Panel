/*
  ICND2153PanelDMA - ESP32 DMA driver for HUB75-style LED panels built on ChipOne ICND2153
  PWM drivers. Part of the ESP32-ICND2153-Panel library
  (https://github.com/n0cturnality/ESP32-ICND2153-Panel). MIT licensed, see LICENSE.

  Tested panel: NCSD-P2.5-96X48(2121)-24S-V2.1 (96x48, 1/24 scan, ICND2153AP column drivers,
  RUC7258D row drivers, MW245B input buffer) on a classic ESP32 (DevKit V1).
  See README.md for wiring, usage and everything learned about the chip.

  Adafruit_GFX display: draw, then show(). Every pin change comes from the I2S peripheral
  in 16-bit parallel (LCD) mode, streamed by DMA at a fixed clock:

    - Refresh: one full scan frame (row address + 138 GCLK pulses per row) sits in DMA
      memory and loops forever in hardware. No CPU time, perfectly even timing.
    - flipDMABuffer(): snapshots the finished drawing and returns at once; a background
      task uploads it, sleeping (not spinning) while the DMA plays, so it's safe to call
      from timer callbacks. show() does the same but waits until the upload is done.
    - Uploads go in slices of rows (uploadParts). Each slice (image data with
      GCLK woven in, then the rest of that scan frame) is spliced into the loop at a frame
      boundary and played once. One small buffer is reused for every slice, which keeps
      the DMA memory to about 34 KB (about 65 KB in total with the frame buffers).

  The panel clock (CLK) is the I2S WS clock, so it runs continuously. That's harmless:
  with LAT low the chips just shift zeros, and every data word is fully re-sent before
  it's latched.

  Also provides begin(), clearScreen(), flipDMABuffer() and setBrightness8() so sketches
  written for ESP32-HUB75-MatrixPanel-DMA work with few changes.

  Requires the "Adafruit GFX Library". Uses the I2S1 peripheral.
*/
#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <vector>
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc.h"
#include "soc/i2s_struct.h"
#include "soc/gpio_sig_map.h"
#include "driver/gpio.h"
#if __has_include("esp_private/periph_ctrl.h")
#include "esp_private/periph_ctrl.h"
#else
#include "driver/periph_ctrl.h"
#endif
#if __has_include("esp_rom_gpio.h")
#include "esp_rom_gpio.h"
#define ICN_MATRIX_OUT(pin, sig, inv) esp_rom_gpio_connect_out_signal((pin), (sig), (inv), false)
#else
#include "rom/gpio.h"
#define ICN_MATRIX_OUT(pin, sig, inv) gpio_matrix_out((pin), (sig), (inv), false)
#endif

#ifndef ICND2153_PINS_DEFINED
#define ICND2153_PINS_DEFINED
struct ICND2153Pins {
  uint8_t r1, g1, b1, r2, g2, b2;
  uint8_t a, b, c, d, e;
  uint8_t lat, oe, clk;
};
// Same pins as the ESP32-HUB75-MatrixPanel-DMA library defaults
static const ICND2153Pins ICND2153_DEFAULT_PINS = {
  25, 26, 27, 14, 12, 13,   // R1 G1 B1 R2 G2 B2
  23, 19, 5, 17, 33,        // A B C D E
  4, 15, 16                 // LAT OE CLK
};
#endif

class ICND2153PanelDMA : public Adafruit_GFX {
 public:
  static const int PANEL_W = 96;
  static const int PANEL_H = 48;
  static const int SCAN = 24;
  static const int CHIPS = 6;
  static const uint16_t GRAY_MAX = 0x3FFF;
  static const int STREAM_WORDS = SCAN * 16 * CHIPS * 16;  // one word per data clock, whole image

  // ---- Settings: change before begin() ----
  int busMHz = 20;               // I2S word clock: 5, 10 or 20 (MHz). GCLK runs at half this.
  int uploadParts = 4;           // slices per upload: 1, 2, 3, 4, 6, 8 (more = less memory, slower)
  bool clkInvert = false;        // try true if the image is garbled or shifted
  bool swapWords = true;         // classic ESP32 16-bit I2S quirk: outputs each word pair swapped
  int gclkPerRow = 138;          // required by the chip (96 and 80 lose row sync)
  int rowGapWords = 10;          // OE-low words at each row change
  bool doubleUpload = true;      // write both of the chips' buffers (needed for reliable updates)
  int taskCore = 0;              // core for the upload task (see begin())
  int taskPriority = 1;
  bool useDimRange = true;       // use the chips' low-current range at low brightness, with the
                                 // image scaled up to match: removes low-level flicker (slight tint)
  // The low range is used below this brightness (0-255). The LEDs' colour shifts slightly at
  // low current, so it's best kept to the levels where the normal range would flicker
  // (below 28 on the tested panel). Capped at the smallest dimRangeRatio x 255.
  uint8_t dimRangeBelow = 28;
  // Low range output / normal range output for each LED colour {R, G, B}, used to scale the
  // image up so brightness matches across the switch. Measured on the tested panel.
  float dimRangeRatio[3] = {0.248f, 0.235f, 0.227f};
  bool vsyncOnUpdate = false;    // VSYNC in every update's setup (on makes animations tear)
  bool gclkDuringUpload = true;  // keep the panel lit while uploading

  explicit ICND2153PanelDMA(const ICND2153Pins &pins = ICND2153_DEFAULT_PINS)
      : Adafruit_GFX(PANEL_W, PANEL_H), _p(pins) {}

  bool begin() {
    if (_begun) return true;
    if (busMHz != 5 && busMHz != 10 && busMHz != 20) busMHz = 20;
    if (uploadParts < 1 || SCAN % uploadParts != 0) uploadParts = 4;
    _partWords = STREAM_WORDS / uploadParts;
    buildGamma();
    memset(_draw, 0, sizeof(_draw));
    memset(_snapA, 0, sizeof(_snapA));
    memset(_snapB, 0, sizeof(_snapB));

    // ---- buffers ----
    std::vector<uint16_t> v;
    buildRefresh(v);  if (!toSegs(v, _refresh)) return fail("refresh buffer");
    buildInit(v);     if (!toSegs(v, _init))    return fail("init buffer");
    buildVsync(v);    if (!toSegs(v, _vsync))   return fail("vsync buffer");
    if (!allocStream()) return fail("stream buffer");
    if (!makeFinish()) return fail("finish buffer");

    // ---- descriptor chains ----
    if (!makeLoop(_loopA) || !makeLoop(_loopB)) return fail("loop descriptors");
    bool ok;
    if (uploadParts == 1) {
      ok = makeChain(_chFirst, {&_init, &_stream, &_finish, &_vsync});
    } else {
      ok = makeChain(_chFirst, {&_init, &_stream, &_finish}) &&
           makeChain(_chMid, {&_stream, &_finish});
    }
    ok = ok && makeChain(_chLast, {&_stream, &_finish, &_vsync});
    if (!ok) return fail("upload descriptors");

    // ---- hardware: start looping the refresh frame, then upload a black image ----
    setupPins();
    setupI2S();
    linkLoop(_loopA);
    linkLoop(_loopB);
    startDMA(_loopA.first);
    _cur = 0;
    if (!waitInLoop(_loopA, 200)) Serial.println("ICND2153PanelDMA: DMA not running");
    // Upload task on core 0 at priority 1, below the Wi-Fi tasks. (Core 1 at priority 2,
    // above Arduino's loop(), starved a sketch whose setup did its network work there.)
    if (xTaskCreatePinnedToCore(taskEntry, "icnd2153", 4096, this, taskPriority, &_task, taskCore) != pdPASS)
      return fail("upload task");
    _begun = true;
    show();   // black frame
    return true;
  }

  // ---- Adafruit_GFX ----
  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || y < 0 || x >= _width || y >= _height) return;
    int16_t t;
    switch (rotation) {
      case 1: t = x; x = WIDTH - 1 - y; y = t; break;
      case 2: x = WIDTH - 1 - x; y = HEIGHT - 1 - y; break;
      case 3: t = x; x = y; y = HEIGHT - 1 - t; break;
    }
    _draw[y][x] = color;
  }

  void fillScreen(uint16_t color) override {
    for (int y = 0; y < PANEL_H; y++)
      for (int x = 0; x < PANEL_W; x++) _draw[y][x] = color;
  }

  // Hand the finished drawing to the panel and return at once (like the HUB75 library's
  // buffer flip). The background task uploads it; the panel stays lit throughout.
  // If frames come faster than they can be uploaded, the newest one wins.
  // The frame is copied into a pending buffer; the upload task swaps it in whole when it
  // starts an upload, so a frame can never change while it's being sent (no tearing).
  void flipDMABuffer() {
    if (!_begun) return;
    portENTER_CRITICAL(&_mux);
    memcpy((void *)_pendBuf, _draw, sizeof(_draw));
    _newFrame = true;
    portEXIT_CRITICAL(&_mux);
    xTaskNotifyGive(_task);
  }

  // Same as flipDMABuffer(), but waits until the panel shows the new image.
  void show() {
    flipDMABuffer();
    while (_begun && (_newFrame || _resend || _busy)) vTaskDelay(1);
  }

  // 0-255. Scales the whole image evenly, like the HUB75 library. Takes effect on the next
  // show(). With useDimRange, brightness below dimRangeBelow instead runs the chips in their
  // low-current range with each colour scaled up to match (see dimRangeRatio).
  void setBrightness(uint8_t b) {
    _brightness = b;
    const float e = b / 255.0f;
    float lim = dimRangeRatio[0];
    for (int c = 1; c < 3; c++) if (dimRangeRatio[c] < lim) lim = dimRangeRatio[c];
    const bool dim = useDimRange && b > 0 && b < dimRangeBelow && lim > 0.0f && e <= lim;
    for (int c = 0; c < 3; c++) _scale[c] = dim ? e / dimRangeRatio[c] : e;
    if (dim != _dimActive) {
      _dimActive = dim;
      applyRange();
    }
  }
  bool dimRangeActive() const { return _dimActive; }

  // Optional minimum level for lit LEDs (0-4095, 0 = off). Takes effect on the next show().
  void setGrayFloor(uint16_t f) { grayFloor = f > GRAY_MAX / 4 ? GRAY_MAX / 4 : f; buildGamma(); }
  uint16_t getGrayFloor() const { return grayFloor; }

  static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  }

  uint32_t lastUploadUs() const { return _lastUploadUs; }

  // Test switch: include VSYNC in the setup sequence of each update (applied next upload)
  void setVsyncOnUpdate(bool on) {
    vsyncOnUpdate = on;
    _regsDirty = true;   // the init sequence is rebuilt in place (same length either way)
    if (_begun) requestUpload();
  }

  // Config registers, numbered by write order: 0 = 0x0008 (2 latch clocks), 1 = scan lines
  // (4), 2 = 0x67FF (6), 3 = 0x40F7 (8), 4 = 0x0040 (10). Register 2: high byte bits 0-1
  // select the current range (11 normal, 01 low); low byte bits 2-7 trim the LED current
  // (FF measured 0.72 A full white vs 0.53 A at 07 on the tested panel). Each LED colour can
  // get its own value. Applied with the next upload (this call triggers one). For register
  // 2 this sets the normal-range value; the low range (useDimRange) is derived from it.
  void setRegister(int idx, uint16_t red, uint16_t green, uint16_t blue) {
    if (idx < 0 || idx > 4) return;
    if (idx == 2) {
      _reg2Normal[0] = red; _reg2Normal[1] = green; _reg2Normal[2] = blue;
      applyRange();
    } else {
      _regs[idx][0] = red; _regs[idx][1] = green; _regs[idx][2] = blue;
      _regsDirty = true;
    }
    if (_begun) requestUpload();
  }
  uint16_t getRegister(int idx, int colour) const { return (idx >= 0 && idx < 5 && colour >= 0 && colour < 3) ? _regs[idx][colour] : 0; }

  // ---- ESP32-HUB75-MatrixPanel-DMA compatibility ----
  void clearScreen() { fillScreen(0); }
  // Takes effect immediately, like the HUB75 library: re-sends the image on screen.
  void setBrightness8(uint8_t b) {
    setBrightness(b);
    if (_begun) requestUpload();
  }

  // Debug: the DMA's current descriptor address (should keep changing)
  uint32_t dmaPosition() const { return readDscr(); }

 private:
  // DMA descriptor, same layout as the ROM's lldesc_t
  struct Desc {
    volatile uint32_t size : 12;
    volatile uint32_t length : 12;
    volatile uint32_t offset : 5;
    volatile uint32_t sosf : 1;
    volatile uint32_t eof : 1;
    volatile uint32_t owner : 1;
    volatile uint8_t *buf;
    Desc *volatile next;
  };
  struct Seg { uint16_t *p; uint16_t n; };          // n is always even
  struct Loop { Desc *d = nullptr; int n = 0; Desc *first = nullptr; Desc *last = nullptr; };
  struct Chain { Desc *d = nullptr; int n = 0; Desc *first = nullptr; Desc *tail = nullptr; };

  enum : int { CHUNK = 2046 };                       // words per DMA buffer (4092 bytes)
  enum : uint16_t { W_LAT = 1 << 11, W_OE = 1 << 12 };
  static uint16_t rowBits(int r) { return (uint16_t)((r & 31) << 6); }

  ICND2153Pins _p;
  uint16_t _draw[PANEL_H][PANEL_W];             // what the sketch draws into
  // Two frame snapshots: _pendBuf receives flips, _workBuf is what the task is sending.
  // The task swaps the pointers (under _mux) at the start of each upload.
  uint16_t _snapA[PANEL_H][PANEL_W];
  uint16_t _snapB[PANEL_H][PANEL_W];
  uint16_t (*volatile _pendBuf)[PANEL_W] = _snapA;
  uint16_t (*_workBuf)[PANEL_W] = _snapB;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
  uint16_t _gamma[256];
  uint8_t _brightness = 128;
  float _scale[3] = {128 / 255.0f, 128 / 255.0f, 128 / 255.0f};  // per-colour image scale (after the range choice)
  bool _dimActive = false;
  uint16_t _reg2Normal[3] = {0x67FF, 0x67FF, 0x67FF};
  uint16_t grayFloor = 0;     // optional minimum level for lit LEDs (0 = off, like HUB75)

  std::vector<Seg> _refresh, _init, _vsync, _stream, _finish;
  Loop _loopA, _loopB;
  Chain _chFirst, _chMid, _chLast;
  int _partWords = STREAM_WORDS;
  int _cur = 0;
  bool _begun = false;
  TaskHandle_t _task = nullptr;
  // Config registers in write order (2, 4, 6, 8, 10 latch clocks), per LED colour {R, G, B}.
  uint16_t _regs[5][3] = {{0x0008, 0x0008, 0x0008},
                          {(uint16_t)(((SCAN - 1) << 8) | 0x70), (uint16_t)(((SCAN - 1) << 8) | 0x70),
                           (uint16_t)(((SCAN - 1) << 8) | 0x70)},
                          {0x67FF, 0x67FF, 0x67FF},   // low byte = LED current trim (FF = max, tested)
                          {0x40F7, 0x40F7, 0x40F7},
                          {0x0040, 0x0040, 0x0040}};
  volatile bool _regsDirty = false;
  bool _firstInit = true;           // the power-on setup includes VSYNC regardless
  volatile bool _newFrame = false;  // a flipped frame is waiting in _pendBuf
  volatile bool _resend = false;    // re-send the current frame (brightness/register change)
  volatile bool _busy = false;      // the task is uploading
  bool _warned = false;
  volatile uint32_t _lastUploadUs = 0;

  void applyRange() {
    for (int c = 0; c < 3; c++)
      // Low range: change only the range bits (11 -> 01, so 0x67FF -> 0x65FF). Clearing the
      // other bits too (0x41FF) made unlit LEDs glow brown on the tested panel.
      _regs[2][c] = _dimActive ? (uint16_t)((_reg2Normal[c] & ~0x0300) | 0x0100) : _reg2Normal[c];
    _regsDirty = true;
  }

  bool fail(const char *what) {
    Serial.printf("ICND2153PanelDMA: out of DMA memory (%s)\n", what);
    return false;
  }

  // Colour mapping matches ESP32-HUB75-MatrixPanel-DMA at PIXEL_COLOR_DEPTH_BITS=6, so this
  // panel lights the same LEDs at the same relative levels as panels driven by that library: RGB565 is expanded the library's way, each channel goes
  // through the CIE 1931 lightness curve into 64 steps (any channel >= 19/255 lights), and
  // brightness scales everything evenly. _gamma holds step n as a 14-bit level.
  void buildGamma() {
    for (int v = 0; v < 256; v++) {
      const float L = v * 100.0f / 255.0f;
      const float Y = L <= 8.0f ? L / 902.3f : powf((L + 16.0f) / 116.0f, 3.0f);
      const int step = (int)(Y * 63.0f + 0.5f);              // 0..63, same as lumConvTab_6bit
      _gamma[v] = (uint16_t)((step * (uint32_t)GRAY_MAX + 31) / 63);
    }
  }

  // RGB565 -> three 14-bit channel levels. grayFloor (0 = off, the default) optionally
  // raises every lit channel to at least that level.
  void toGray(uint16_t c, uint16_t &r, uint16_t &g, uint16_t &b) const {
    r = level(_gamma[(c >> 8) & 0xF8], _scale[0]);   // same expansion as the HUB75 library's color565to888()
    g = level(_gamma[(c >> 3) & 0xFC], _scale[1]);
    b = level(_gamma[(c << 3) & 0xF8], _scale[2]);
  }

  uint16_t level(uint16_t full, float scale) const {
    if (full == 0 || scale <= 0.0f) return 0;
    uint32_t v = (uint32_t)(full * scale + 0.5f);
    if (v == 0) v = 1;                      // a lit LED stays lit at any brightness
    if (v < grayFloor) v = grayFloor;
    return (uint16_t)(v > GRAY_MAX ? GRAY_MAX : v);
  }

  // ---- word sequences ----
  // Word bits: 0-5 data (R1 G1 B1 R2 G2 B2), 6-10 row address A-E, 11 LAT, 12 OE.

  void latch(std::vector<uint16_t> &v, int k, int pad) {
    for (int i = 0; i < k; i++) v.push_back(W_LAT);
    for (int i = 0; i < pad; i++) v.push_back(0);
  }

  // One register to every chip. Each LED colour's chips get their own value; remember the
  // pins are rotated on this panel: R1/R2 lines feed the blue chips, G the red, B the green.
  void regWrite(std::vector<uint16_t> &v, uint16_t red, uint16_t green, uint16_t blue, int k) {
    const int total = 16 * CHIPS;
    for (int i = 0; i < total; i++) {
      const int bit = 15 - (i % 16);
      uint16_t w = 0;
      if ((blue >> bit) & 1)  w |= (1 << 0) | (1 << 3);   // R1, R2 lines
      if ((red >> bit) & 1)   w |= (1 << 1) | (1 << 4);   // G1, G2 lines
      if ((green >> bit) & 1) w |= (1 << 2) | (1 << 5);   // B1, B2 lines
      if (i >= total - k) w |= W_LAT;
      v.push_back(w);
    }
    v.push_back(0);
    v.push_back(0);
  }

  void buildInit(std::vector<uint16_t> &v) {
    v.clear();
    latch(v, 0, 4);
    latch(v, 14, 2);   // PRE_ACT
    latch(v, 12, 2);   // EN_OP
    if (vsyncOnUpdate || _firstInit) latch(v, 3, 2);  // VSYNC (always on the first upload)
    else latch(v, 0, 5);                // same length, so it can be switched at runtime
    for (int r = 0; r < 5; r++) {
      latch(v, 0, busMHz);             // ~1 us pause
      latch(v, 14, 2);                 // PRE_ACT
      regWrite(v, _regs[r][0], _regs[r][1], _regs[r][2], r * 2 + 2);
    }
  }

  void buildVsync(std::vector<uint16_t> &v) {
    v.clear();
    latch(v, 0, 2);
    latch(v, 3, 3);
  }

  int frameWords() const { return SCAN * (rowGapWords + 2 * gclkPerRow); }

  // One scan frame: per row, a short OE-low gap with the new address, then 138 GCLK pulses
  void buildRefresh(std::vector<uint16_t> &v) {
    v.clear();
    for (int r = 0; r < SCAN; r++) {
      for (int i = 0; i < rowGapWords; i++) v.push_back(rowBits(r));
      for (int i = 0; i < gclkPerRow; i++) {
        v.push_back(rowBits(r) | W_OE);
        v.push_back(rowBits(r));
      }
    }
  }

  // GCLK/row pattern woven into the data stream (same timing as the refresh frame)
  struct Pattern {
    int r = 0, p = 0, gap, len;
    Pattern(int gapWords, int gclk) : gap(gapWords), len(gapWords + 2 * gclk) {}
    uint16_t next() {
      uint16_t w = rowBits(r);
      if (p >= gap && ((p - gap) & 1) == 0) w |= W_OE;
      if (++p >= len) { p = 0; if (++r >= SCAN) r = 0; }
      return w;
    }
  };

  // After a slice, the rest of the scan frame must play so the next thing starts at row 0.
  // Those words are identical to the tail of the refresh frame, so point at that when the
  // offset allows; otherwise build a small separate buffer.
  bool makeFinish() {
    _finish.clear();
    if (!gclkDuringUpload) return true;
    const int fw = frameWords();
    const int off = _partWords % fw;
    if (off == 0) return true;
    if ((off & 1) == 0) {
      int start = 0;
      for (const Seg &s : _refresh) {
        const int end = start + s.n;
        if (end > off) {
          const int skip = off > start ? off - start : 0;
          _finish.push_back({s.p + skip, (uint16_t)(s.n - skip)});
        }
        start = end;
      }
      return true;
    }
    std::vector<uint16_t> v;
    Pattern pat(rowGapWords, gclkPerRow);
    for (int i = 0; i < _partWords; i++) pat.next();
    while (!(pat.r == 0 && pat.p == 0)) v.push_back(pat.next());
    return toSegs(v, _finish);
  }

  // ---- DMA memory ----
  bool toSegs(std::vector<uint16_t> &v, std::vector<Seg> &segs) {
    if (v.size() & 1) v.push_back(v.empty() ? 0 : (v.back() & ~(W_OE | W_LAT)));
    segs.clear();
    size_t i = 0;
    while (i < v.size()) {
      int n = (int)std::min<size_t>(CHUNK, v.size() - i);
      uint16_t *p = (uint16_t *)heap_caps_malloc(n * 2, MALLOC_CAP_DMA);
      if (!p) return false;
      for (int k = 0; k < n; k++) p[swapWords ? (k ^ 1) : k] = v[i + k];
      segs.push_back({p, (uint16_t)n});
      i += n;
    }
    return true;
  }

  bool allocStream() {
    int left = _partWords;
    while (left > 0) {
      int n = std::min<int>(CHUNK, left);
      uint16_t *p = (uint16_t *)heap_caps_calloc(n, 2, MALLOC_CAP_DMA);
      if (!p) return false;
      _stream.push_back({p, (uint16_t)n});
      left -= n;
    }
    return true;
  }

  static void setDesc(Desc &d, const Seg &s, Desc *next) {
    d.size = s.n * 2;
    d.length = s.n * 2;
    d.offset = 0;
    d.sosf = 0;
    d.eof = 0;
    d.owner = 1;
    d.buf = (volatile uint8_t *)s.p;
    d.next = next;
  }

  bool makeLoop(Loop &L) {
    L.n = _refresh.size();
    L.d = (Desc *)heap_caps_calloc(L.n, sizeof(Desc), MALLOC_CAP_DMA);
    if (!L.d) return false;
    for (int i = 0; i < L.n; i++) setDesc(L.d[i], _refresh[i], i + 1 < L.n ? &L.d[i + 1] : &L.d[0]);
    L.first = &L.d[0];
    L.last = &L.d[L.n - 1];
    return true;
  }

  void linkLoop(Loop &L) { L.last->next = L.first; }

  bool makeChain(Chain &c, std::initializer_list<const std::vector<Seg> *> parts) {
    c.n = 0;
    for (auto *pp : parts) c.n += pp->size();
    c.d = (Desc *)heap_caps_calloc(c.n, sizeof(Desc), MALLOC_CAP_DMA);
    if (!c.d) return false;
    int i = 0;
    for (auto *pp : parts)
      for (const Seg &s : *pp) { setDesc(c.d[i], s, i + 1 < c.n ? &c.d[i + 1] : nullptr); i++; }
    c.first = &c.d[0];
    c.tail = &c.d[c.n - 1];
    return true;
  }

  // Image rows for one slice -> stream words, with the GCLK/row pattern woven in
  void buildStream(int part) {
    const int rowsPer = SCAN / uploadParts;
    Pattern pat(rowGapWords, gclkPerRow);
    size_t si = 0, k = 0;
    for (int row = part * rowsPer; row < (part + 1) * rowsPer; row++) {
      const int yt = PANEL_H - 1 - row;            // panel is mounted upside down
      const int yb = PANEL_H - 1 - (row + SCAN);
      for (int ch = 0; ch < 16; ch++) {
        for (int chip = 0; chip < CHIPS; chip++) {
          const int x = (CHIPS - 1 - chip) * 16 + (15 - ch);  // channels reversed in each chip
          uint16_t tr, tg, tb, br, bg, bb;
          toGray(_workBuf[yt][x], tr, tg, tb);
          toGray(_workBuf[yb][x], br, bg, bb);
          // Color pins are rotated on this panel: R pin -> blue, G pin -> red, B pin -> green
          const uint16_t line[6] = {tb, tr, tg, bb, br, bg};
          for (int bit = 15; bit >= 0; bit--) {
            uint16_t w = 0;
            for (int n = 0; n < 6; n++) w |= ((line[n] >> bit) & 1) << n;
            if (chip == CHIPS - 1 && bit == 0) w |= W_LAT;   // DATA_LATCH
            if (gclkDuringUpload) w |= pat.next();
            Seg &s = _stream[si];
            s.p[swapWords ? (k ^ 1) : k] = w;
            if (++k >= s.n) { k = 0; si++; }
          }
        }
      }
    }
  }

  // Re-send the frame that's on screen (after a brightness or register change)
  void requestUpload() {
    _resend = true;
    xTaskNotifyGive(_task);
  }

  static void taskEntry(void *arg) { static_cast<ICND2153PanelDMA *>(arg)->taskLoop(); }

  void taskLoop() {
    for (;;) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      for (;;) {
        portENTER_CRITICAL(&_mux);
        const bool go = _newFrame || _resend;
        if (_newFrame) {                       // take the newest flipped frame, whole
          uint16_t (*t)[PANEL_W] = _workBuf;
          _workBuf = _pendBuf;
          _pendBuf = t;
          _newFrame = false;
        }
        _resend = false;
        _busy = go;          // set inside the lock so show() never sees everything false early
        portEXIT_CRITICAL(&_mux);
        if (!go) break;
        upload();
        if (_firstInit) {      // later updates use the normal setup (no start VSYNC)
          _firstInit = false;
          _regsDirty = true;
        }
        _busy = false;
        // Always sleep one tick between uploads. Uploads poll rather than sleep while the
        // DMA plays, so when frames arrive back to back (splash, animations) this is what
        // lets lower-priority tasks and core 0's idle task run (else: watchdog reset).
        vTaskDelay(1);
      }
    }
  }

  void upload() {
    uint32_t t0 = micros();
    if (_regsDirty) {          // rewrite the init sequence in place (same length, new bits)
      _regsDirty = false;
      std::vector<uint16_t> v;
      buildInit(v);
      if (v.size() & 1) v.push_back(0);
      size_t i = 0;
      for (Seg &sg : _init)
        for (int k = 0; k < sg.n && i < v.size(); k++, i++) sg.p[swapWords ? (k ^ 1) : k] = v[i];
    }
    const int passes = doubleUpload ? 2 : 1;
    for (int pass = 0; pass < passes; pass++) {
      for (int part = 0; part < uploadParts; part++) {
        buildStream(part);
        const bool last = (part == uploadParts - 1);
        Chain &c = last ? _chLast : (pass == 0 && part == 0 ? _chFirst : _chMid);
        // With a single part, the first pass uses the chain that also has the init sequence
        play(uploadParts == 1 && pass == 0 ? _chFirst : c);
      }
    }
    _lastUploadUs = micros() - t0;
  }

  // Splice a chain in after the current refresh frame and wait until it has played
  void play(Chain &c) {
    Loop &cur = _cur ? _loopB : _loopA;
    Loop &nxt = _cur ? _loopA : _loopB;
    linkLoop(nxt);              // the loop we'll land on must loop on itself
    c.tail->next = nxt.first;   // chain ends in that loop
    cur.last->next = c.first;   // splice: after this frame, play the chain
    if (!sleepUntilInLoop(nxt, 200) && !_warned) {
      Serial.println("ICND2153PanelDMA: upload completion not detected");
      _warned = true;
    }
    _cur ^= 1;
  }

  // ---- hardware ----
  void setupPins() {
    const uint8_t pins[13] = {_p.r1, _p.g1, _p.b1, _p.r2, _p.g2, _p.b2,
                              _p.a, _p.b, _p.c, _p.d, _p.e, _p.lat, _p.oe};
    for (int i = 0; i < 13; i++) {
      pinMode(pins[i], OUTPUT);
      ICN_MATRIX_OUT(pins[i], I2S1O_DATA_OUT8_IDX + i, false);
    }
    pinMode(_p.clk, OUTPUT);
    ICN_MATRIX_OUT(_p.clk, I2S1O_WS_OUT_IDX, clkInvert);
  }

  void setupI2S() {
    periph_module_enable(PERIPH_I2S1_MODULE);
    i2s_dev_t &d = I2S1;

    d.conf.rx_fifo_reset = 1; d.conf.rx_fifo_reset = 0;
    d.conf.tx_fifo_reset = 1; d.conf.tx_fifo_reset = 0;
    d.lc_conf.in_rst = 1;     d.lc_conf.in_rst = 0;
    d.lc_conf.out_rst = 1;    d.lc_conf.out_rst = 0;
    d.lc_conf.ahbm_rst = 1;   d.lc_conf.ahbm_rst = 0;
    d.conf.rx_reset = 1;      d.conf.rx_reset = 0;
    d.conf.tx_reset = 1;      d.conf.tx_reset = 0;

    d.conf.val = 0;
    d.conf2.val = 0;
    d.conf2.lcd_en = 1;
    d.conf2.lcd_tx_wrx2_en = 0;
    d.conf2.lcd_tx_sdx2_en = 0;

    d.sample_rate_conf.val = 0;
    d.sample_rate_conf.rx_bits_mod = 16;
    d.sample_rate_conf.tx_bits_mod = 16;
    d.sample_rate_conf.rx_bck_div_num = 2;
    d.sample_rate_conf.tx_bck_div_num = 2;

    // 80 MHz base / clkm_div_num / 2  ->  div 2 = 20 MHz, 4 = 10 MHz, 8 = 5 MHz
    d.clkm_conf.val = 0;
    d.clkm_conf.clka_en = 0;
    d.clkm_conf.clkm_div_a = 1;
    d.clkm_conf.clkm_div_b = 0;
    d.clkm_conf.clkm_div_num = 40 / busMHz;

    d.fifo_conf.val = 0;
    d.fifo_conf.rx_fifo_mod_force_en = 1;
    d.fifo_conf.tx_fifo_mod_force_en = 1;
    d.fifo_conf.rx_fifo_mod = 1;
    d.fifo_conf.tx_fifo_mod = 1;   // 16-bit single channel
    d.fifo_conf.rx_data_num = 32;
    d.fifo_conf.tx_data_num = 32;
    d.fifo_conf.dscr_en = 1;

    d.conf1.val = 0;
    d.conf1.tx_stop_en = 0;
    d.conf1.tx_pcm_bypass = 1;

    d.conf_chan.val = 0;
    d.conf_chan.tx_chan_mod = 1;
    d.conf_chan.rx_chan_mod = 1;

    d.timing.val = 0;
    d.int_ena.val = 0;
    d.int_clr.val = 0xFFFFFFFF;

    d.lc_conf.val = 0;
    d.lc_conf.out_data_burst_en = 1;
    d.lc_conf.outdscr_burst_en = 1;
  }

  void startDMA(Desc *first) {
    i2s_dev_t &d = I2S1;
    d.out_link.addr = ((uint32_t)(uintptr_t)first) & 0xFFFFF;
    d.out_link.stop = 0;
    d.out_link.start = 1;
    d.conf.tx_start = 1;
  }

  static uint32_t readDscr() { return I2S1.out_link_dscr; }

  static bool inRange(uint32_t a, const void *lo, const void *hi) {
    a &= 0xFFFFF;
    uint32_t l = ((uint32_t)(uintptr_t)lo) & 0xFFFFF, h = ((uint32_t)(uintptr_t)hi) & 0xFFFFF;
    return a >= l && a < h;
  }

  // Wait for a spliced chain to finish. A chain plays in well under a millisecond, so poll
  // closely for the first 2 ms (sleeping a whole 1 ms tick each time made every update take
  // 8+ ms). After that, sleep a tick between checks so other tasks and the idle task the
  // watchdog watches keep running. Higher-priority tasks (Wi-Fi) preempt the polling anyway.
  bool sleepUntilInLoop(const Loop &L, uint32_t timeoutMs) {
    const uint32_t t0 = micros();
    while (!inRange(readDscr(), L.d, L.d + L.n)) {
      const uint32_t el = micros() - t0;
      if (el >= timeoutMs * 1000UL) return false;
      if (el < 2000) delayMicroseconds(20);
      else vTaskDelay(1);
    }
    return true;
  }

  // Wait until the DMA is working inside loop L (busy-waits; only used once in begin())
  bool waitInLoop(const Loop &L, uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
      if (inRange(readDscr(), L.d, L.d + L.n)) return true;
      delayMicroseconds(50);
    }
    return false;
  }
};
