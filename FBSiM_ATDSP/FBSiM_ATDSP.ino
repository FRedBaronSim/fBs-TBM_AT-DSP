// ====================================================================
// FBSiM AT-DSP v0.5.7 — TIMEOUT underflow race fix
//
// Changes from v0.5.6:
//   - Fixed TIMEOUT detection underflow race. v0.5.6 captured `now` at
//     the top of loop() once, then later compared (now - last_pc_msg_ms)
//     against TIMEOUT_MS. If a valid @ATD: message arrived DURING
//     serial_poll() (inside the same loop iteration), last_pc_msg_ms
//     was set to a fresher millis() value than the captured `now`. The
//     unsigned subtraction then wrapped to ~UINT32_MAX, spuriously
//     declaring TIMEOUT even though data was flowing normally.
//
//   - Fix: use a saturated subtraction that returns 0 when
//     last_pc_msg_ms > now. Zero msg_age is semantically correct
//     (message just arrived = fresh data) and prevents the underflow
//     from triggering false TIMEOUT events.
//
// Root cause discovery: v0.5.6-diag1 instrumentation (counters in
// serial_poll, handle_line, and at TIMEOUT trips) captured the
// signature: last_msg_age_ms = 4294967295 (0xFFFFFFFF) at every
// TIMEOUT event, while bytes/prefix/parsed counters incremented
// steadily through the same window. Data was flowing; the timer
// was lying. Cross-validated with fBsOperator's host-side heartbeat
// success counter (climbed steadily through TIMEOUT events).
//
// Impact on production: TIMEOUT/RECOVER cycling every ~30 seconds in
// v0.5.6 production builds when fBsOperator was actively sending its
// 1 Hz heartbeat. Same-millisecond recovery pattern was the visible
// symptom (board declared TIMEOUT and immediately undeclared it in
// the next loop iteration). Cosmetic effect during normal operation,
// but would have ruined demo video footage (NO DATA dot flashing
// every ~30s) and confused customers in field deployment.
//
// Diagnostic instrumentation from v0.5.6-diag1 removed in this
// release; v0.5.7 is production-clean.
// ====================================================================
// FBSiM AT-DSP v0.5.6 — Display backlight PWM control
//
// Changes from v0.5.5:
//   - Backlight pin (BLK) now driven by PB1 (TIM3_CH4 PWM) instead of
//     hard-tied to 3.3V. Enables software-controllable display brightness.
//   - Default brightness: 200/255 (~78%) — adjustable at compile time
//     via BRIGHTNESS_DEFAULT.
//   - New helper: set_brightness(uint8_t value) — runtime brightness
//     setter, also updates current_brightness global for state sync.
//   - NEW: @ATD:BRT:nnn protocol message — runtime brightness setter
//     (0-255). Responds with @ATD:BRT:ACK:nnn confirming applied value.
//   - NEW: @ATD:LOGO protocol message — display logo and hold mode.
//     Useful for brightness validation and production display testing.
//     Responds with @ATD:LOGO:ACK. Exit via any state-changing command.
//     Both originally planned for v0.6.x but added here since
//     recompile-reflash for brightness validation was impractical.
//
// HARDWARE REQUIREMENT: a wire must be added from Black Pill PB1 to
// display module BLK pin BEFORE this firmware will dim the display.
// Until that wire exists, the firmware drives PB1 at its PWM duty cycle
// but the display remains at full brightness (BLK still tied to VIN).
//
// PIN CHOICE NOTE (PB1 vs PA8 fallback): PB1 = TIM3_CH4. Inner encoder
// already puts TIM3 in encoder mode using CH1+CH2 (PB4/PB5). CH4 is an
// independent capture/compare channel, but STM32 timer encoder mode can
// affect PWM behavior on other channels of the same timer. If post-flash
// bench testing shows the PWM doesn't dim OR the inner encoder breaks,
// fall back to PA8 (TIM1_CH1) in a v0.5.7 hotfix — TIM1 is fully
// independent of TIM3 and definitively supports PWM.
//
// Changes from v0.5.4:
//   - "1" glyph flag_w reduced from w/3 to w/6. Hardware validation
//     showed v0.5.4's flag was too wide — combined with the 8px stroke
//     thickness it visually matched a full SEG_A (top horizontal), so
//     "180" read as "780" and "111" read as "777". Shorter flag now
//     reads as an unambiguous tick. TGT cell flag: 13→6px. PRESEL
//     cell flag: 9→4px.
//
// Changes from v0.5.3:
//   - MODE_SCALE 4→3 (uniform banner size across all states). Eliminates
//     v0.5.3 hardware-observed crowding between banner bottom and active
//     digit top. Removes the v0.5.2 adaptive-scale branch (no longer
//     needed — all banners fit at scale 3). Gains ~7px of breathing
//     room between banner and target digits.
//   - "1" digit glyph gains a top flag in draw_7seg_digit(), making
//     "180" unmistakable from ":80". Fixes v0.5.3 hardware-observed
//     readability issue with thin-vertical "1" reading as colon.
//     Flag geometry: (w/3)-wide stroke + t-wide cap atop SEG_B, forming
//     an L-shape with the vertical. Two-stroke avionics "1" aesthetic.
//
// Changes from v0.5.2:
//   - NO DATA full-screen takeover REMOVED (was too dramatic for brief
//     USB scheduling hiccups). Replaced with small amber flashing dot
//     overlay (10px, bottom-right @ (200,200), 1Hz flash).
//   - SCREEN_NO_DATA mode removed from state machine (3 modes now:
//     BOOT / OFF / MAIN). "Waiting for PC App" case now shows
//     SCREEN_OFF (dark) + dot overlay.
//   - Phase row shows "OFF" in dim gray when phase=OFF (was hidden when
//     state was OFF/ARMED); TOGA/GA now amber; CLIMB/CRUISE/DESCENT/
//     APPROACH white. Active-state gate dropped — phase always renders.
//   - Active digits hidden when mode="-" (TO/GA states, where target-
//     speed tracking isn't the active control mode).
//   - ENG_TEXT_Y 200→206: 6px lower to reduce visual crowding against
//     active digits (v0.5.2 backlog item).
//   - Default wake mode = MAN (PC App Request 2): purely PC-App-side
//     change; ATDSP already handled MAN correctly. Documented for
//     completeness.
//   - New helpers: render_nodata_dot, update_nodata_dot. Dot renders
//     via tft_fill_rect 1x1 fallback (no tft_write_pixel helper exists).
//   - render_no_data_screen() removed.
//
// Guiding principle: dark screen = not operational. Main layout =
// operational. Dot overlay = comms problem. Each visual element has
// one meaning.
//
// Changes from v0.5.1:
//   - MODE_Y 14→18 (banner shifted down, opens horizontal budget
//     against the round display bezel at the chord width).
//   - Adaptive banner scale for 6+ char strings — TO ARM and GA ARM
//     now render at scale 3; shorter banners (SPD, FLC, ARMED, TO, GA)
//     stay at scale 4 (MODE_SCALE).
//
// Changes from v0.5.0:
//   - Complete layout redesign (Proposal D):
//     * Banner scale 3→4 (bigger, more prominent)
//     * Phase scale 1→3 (much more visible; draw_char is integer-scale only)
//     * NEW engagement text region at bottom (ENGAGED / ARMED / OFF)
//   - Screen mode state machine: BOOT / NO_DATA / OFF / MAIN.
//     Transitions full-erase and redraw; per-region dirty-tracking runs
//     only while on SCREEN_MAIN.
//   - NO DATA is now a full-screen takeover (was an overlay in v0.5.0).
//   - Boot splash shows theFBSiM logo (logo_bitmap.h, 180x115 RGB565)
//     centered for the first BOOT_SCREEN_MS (2000ms).
//   - OFF state shows solid black screen (per TBM 940 behavior).
//   - Banner logic now reads state+phase combo:
//       state=ARMED + phase=TOGA → "TO ARM" amber
//       state=ARMED + phase=GA   → "GA ARM" amber
//     (falls through to plain "ARMED" if PC App does not populate phase.)
//   - FLC preselect color: dim magenta → light gray (readability fix).
//   - Demo mode removed entirely — NO DATA screen replaces the "no PC
//     App yet" visual. Board is pure DUMB I/O per spec rule #7.
//   - New helpers: tft_write_pixels_bulk (logo burst), render_logo,
//     render_boot_screen / render_no_data_screen / render_off_screen,
//     derive_banner_text, derive_engagement_text, decide_screen_mode,
//     render_main_screen_full.
//   - Protocol unchanged: still 7-field v0.5.0 format.
//
// Changes from v0.4.4:
//   - Protocol: compound @ATD: state is now 7 fields (added `preselected`
//     between `target` and `ias`). Parser extended with one extra strtok.
//   - DisplayState carries a new `preselected` int alongside
//     the existing `target` (active).
//   - render_preselect() helper — Proposal C stacked-bug layout.
//     Shows dim preselect digits above a gray separator whenever
//     target != preselected; fully hides both when matched.
//   - Subtle flash of preselect digits at 600 ms cadence while differing.
//
// Changes from v0.4.3:
//   - DETENTS_PER_COUNT 4→2: E37 variant in use emits 2 pulses per detent,
//     not 4 (verified on hardware 2026-04-22 — was emitting 1 event per 2 clicks)
//
// Changes from v0.4.2:
//   - render_mode_label / render_target_digits / render_phase / render_ias /
//     render_engagement / render_no_data signatures now use
//     `const struct DisplayState&` (Arduino IDE auto-prototype bug, CLAUDE.md #8)
//
// Changes from v0.4.1:
//   - ENCODER_POLL_MS 10→2 for lower detent latency
//   - poll_encoders() extracted; second call before render() drains detents
//     that arrived during the prior (SPI-blocking) render
// ====================================================================

#include <SPI.h>
#include <HardwareTimer.h>
#include "logo_bitmap.h"

// Declared at file top so Arduino IDE auto-prototypes can resolve the type
enum ScreenMode {
    SCREEN_BOOT      = 0,   // logo-only, first BOOT_SCREEN_MS after power-on
    SCREEN_OFF       = 1,   // PC App never connected, silent, or reported state=OFF
    SCREEN_MAIN      = 2    // normal operation (Proposal D layout)
};

// ---- Version ------------------------------------------------------
static const char* FW_VERSION = "0.5.7";

// ---- Display pins -------------------------------------------------
#define TFT_CS   PA3
#define TFT_DC   PA2
#define TFT_RST  PB0
#define DISPLAY_BLK_PIN     PB1     // TIM3_CH4 PWM, 5V-tolerant, near PB0/RST for clean routing

// ---- E37 encoder pins (hardware quadrature) ----------------------
// TIM2 uses PA0 (CH1) and PA1 (CH2) — outer ring
#define OUTER_A  PA0
#define OUTER_B  PA1
// TIM3 uses PB4 (CH1) and PB5 (CH2) — inner knob
#define INNER_A  PB4
#define INNER_B  PB5
// Push button on inner knob shaft
#define BTN_PIN  PB3

// ---- SPI ---------------------------------------------------------
SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE0);

// ---- Colors (RGB565) ---------------------------------------------
#define COLOR_BLACK    0x0000
#define COLOR_WHITE    0xFFFF
#define COLOR_CYAN     0x07FF
#define COLOR_CYAN_DIM    0x043F   // preselect cyan (MAN preselect)
#define COLOR_MAGENTA  0xF81F
#define COLOR_GREEN    0x07E0
#define COLOR_AMBER    0xFD20   // G3000 caution amber
#define COLOR_RED      0xF800
#define COLOR_GRAY_LT  0xC618
#define COLOR_GRAY_DK  0x4208

// ---- Encoder tuning ----------------------------------------------
// E37 variant in use = 2 pulses per detent (verified on hardware 2026-04-22)
static const int DETENTS_PER_COUNT = 2;

// ---- Timings ------------------------------------------------------
static const uint32_t HEARTBEAT_INTERVAL_MS    = 2000;   // send @ATD:ACK
static const uint32_t TIMEOUT_MS               = 5000;   // PC silent → NO DATA
static const uint32_t ENCODER_POLL_MS          = 2;      // hardware counter poll
static const uint32_t BUTTON_DEBOUNCE_MS       = 20;
static const uint32_t FLASH_INTERVAL_MS        = 600;    // preselect flash cadence
#define BOOT_SCREEN_MS 2000                              // logo-only startup hold
#define BRIGHTNESS_DEFAULT  200     // 0-255, ~78% brightness; comfortable bench/cockpit level

// NO DATA dot overlay (v0.5.3). Position (200,200): distance from display
// center (120,120) is ~113, inside the round viewport radius of 120 with
// 7px margin. Overlaps the ENG_TEXT clear band (y=204..221) by two rows —
// handled by dot re-assertion at end of render().
#define NODATA_DOT_X        200
#define NODATA_DOT_Y        200
#define NODATA_DOT_R          5     // 10px diameter (11x11 bbox incl center)
#define NODATA_FLASH_MS     500     // 500ms on, 500ms off = 1Hz

// ---- Serial buffer -----------------------------------------------
static const int SERIAL_RX_BUF = 128;
char rx_buf[SERIAL_RX_BUF];
int rx_len = 0;

// ====================================================================
// LOW-LEVEL SPI + GC9A01 DRIVER (proven, unchanged from v0.2.0)
// ====================================================================
static inline void tft_cs_low()   { digitalWrite(TFT_CS, LOW);  }
static inline void tft_cs_high()  { digitalWrite(TFT_CS, HIGH); }
static inline void tft_dc_cmd()   { digitalWrite(TFT_DC, LOW);  }
static inline void tft_dc_data()  { digitalWrite(TFT_DC, HIGH); }

void tft_write_cmd(uint8_t c) {
    tft_dc_cmd(); tft_cs_low(); SPI.transfer(c); tft_cs_high();
}
void tft_write_data8(uint8_t d) {
    tft_dc_data(); tft_cs_low(); SPI.transfer(d); tft_cs_high();
}
void tft_write_data16(uint16_t d) {
    tft_dc_data(); tft_cs_low();
    SPI.transfer(d >> 8); SPI.transfer(d & 0xFF);
    tft_cs_high();
}

// Bulk 16-bit-per-pixel burst. Caller must have set the draw window first.
// Single SPI transaction + CS-low for the whole run — much faster than
// per-pixel tft_write_data16 for logo-sized blits.
void tft_write_pixels_bulk(const uint16_t* data, uint32_t count) {
    SPI.beginTransaction(spiSettings);
    tft_dc_data(); tft_cs_low();
    for (uint32_t i = 0; i < count; i++) {
        uint16_t p = data[i];
        SPI.transfer(p >> 8);
        SPI.transfer(p & 0xFF);
    }
    tft_cs_high();
    SPI.endTransaction();
}

void tft_init_hw() {
    pinMode(TFT_CS, OUTPUT);
    pinMode(TFT_DC, OUTPUT);
    pinMode(TFT_RST, OUTPUT);
    tft_cs_high(); tft_dc_data();

    digitalWrite(TFT_RST, HIGH); delay(10);
    digitalWrite(TFT_RST, LOW);  delay(20);
    digitalWrite(TFT_RST, HIGH); delay(150);

    SPI.begin();
    SPI.beginTransaction(spiSettings);

    tft_write_cmd(0xEF);
    tft_write_cmd(0xEB); tft_write_data8(0x14);
    tft_write_cmd(0xFE);
    tft_write_cmd(0xEF);
    tft_write_cmd(0xEB); tft_write_data8(0x14);
    tft_write_cmd(0x84); tft_write_data8(0x40);
    tft_write_cmd(0x85); tft_write_data8(0xFF);
    tft_write_cmd(0x86); tft_write_data8(0xFF);
    tft_write_cmd(0x87); tft_write_data8(0xFF);
    tft_write_cmd(0x88); tft_write_data8(0x0A);
    tft_write_cmd(0x89); tft_write_data8(0x21);
    tft_write_cmd(0x8A); tft_write_data8(0x00);
    tft_write_cmd(0x8B); tft_write_data8(0x80);
    tft_write_cmd(0x8C); tft_write_data8(0x01);
    tft_write_cmd(0x8D); tft_write_data8(0x01);
    tft_write_cmd(0x8E); tft_write_data8(0xFF);
    tft_write_cmd(0x8F); tft_write_data8(0xFF);
    tft_write_cmd(0xB6); tft_write_data8(0x00); tft_write_data8(0x20);
    tft_write_cmd(0x36); tft_write_data8(0x08);
    tft_write_cmd(0x3A); tft_write_data8(0x05);
    tft_write_cmd(0x90); tft_write_data8(0x08); tft_write_data8(0x08);
                        tft_write_data8(0x08); tft_write_data8(0x08);
    tft_write_cmd(0xBD); tft_write_data8(0x06);
    tft_write_cmd(0xBC); tft_write_data8(0x00);
    tft_write_cmd(0xFF); tft_write_data8(0x60); tft_write_data8(0x01);
                        tft_write_data8(0x04);
    tft_write_cmd(0xC3); tft_write_data8(0x13);
    tft_write_cmd(0xC4); tft_write_data8(0x13);
    tft_write_cmd(0xC9); tft_write_data8(0x22);
    tft_write_cmd(0xBE); tft_write_data8(0x11);
    tft_write_cmd(0xE1); tft_write_data8(0x10); tft_write_data8(0x0E);
    tft_write_cmd(0xDF); tft_write_data8(0x21); tft_write_data8(0x0C);
                        tft_write_data8(0x02);
    tft_write_cmd(0xF0);
    tft_write_data8(0x45); tft_write_data8(0x09); tft_write_data8(0x08);
    tft_write_data8(0x08); tft_write_data8(0x26); tft_write_data8(0x2A);
    tft_write_cmd(0xF1);
    tft_write_data8(0x43); tft_write_data8(0x70); tft_write_data8(0x72);
    tft_write_data8(0x36); tft_write_data8(0x37); tft_write_data8(0x6F);
    tft_write_cmd(0xF2);
    tft_write_data8(0x45); tft_write_data8(0x09); tft_write_data8(0x08);
    tft_write_data8(0x08); tft_write_data8(0x26); tft_write_data8(0x2A);
    tft_write_cmd(0xF3);
    tft_write_data8(0x43); tft_write_data8(0x70); tft_write_data8(0x72);
    tft_write_data8(0x36); tft_write_data8(0x37); tft_write_data8(0x6F);
    tft_write_cmd(0xED); tft_write_data8(0x1B); tft_write_data8(0x0B);
    tft_write_cmd(0xAE); tft_write_data8(0x77);
    tft_write_cmd(0xCD); tft_write_data8(0x63);
    tft_write_cmd(0x70);
    tft_write_data8(0x07); tft_write_data8(0x07); tft_write_data8(0x04);
    tft_write_data8(0x0E); tft_write_data8(0x0F); tft_write_data8(0x09);
    tft_write_data8(0x07); tft_write_data8(0x08); tft_write_data8(0x03);
    tft_write_cmd(0xE8); tft_write_data8(0x34);
    tft_write_cmd(0x62);
    tft_write_data8(0x18); tft_write_data8(0x0D); tft_write_data8(0x71);
    tft_write_data8(0xED); tft_write_data8(0x70); tft_write_data8(0x70);
    tft_write_data8(0x18); tft_write_data8(0x0F); tft_write_data8(0x71);
    tft_write_data8(0xEF); tft_write_data8(0x70); tft_write_data8(0x70);
    tft_write_cmd(0x63);
    tft_write_data8(0x18); tft_write_data8(0x11); tft_write_data8(0x71);
    tft_write_data8(0xF1); tft_write_data8(0x70); tft_write_data8(0x70);
    tft_write_data8(0x18); tft_write_data8(0x13); tft_write_data8(0x71);
    tft_write_data8(0xF3); tft_write_data8(0x70); tft_write_data8(0x70);
    tft_write_cmd(0x64);
    tft_write_data8(0x28); tft_write_data8(0x29); tft_write_data8(0xF1);
    tft_write_data8(0x01); tft_write_data8(0xF1); tft_write_data8(0x00);
    tft_write_data8(0x07);
    tft_write_cmd(0x66);
    tft_write_data8(0x3C); tft_write_data8(0x00); tft_write_data8(0xCD);
    tft_write_data8(0x67); tft_write_data8(0x45); tft_write_data8(0x45);
    tft_write_data8(0x10); tft_write_data8(0x00); tft_write_data8(0x00);
    tft_write_data8(0x00);
    tft_write_cmd(0x67);
    tft_write_data8(0x00); tft_write_data8(0x3C); tft_write_data8(0x00);
    tft_write_data8(0x00); tft_write_data8(0x00); tft_write_data8(0x01);
    tft_write_data8(0x54); tft_write_data8(0x10); tft_write_data8(0x32);
    tft_write_data8(0x98);
    tft_write_cmd(0x74);
    tft_write_data8(0x10); tft_write_data8(0x85); tft_write_data8(0x80);
    tft_write_data8(0x00); tft_write_data8(0x00); tft_write_data8(0x4E);
    tft_write_data8(0x00);
    tft_write_cmd(0x98); tft_write_data8(0x3E); tft_write_data8(0x07);
    tft_write_cmd(0x35);
    tft_write_cmd(0x21);
    tft_write_cmd(0x11); delay(120);
    tft_write_cmd(0x29); delay(20);

    SPI.endTransaction();
}

void tft_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    tft_write_cmd(0x2A);
    tft_write_data16(x); tft_write_data16(x + w - 1);
    tft_write_cmd(0x2B);
    tft_write_data16(y); tft_write_data16(y + h - 1);
    tft_write_cmd(0x2C);
}

void tft_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > 240) w = 240 - x;
    if (y + h > 240) h = 240 - y;
    if (w <= 0 || h <= 0) return;

    SPI.beginTransaction(spiSettings);
    tft_set_window(x, y, w, h);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    tft_dc_data(); tft_cs_low();
    uint32_t pixels = (uint32_t)w * h;
    for (uint32_t i = 0; i < pixels; i++) {
        SPI.transfer(hi); SPI.transfer(lo);
    }
    tft_cs_high();
    SPI.endTransaction();
}

void tft_fill_screen(uint16_t color) {
    tft_fill_rect(0, 0, 240, 240, color);
}

// Display backlight brightness state. Grouped with set_brightness() here
// (rather than with the render-mode globals further down) so the global
// is defined before its only mutator — required because set_brightness()
// lives in this hardware-utilities block.
static uint8_t current_brightness = BRIGHTNESS_DEFAULT;

// Set display backlight brightness (0=off, 255=full).
// Stored in current_brightness so any later state changes can re-apply.
void set_brightness(uint8_t value) {
    current_brightness = value;
    analogWrite(DISPLAY_BLK_PIN, value);
}

// ====================================================================
// SEVEN-SEGMENT DIGITS
// ====================================================================
static const uint8_t seg_patterns[11] = {
    0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110,
    0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111,
    0b00000000   // index 10 = blank
};

void draw_7seg_digit(int16_t cx, int16_t cy, int16_t w, int16_t h, int16_t t,
                     uint8_t digit, uint16_t fg, uint16_t bg) {
    tft_fill_rect(cx, cy, w, h, bg);
    if (digit > 10) return;
    uint8_t pat = seg_patterns[digit];
    int16_t hh = (h - t) / 2;
    if (pat & 0x01) tft_fill_rect(cx + t,       cy,           w - 2*t, t, fg);
    if (pat & 0x08) tft_fill_rect(cx + t,       cy + h - t,   w - 2*t, t, fg);
    if (pat & 0x40) tft_fill_rect(cx + t,       cy + hh,      w - 2*t, t, fg);
    if (pat & 0x20) tft_fill_rect(cx,           cy + t,       t, hh - t, fg);
    if (pat & 0x02) tft_fill_rect(cx + w - t,   cy + t,       t, hh - t, fg);
    if (pat & 0x10) tft_fill_rect(cx,           cy + hh + t,  t, hh - t, fg);
    if (pat & 0x04) tft_fill_rect(cx + w - t,   cy + hh + t,  t, hh - t, fg);

    // v0.5.4: top flag on "1" for readability. Hardware validation of
    // v0.5.3 showed thin-vertical "1" reading as ":" — e.g. "180" as
    // ":80". Flag spans (w/3)-wide from just-left-of-SEG_B to the right
    // edge, thickness t, seated at top-of-cell. Adjacent to SEG_B's top
    // with no gap — forms an L-shape. Proportional to both TGT (w=40)
    // and PRESEL (w=28) cell sizes.
    if (digit == 1) {
        int16_t flag_w = w / 6;   // v0.5.5: was w/3 — shorter tick, not a top-bar
        tft_fill_rect(cx + w - t - flag_w, cy, flag_w + t, t, fg);
    }
}

// Right-aligned integer with leading blanks
void draw_7seg_number(int16_t right_x, int16_t y,
                      int16_t cw, int16_t ch, int16_t ct, int16_t spacing,
                      int value, uint8_t max_digits,
                      uint16_t fg, uint16_t bg) {
    if (value < 0) value = 0;
    if (value > 999) value = 999;

    uint8_t digits[8]; uint8_t count = 0;
    if (value == 0) { digits[count++] = 0; }
    else { int v = value; while (v > 0 && count < max_digits) { digits[count++] = v % 10; v /= 10; } }

    int16_t x = right_x - cw;
    for (uint8_t i = 0; i < max_digits; i++) {
        uint8_t d = (i < count) ? digits[i] : 10;
        draw_7seg_digit(x, y, cw, ch, ct, d, fg, bg);
        x -= (cw + spacing);
    }
}

// Erase the whole 3-digit area (used when blanking)
void blank_7seg_area(int16_t right_x, int16_t y, int16_t cw, int16_t ch,
                     int16_t spacing, uint8_t num_digits, uint16_t bg) {
    int16_t total = num_digits * cw + (num_digits - 1) * spacing;
    tft_fill_rect(right_x - total, y, total, ch, bg);
}

// ====================================================================
// 5x7 BITMAP FONT — extended with all letters we need
// ====================================================================
struct Glyph { char c; uint8_t cols[5]; };

// Forward declaration for Arduino auto-prototype ordering
const struct Glyph* find_glyph(char c);

static const Glyph font5x7[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'-', {0x08,0x08,0x08,0x08,0x08}},
    {'.', {0x00,0x60,0x60,0x00,0x00}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x42,0x61,0x51,0x49,0x46}},
    {'3', {0x21,0x41,0x45,0x4B,0x31}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'J', {0x20,0x40,0x41,0x3F,0x01}},
    {'K', {0x7F,0x08,0x14,0x22,0x41}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'Q', {0x3E,0x41,0x51,0x21,0x5E}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}},
    {'V', {0x1F,0x20,0x40,0x20,0x1F}},
    {'W', {0x7F,0x20,0x18,0x20,0x7F}},
    {'X', {0x63,0x14,0x08,0x14,0x63}},
    {'Y', {0x03,0x04,0x78,0x04,0x03}},
    {'Z', {0x61,0x51,0x49,0x45,0x43}},
};
static const int font5x7_count = sizeof(font5x7) / sizeof(Glyph);

const struct Glyph* find_glyph(char c) {
    if (c >= 'a' && c <= 'z') c -= 32;  // uppercase
    for (int i = 0; i < font5x7_count; i++) if (font5x7[i].c == c) return &font5x7[i];
    return &font5x7[0];
}

void draw_char(int16_t x, int16_t y, char c, uint8_t scale,
               uint16_t fg, uint16_t bg) {
    const Glyph* g = find_glyph(c);
    for (uint8_t col = 0; col < 5; col++) {
        uint8_t bits = g->cols[col];
        for (uint8_t row = 0; row < 7; row++) {
            uint16_t color = (bits & (1 << row)) ? fg : bg;
            tft_fill_rect(x + col * scale, y + row * scale, scale, scale, color);
        }
    }
}
void draw_string(int16_t x, int16_t y, const char* s, uint8_t scale,
                 uint16_t fg, uint16_t bg) {
    while (*s) { draw_char(x, y, *s, scale, fg, bg); x += 6 * scale; s++; }
}
void draw_string_centered(int16_t cx, int16_t y, const char* s, uint8_t scale,
                          uint16_t fg, uint16_t bg) {
    int len = 0; for (const char* p = s; *p; p++) len++;
    int w = len * 6 * scale - scale;
    draw_string(cx - w / 2, y, s, scale, fg, bg);
}

// ====================================================================
// DISPLAY STATE (what we're currently showing)
// ====================================================================
struct DisplayState {
    char state[8];   // OFF, ARMED, MAN, FLC, TO, GA
    char mode[8];    // MAN, FLC, -
    int  target;       // active (committed) value
    int  preselected;  // pilot-dialed, uncommitted value (v0.5.0)
    int  ias;
    char phase[12];
    char envelope[8];
    bool no_data;    // overlay NO DATA on top of last state
};

DisplayState cur_state  = {"OFF",  "-",  0,  0,  0,  "OFF",  "OK",   false};
DisplayState prev_state = {"----", "-", -1, -1, -1, "----", "----", true};  // force initial redraw

// Preselect flash state (v0.5.0) — placed here so render_preselect() sees them
uint32_t preselect_flash_toggle_ms = 0;
bool     preselect_flash_visible   = true;
bool     rendered_flash_state      = true;   // what render_preselect last drew

// Screen-mode state machine (v0.5.1) — enum declared at file top
static ScreenMode current_screen      = SCREEN_BOOT;
static ScreenMode prev_screen         = SCREEN_BOOT;
static uint32_t   boot_screen_start_ms = 0;     // set in setup()
// v0.5.6: @ATD:LOGO hold mode. While true, decide_screen_mode() pins
// the display to SCREEN_BOOT regardless of the boot-timer / cur_state
// inputs. Cleared the moment any compound @ATD: state message arrives,
// so @ATD:OFF / @ATD:MAN / @ATD:FLC etc. exit the hold normally.
// (Can't pin via boot_screen_start_ms — the now-start<dur comparison
// underflows when start is in the future, which would exit boot mode.)
static bool       logo_hold_active     = false;

// NO DATA dot overlay state (v0.5.3). Runs independently of render() —
// the dot has its own 1Hz flash cadence and re-asserts itself on top of
// whatever screen mode is active.
static bool     nodata_dot_visible    = false;  // currently drawn?
static bool     nodata_dot_rendered   = false;  // last rendered state
static uint32_t nodata_dot_toggle_ms  = 0;      // when we last flipped

// ====================================================================
// LAYOUT CONSTANTS (v0.5.1 Proposal D)
// ====================================================================
#define MODE_Y            18
#define MODE_SCALE        3                   // v0.5.4: 4→3 uniform, gains ~7px breathing room
#define MODE_H_PX         (MODE_SCALE * 7)    // 21

#define TGT_CELL_W        40
#define TGT_CELL_H        50                  // was 55
#define TGT_CELL_T        8
#define TGT_SPACING       10
#define TGT_Y             46                  // was 40 — nudged for banner growth
#define TGT_RIGHT_X       190

// Preselect row (v0.5.0 Proposal C stacked-bug, slightly retuned in v0.5.1)
#define SEPARATOR_Y       101                 // was 100
#define SEPARATOR_W       120
#define SEPARATOR_H       3
#define PRESEL_CELL_W     28
#define PRESEL_CELL_H     34                  // was 38
#define PRESEL_CELL_T     5
#define PRESEL_SPACING    7
#define PRESEL_Y          110
#define PRESEL_RIGHT_X    167

#define PHASE_Y           150                 // was 152
#define PHASE_SCALE       3                   // was 1 — much more visible
#define PHASE_H_PX        (PHASE_SCALE * 7)

#define CUR_Y             176                 // was 168
#define CUR_SCALE         2
#define CUR_H_PX          (CUR_SCALE * 7)

// Engagement text (v0.5.1 — replaces the dot + label from v0.5.0)
#define ENG_TEXT_Y        206                 // was 200 — +6 reduces v0.5.2 crowding
#define ENG_TEXT_SCALE    2

// ====================================================================
// RENDERING (v0.4.1 — per-region dirty tracking, no full wipes)
// ====================================================================
uint16_t color_for_state(const char* s) {
    if (!strcmp(s, "MAN"))   return COLOR_CYAN;
    if (!strcmp(s, "FLC"))   return COLOR_MAGENTA;
    if (!strcmp(s, "TO"))    return COLOR_GREEN;
    if (!strcmp(s, "GA"))    return COLOR_GREEN;
    if (!strcmp(s, "ARMED")) return COLOR_WHITE;
    return COLOR_GRAY_LT;   // OFF and default
}

bool state_is_active(const char* s) {
    return !strcmp(s, "MAN") || !strcmp(s, "FLC") ||
           !strcmp(s, "TO")  || !strcmp(s, "GA");
}

bool state_is_armed(const char* s) {
    return !strcmp(s, "ARMED");
}

// --- Banner / engagement text derivation (v0.5.1) -----------------

// Derives the top-banner text + color from the state+phase combination.
// ARMED+TOGA and ARMED+GA get a distinct amber "TO ARM" / "GA ARM" banner
// so the pilot can see the pre-engaged takeoff/go-around arming state
// without having to cross-reference the phase line.
void derive_banner_text(const struct DisplayState& s,
                        char* out_text, uint16_t* out_color) {
    if (!strcmp(s.state, "MAN")) {
        strcpy(out_text, "SPD");
        *out_color = COLOR_CYAN;
    } else if (!strcmp(s.state, "FLC")) {
        strcpy(out_text, "FLC");
        *out_color = COLOR_MAGENTA;
    } else if (!strcmp(s.state, "TO")) {
        strcpy(out_text, "TO");
        *out_color = COLOR_GREEN;
    } else if (!strcmp(s.state, "GA")) {
        strcpy(out_text, "GA");
        *out_color = COLOR_GREEN;
    } else if (!strcmp(s.state, "ARMED")) {
        if (!strcmp(s.phase, "TOGA")) {
            strcpy(out_text, "TO ARM");
            *out_color = COLOR_AMBER;
        } else if (!strcmp(s.phase, "GA")) {
            strcpy(out_text, "GA ARM");
            *out_color = COLOR_AMBER;
        } else {
            strcpy(out_text, "ARMED");
            *out_color = COLOR_GRAY_LT;
        }
    } else {
        // OFF or unknown — banner not shown in SCREEN_OFF anyway
        out_text[0] = 0;
        *out_color = COLOR_BLACK;
    }
}

// Derives the bottom engagement-row text + color from state alone.
void derive_engagement_text(const struct DisplayState& s,
                            char* out_text, uint16_t* out_color) {
    if (!strcmp(s.state, "OFF")) {
        strcpy(out_text, "OFF");
        *out_color = COLOR_GRAY_DK;
    } else if (!strcmp(s.state, "ARMED")) {
        strcpy(out_text, "ARMED");
        *out_color = COLOR_AMBER;
    } else {
        // MAN / FLC / TO / GA
        strcpy(out_text, "ENGAGED");
        *out_color = COLOR_GREEN;
    }
}

// --- Full-screen modes (v0.5.1) ----------------------------------

// Blit logo_bitmap centered at (center_x, center_y). Only called on
// screen-mode transitions, so blocking SPI burst is acceptable.
void render_logo(int16_t center_x, int16_t center_y) {
    int16_t x0 = center_x - LOGO_W / 2;
    int16_t y0 = center_y - LOGO_H / 2;
    SPI.beginTransaction(spiSettings);
    tft_set_window((uint16_t)x0, (uint16_t)y0, LOGO_W, LOGO_H);
    SPI.endTransaction();
    tft_write_pixels_bulk(logo_bitmap, LOGO_SIZE);
}

void render_boot_screen() {
    tft_fill_rect(0, 0, 240, 240, COLOR_BLACK);
    render_logo(120, 120);
}

void render_off_screen() {
    tft_fill_rect(0, 0, 240, 240, COLOR_BLACK);
    // Intentionally blank per TBM 940 behavior
}

// --- NO DATA dot overlay (v0.5.3) --------------------------------
//
// Draws or erases the amber dot at (NODATA_DOT_X, NODATA_DOT_Y).
// Called only when visibility state changes. Does not touch the rest
// of the display — overlay behavior only. The driver lacks a
// single-pixel-write helper, so we use tft_fill_rect with 1x1 rects
// inside the circle test. 10px diameter = ~81 rects per toggle at
// 1Hz; the cost is negligible.
void render_nodata_dot(bool visible) {
    // Erase the dot's bounding box first (1px margin each side)
    int16_t x0 = NODATA_DOT_X - NODATA_DOT_R - 1;
    int16_t y0 = NODATA_DOT_Y - NODATA_DOT_R - 1;
    int16_t wh = 2 * (NODATA_DOT_R + 1);
    tft_fill_rect(x0, y0, wh, wh, COLOR_BLACK);

    if (!visible) return;

    int16_t r2 = NODATA_DOT_R * NODATA_DOT_R;
    for (int16_t dy = -NODATA_DOT_R; dy <= NODATA_DOT_R; dy++) {
        for (int16_t dx = -NODATA_DOT_R; dx <= NODATA_DOT_R; dx++) {
            if (dx * dx + dy * dy <= r2) {
                tft_fill_rect(NODATA_DOT_X + dx, NODATA_DOT_Y + dy,
                              1, 1, COLOR_AMBER);
            }
        }
    }
}

// --- Per-region renderers -----------------------------------------

void render_mode_label(const struct DisplayState& s) {
    char text[16];
    uint16_t color;
    derive_banner_text(s, text, &color);
    tft_fill_rect(0, MODE_Y - 2, 240, MODE_H_PX + 4, COLOR_BLACK);
    if (text[0]) {
        draw_string_centered(120, MODE_Y, text, MODE_SCALE, color, COLOR_BLACK);
    }
}

void render_target_digits(const struct DisplayState& s) {
    // TO/GA states use mode="-" because target-speed tracking isn't the
    // active control mode. Blank the digit region and skip rendering.
    if (!strcmp(s.mode, "-")) {
        blank_7seg_area(TGT_RIGHT_X, TGT_Y,
                        TGT_CELL_W, TGT_CELL_H, TGT_SPACING, 3, COLOR_BLACK);
        return;
    }

    bool active = state_is_active(s.state);
    bool armed  = state_is_armed(s.state);
    bool envelope_alert = strcmp(s.envelope, "OK") != 0;

    if (active) {
        // Live modes: primary color, amber if envelope alert
        uint16_t digit_color = envelope_alert ? COLOR_AMBER : color_for_state(s.state);
        draw_7seg_number(TGT_RIGHT_X, TGT_Y,
                         TGT_CELL_W, TGT_CELL_H, TGT_CELL_T, TGT_SPACING,
                         s.target, 3, digit_color, COLOR_BLACK);
    } else if (armed) {
        // ARMED: show target in dim white (the bugged value)
        draw_7seg_number(TGT_RIGHT_X, TGT_Y,
                         TGT_CELL_W, TGT_CELL_H, TGT_CELL_T, TGT_SPACING,
                         s.target, 3, COLOR_GRAY_LT, COLOR_BLACK);
    } else {
        // OFF: blank area entirely
        blank_7seg_area(TGT_RIGHT_X, TGT_Y,
                        TGT_CELL_W, TGT_CELL_H, TGT_SPACING, 3, COLOR_BLACK);
    }
}

void render_phase(const struct DisplayState& s) {
    tft_fill_rect(0, PHASE_Y - 2, 240, PHASE_H_PX + 4, COLOR_BLACK);
    if (strlen(s.phase) == 0) return;

    uint16_t color;
    if (!strcmp(s.phase, "OFF")) {
        color = COLOR_GRAY_DK;                          // dim gray — not operational
    } else if (!strcmp(s.phase, "TOGA") || !strcmp(s.phase, "GA")) {
        color = COLOR_AMBER;                            // takeoff / go-around
    } else {
        color = COLOR_WHITE;                            // CLIMB/CRUISE/DESCENT/APPROACH
    }
    draw_string_centered(120, PHASE_Y, s.phase, PHASE_SCALE, color, COLOR_BLACK);
}

void render_ias(const struct DisplayState& s) {
    tft_fill_rect(0, CUR_Y - 2, 240, CUR_H_PX + 4, COLOR_BLACK);
    char buf[16];
    snprintf(buf, sizeof(buf), "IAS %d", s.ias);
    draw_string_centered(120, CUR_Y, buf, CUR_SCALE,
                         COLOR_WHITE, COLOR_BLACK);
}

void render_engagement(const struct DisplayState& s) {
    char text[16];
    uint16_t color;
    bool envelope_alert = strcmp(s.envelope, "OK") != 0;
    bool active = state_is_active(s.state);

    derive_engagement_text(s, text, &color);

    // Envelope alert overrides the ENGAGED label with the alert code in amber
    if (envelope_alert && active) {
        strncpy(text, s.envelope, sizeof(text) - 1);
        text[sizeof(text) - 1] = 0;
        color = COLOR_AMBER;
    }

    int16_t band_h = ENG_TEXT_SCALE * 7 + 4;
    tft_fill_rect(0, ENG_TEXT_Y - 2, 240, band_h, COLOR_BLACK);
    draw_string_centered(120, ENG_TEXT_Y, text, ENG_TEXT_SCALE, color, COLOR_BLACK);
}

void render_preselect(const struct DisplayState& s) {
    int16_t total_w = 3 * PRESEL_CELL_W + 2 * PRESEL_SPACING;
    int16_t left_x  = PRESEL_RIGHT_X - total_w;

    // Matched case: hide preselect digits AND separator entirely
    if (s.target == s.preselected) {
        tft_fill_rect(left_x, PRESEL_Y, total_w, PRESEL_CELL_H, COLOR_BLACK);
        tft_fill_rect(60, SEPARATOR_Y, SEPARATOR_W, SEPARATOR_H, COLOR_BLACK);
        return;
    }

    // Values differ: separator always visible, digits obey flash visibility
    tft_fill_rect(60, SEPARATOR_Y, SEPARATOR_W, SEPARATOR_H, COLOR_GRAY_DK);

    // Preselect color: dim cyan for MAN, light gray for FLC/ARMED/fallback
    // (FLC switched from dim magenta to gray in v0.5.1 — readability fix)
    uint16_t color;
    if      (!strcmp(s.state, "MAN"))   color = COLOR_CYAN_DIM;
    else if (!strcmp(s.state, "FLC"))   color = COLOR_GRAY_LT;
    else if (!strcmp(s.state, "ARMED")) color = COLOR_GRAY_LT;
    else                                color = COLOR_GRAY_LT;  // OFF/TO/GA fallback

    if (preselect_flash_visible) {
        draw_7seg_number(PRESEL_RIGHT_X, PRESEL_Y,
                         PRESEL_CELL_W, PRESEL_CELL_H, PRESEL_CELL_T,
                         PRESEL_SPACING, s.preselected, 3,
                         color, COLOR_BLACK);
    } else {
        // Flash off phase: erase digits only; separator stays drawn
        tft_fill_rect(left_x, PRESEL_Y, total_w, PRESEL_CELL_H, COLOR_BLACK);
    }
}

// --- Top-level render orchestrator with screen-mode + dirty tracking ---

// Unconditionally redraws every region of the MAIN layout. Called on
// transitions into SCREEN_MAIN from another screen mode.
void render_main_screen_full(const struct DisplayState& s) {
    tft_fill_rect(0, 0, 240, 240, COLOR_BLACK);
    render_mode_label(s);
    render_target_digits(s);
    render_preselect(s);
    render_phase(s);
    render_ias(s);
    render_engagement(s);
    rendered_flash_state = preselect_flash_visible;   // sync after full redraw
}

void render(const struct DisplayState& s, const struct DisplayState& prev) {
    // Screen-mode transition: full-screen erase + re-render in new mode.
    if (current_screen != prev_screen) {
        switch (current_screen) {
            case SCREEN_BOOT:    render_boot_screen();         break;
            case SCREEN_OFF:     render_off_screen();          break;
            case SCREEN_MAIN:    render_main_screen_full(s);   break;
        }
        prev_screen = current_screen;
        nodata_dot_rendered = false;   // mode-transition wiped the dot
        return;                         // no per-region work on transition frame
    }

    // BOOT / OFF are static once drawn — nothing to redraw.
    if (current_screen != SCREEN_MAIN) {
        return;
    }

    // SCREEN_MAIN: per-region dirty-tracked redraws only.
    bool state_changed        = strcmp(s.state, prev.state) != 0;
    bool target_changed       = s.target != prev.target;
    bool preselected_changed  = s.preselected != prev.preselected;
    bool ias_changed          = s.ias != prev.ias;
    bool phase_changed        = strcmp(s.phase, prev.phase) != 0;
    bool envelope_changed     = strcmp(s.envelope, prev.envelope) != 0;
    bool flash_dirty          = preselect_flash_visible != rendered_flash_state;

    // Banner — state OR phase change (phase affects ARMED+TOGA/GA variants)
    if (state_changed || phase_changed) {
        render_mode_label(s);
    }

    if (state_changed || target_changed || envelope_changed) {
        render_target_digits(s);
    }

    if (state_changed || target_changed || preselected_changed || flash_dirty) {
        render_preselect(s);
        rendered_flash_state = preselect_flash_visible;
    }

    if (state_changed || phase_changed) {
        render_phase(s);
    }

    if (ias_changed) {
        render_ias(s);
    }

    if (state_changed || envelope_changed) {
        render_engagement(s);
    }

    // Dirty-tracked region renders (notably render_engagement) can wipe
    // the dot's pixels. Re-assert it here if it was visible before the
    // render pass. If hardware shows flicker, switch to clearing
    // nodata_dot_rendered=false and letting update_nodata_dot() redraw.
    if (nodata_dot_rendered) {
        render_nodata_dot(true);
    }
}

// ====================================================================
// E37 HARDWARE QUADRATURE ENCODER SETUP
// ====================================================================
HardwareTimer* outer_timer = nullptr;
HardwareTimer* inner_timer = nullptr;

int32_t outer_last_count = 0;
int32_t inner_last_count = 0;

void encoder_init() {
    // OUTER — TIM2, CH1/CH2 on PA0/PA1
    // INNER — TIM3, CH1/CH2 on PB4/PB5

    pinMode(OUTER_A, INPUT_PULLUP);
    pinMode(OUTER_B, INPUT_PULLUP);
    pinMode(INNER_A, INPUT_PULLUP);
    pinMode(INNER_B, INPUT_PULLUP);

    // Configure TIM2 for encoder mode
    TIM_TypeDef* t2 = TIM2;
    outer_timer = new HardwareTimer(t2);
    outer_timer->pause();
    outer_timer->setMode(1, TIMER_INPUT_CAPTURE_RISING, OUTER_A);
    outer_timer->setMode(2, TIMER_INPUT_CAPTURE_RISING, OUTER_B);
    // Switch to encoder mode (TI1+TI2, x4)
    t2->SMCR = (t2->SMCR & ~0x7) | 0x3;  // SMS = 011 = encoder mode 3
    t2->CCMR1 = 0x0101;  // CC1S=01, CC2S=01 (IC1→TI1, IC2→TI2)
    t2->CCER  = 0x0011;  // Enable CC1, CC2 (rising)
    t2->ARR   = 0xFFFF;
    t2->CNT   = 0x8000;  // center so we can detect +/-
    t2->CR1  |= 0x0001;  // CEN
    outer_last_count = 0x8000;

    // Configure TIM3 for encoder mode
    TIM_TypeDef* t3 = TIM3;
    inner_timer = new HardwareTimer(t3);
    inner_timer->pause();
    inner_timer->setMode(1, TIMER_INPUT_CAPTURE_RISING, INNER_A);
    inner_timer->setMode(2, TIMER_INPUT_CAPTURE_RISING, INNER_B);
    t3->SMCR = (t3->SMCR & ~0x7) | 0x3;
    t3->CCMR1 = 0x0101;
    t3->CCER  = 0x0011;
    t3->ARR   = 0xFFFF;
    t3->CNT   = 0x8000;
    t3->CR1  |= 0x0001;
    inner_last_count = 0x8000;
}

int32_t encoder_read_delta(TIM_TypeDef* t, int32_t& last_count) {
    int32_t now = (int32_t)(t->CNT);
    int32_t delta = now - last_count;
    // Handle wrap
    if (delta >  0x4000) delta -= 0x10000;
    if (delta < -0x4000) delta += 0x10000;
    last_count = now;
    return delta;
}

// ====================================================================
// SERIAL EMISSION HELPERS
// ====================================================================
void emit_version() {
    Serial.print("@ATD:VER:");
    Serial.println(FW_VERSION);
}
void emit_ack() {
    Serial.println("@ATD:ACK");
}
void emit_inner(int detents) {
    Serial.print("@ATD:INNER:");
    if (detents > 0) Serial.print('+');
    Serial.println(detents);
}
void emit_outer(int detents) {
    Serial.print("@ATD:OUTER:");
    if (detents > 0) Serial.print('+');
    Serial.println(detents);
}
void emit_button_press() {
    Serial.println("@ATD:BTN:PRESS");
}
void emit_error(const char* code) {
    Serial.print("@ATD:ERR:");
    Serial.println(code);
}

// ====================================================================
// PROTOCOL PARSER — compound @ATD: state messages
// Format (v0.5.0, 7 fields):
//   @ATD:<state>,<mode>,<active>,<preselected>,<ias>,<phase>,<envelope>\n
// ====================================================================
bool parse_compound_state(const char* msg) {
    // msg starts after "@ATD:"
    // Example: "MAN,MAN,180,195,175,CRUISE,OK"
    char buf[128];
    strncpy(buf, msg, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    char* tok = strtok(buf, ",");
    if (!tok) return false;
    strncpy(cur_state.state, tok, sizeof(cur_state.state) - 1);
    cur_state.state[sizeof(cur_state.state) - 1] = 0;

    tok = strtok(NULL, ",");
    if (!tok) return false;
    strncpy(cur_state.mode, tok, sizeof(cur_state.mode) - 1);
    cur_state.mode[sizeof(cur_state.mode) - 1] = 0;

    tok = strtok(NULL, ",");
    if (!tok) return false;
    cur_state.target = atoi(tok);

    tok = strtok(NULL, ",");
    if (!tok) return false;
    cur_state.preselected = atoi(tok);

    tok = strtok(NULL, ",");
    if (!tok) return false;
    cur_state.ias = atoi(tok);

    tok = strtok(NULL, ",");
    if (!tok) return false;
    strncpy(cur_state.phase, tok, sizeof(cur_state.phase) - 1);
    cur_state.phase[sizeof(cur_state.phase) - 1] = 0;

    tok = strtok(NULL, ",\r\n");
    if (!tok) return false;
    strncpy(cur_state.envelope, tok, sizeof(cur_state.envelope) - 1);
    cur_state.envelope[sizeof(cur_state.envelope) - 1] = 0;

    return true;
}

void handle_line(const char* line) {
    // Trim \r
    static char trimmed[SERIAL_RX_BUF];
    int i = 0;
    while (line[i] && line[i] != '\r' && line[i] != '\n' && i < SERIAL_RX_BUF - 1) {
        trimmed[i] = line[i]; i++;
    }
    trimmed[i] = 0;

    if (strncmp(trimmed, "@ATD:", 5) != 0) {
        // Not our prefix — ignore silently
        return;
    }

    const char* payload = trimmed + 5;

    // Known non-state messages first
    if (strncmp(payload, "PING", 4) == 0) {
        emit_version();
        return;
    }

    // v0.5.6: @ATD:BRT:nnn — runtime brightness setter (0-255)
    if (strncmp(payload, "BRT:", 4) == 0) {
        int value = atoi(payload + 4);
        if (value < 0)   value = 0;
        if (value > 255) value = 255;
        set_brightness((uint8_t)value);
        Serial.print("@ATD:BRT:ACK:");
        Serial.println(value);
        return;
    }

    // v0.5.6: @ATD:LOGO — render logo and hold (brightness validation target).
    // Exits when any subsequent compound state message clears logo_hold_active.
    if (strncmp(payload, "LOGO", 4) == 0) {
        logo_hold_active = true;
        current_screen   = SCREEN_BOOT;
        render_boot_screen();
        prev_screen      = SCREEN_BOOT;   // suppress redundant transition redraw next tick
        Serial.println("@ATD:LOGO:ACK");
        return;
    }

    // Default: attempt to parse as compound state message
    if (!parse_compound_state(payload)) {
        emit_error("PARSE");
        return;
    }

    // Valid state message — update PC liveness tracking
    extern uint32_t last_pc_msg_ms;
    extern bool     pc_ever_connected;
    last_pc_msg_ms = millis();
    pc_ever_connected = true;
    cur_state.no_data = false;  // clear NO DATA watchdog flag
    // v0.5.6: any successful state message exits @ATD:LOGO hold mode.
    logo_hold_active = false;
}

void serial_poll() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rx_len > 0) {
                rx_buf[rx_len] = 0;
                handle_line(rx_buf);
                rx_len = 0;
            }
        } else if (rx_len < SERIAL_RX_BUF - 1) {
            rx_buf[rx_len++] = c;
        } else {
            // buffer overrun
            emit_error("OVERRUN");
            rx_len = 0;
        }
    }
}

// ====================================================================
// SETUP / LOOP
// ====================================================================
uint32_t last_pc_msg_ms    = 0;
uint32_t last_heartbeat_ms = 0;
uint32_t last_enc_poll_ms  = 0;
uint32_t last_btn_change_ms = 0;
int      last_btn_state    = HIGH;
int      stable_btn_state  = HIGH;
bool     pc_ever_connected = false;

// Accumulated detents between emissions
int32_t inner_accum_raw = 0;
int32_t outer_accum_raw = 0;

// Read hardware counters, accumulate, and emit detent events.
// No time gate — caller decides cadence.
void poll_encoders() {
    int32_t outer_delta = encoder_read_delta(TIM2, outer_last_count);
    int32_t inner_delta = encoder_read_delta(TIM3, inner_last_count);

    outer_accum_raw += outer_delta;
    inner_accum_raw += inner_delta;

    int outer_detents = outer_accum_raw / DETENTS_PER_COUNT;
    if (outer_detents != 0) {
        outer_accum_raw -= outer_detents * DETENTS_PER_COUNT;
        emit_outer(outer_detents);
    }
    int inner_detents = inner_accum_raw / DETENTS_PER_COUNT;
    if (inner_detents != 0) {
        inner_accum_raw -= inner_detents * DETENTS_PER_COUNT;
        emit_inner(inner_detents);
    }
}

// Decides which full-screen mode we should be on this tick. Reads only
// boot timestamp, pc_ever_connected, and cur_state.state.
//
// v0.5.3: "no PC App yet" and cur_state.no_data no longer force a
// full-screen takeover — the dot overlay handles "waiting for comms"
// signaling on top of whatever mode is active. The dark screen is
// equivalent for both "never connected" and "OFF" cases: the pilot
// distinguishes them by the presence of the flashing dot.
ScreenMode decide_screen_mode(uint32_t now) {
    if (logo_hold_active) {
        return SCREEN_BOOT;
    }
    if (now - boot_screen_start_ms < BOOT_SCREEN_MS) {
        return SCREEN_BOOT;
    }
    if (!pc_ever_connected || !strcmp(cur_state.state, "OFF")) {
        return SCREEN_OFF;
    }
    return SCREEN_MAIN;
}

// Updates the NO DATA dot flash state. Called every loop() iteration —
// the dot has its own 1Hz cadence independent of render() calls.
// Suppressed during the boot splash so the logo stays clean.
void update_nodata_dot(uint32_t now) {
    // No dot during the boot splash — keep the logo clean.
    if (current_screen == SCREEN_BOOT) {
        if (nodata_dot_rendered) {
            render_nodata_dot(false);
            nodata_dot_rendered = false;
        }
        nodata_dot_visible = false;
        return;
    }

    // Should the dot be flashing at all?
    bool should_flash = cur_state.no_data || !pc_ever_connected;

    if (!should_flash) {
        if (nodata_dot_rendered) {
            render_nodata_dot(false);
            nodata_dot_rendered = false;
        }
        nodata_dot_visible = false;
        return;
    }

    // Toggle visibility every NODATA_FLASH_MS
    if (now - nodata_dot_toggle_ms >= NODATA_FLASH_MS) {
        nodata_dot_visible   = !nodata_dot_visible;
        nodata_dot_toggle_ms = now;
    }

    // Render only if visibility state diverged from what's on screen
    if (nodata_dot_visible != nodata_dot_rendered) {
        render_nodata_dot(nodata_dot_visible);
        nodata_dot_rendered = nodata_dot_visible;
    }
}

void setup() {
    Serial.begin(250000);
    delay(1000);

    tft_init_hw();
    tft_fill_screen(COLOR_BLACK);

    // Display backlight PWM (v0.5.6). Harmless until BLK is physically
    // wired from PB1 to the display module — until then this drives an
    // unconnected pin.
    pinMode(DISPLAY_BLK_PIN, OUTPUT);
    set_brightness(BRIGHTNESS_DEFAULT);

    // Encoder + button setup
    encoder_init();
    pinMode(BTN_PIN, INPUT_PULLUP);

    // Emit version immediately so PC knows we're alive
    emit_version();
    last_heartbeat_ms   = millis();
    last_enc_poll_ms    = millis();
    boot_screen_start_ms = millis();

    // Force a screen-mode transition on the first loop() iteration so the
    // boot screen (logo) is rendered via the normal render() path.
    current_screen = SCREEN_BOOT;
    prev_screen    = SCREEN_MAIN;
}

void loop() {
    uint32_t now = millis();

    // --- Serial RX -----------------------------------------------
    serial_poll();

    // --- Heartbeat -----------------------------------------------
    if (now - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
        emit_ack();
        last_heartbeat_ms = now;
    }

    // --- Encoder polling -----------------------------------------
    if (now - last_enc_poll_ms >= ENCODER_POLL_MS) {
        last_enc_poll_ms = now;
        poll_encoders();
    }

    // --- Preselect flash toggle ----------------------------------
    if (cur_state.target != cur_state.preselected &&
        now - preselect_flash_toggle_ms >= FLASH_INTERVAL_MS) {
        preselect_flash_visible = !preselect_flash_visible;
        preselect_flash_toggle_ms = now;
    }

    // --- Button debounce -----------------------------------------
    int raw = digitalRead(BTN_PIN);
    if (raw != last_btn_state) {
        last_btn_change_ms = now;
        last_btn_state = raw;
    }
    if (now - last_btn_change_ms >= BUTTON_DEBOUNCE_MS) {
        if (raw != stable_btn_state) {
            stable_btn_state = raw;
            if (stable_btn_state == LOW) {
                // Press (active-low with pullup)
                emit_button_press();
            }
            // No release event per spec
        }
    }

    // --- TIMEOUT detection ---------------------------------------
    // v0.5.7: saturated subtraction guards against the case where
    // last_pc_msg_ms was set inside serial_poll() to a millis() value
    // LATER than `now` captured at top of loop. In that case, the raw
    // unsigned subtraction underflows to ~UINT32_MAX, spuriously
    // triggering TIMEOUT. The saturated form returns 0 when the message
    // just arrived, which is the correct semantic (zero age = fresh).
    if (pc_ever_connected) {
        uint32_t msg_age = (now >= last_pc_msg_ms) ? (now - last_pc_msg_ms) : 0;
        bool want_no_data = (msg_age > TIMEOUT_MS);
        if (want_no_data != cur_state.no_data) {
            cur_state.no_data = want_no_data;
            if (want_no_data) {
                emit_error("TIMEOUT");
            }
        }
    }

    // --- Screen-mode decision ------------------------------------
    current_screen = decide_screen_mode(now);

    // --- Render when something that affects the display changes --
    bool changed =
        current_screen != prev_screen ||
        strcmp(cur_state.state,    prev_state.state)    != 0 ||
        strcmp(cur_state.mode,     prev_state.mode)     != 0 ||
        cur_state.target      != prev_state.target      ||
        cur_state.preselected != prev_state.preselected ||
        cur_state.ias         != prev_state.ias         ||
        strcmp(cur_state.phase,    prev_state.phase)    != 0 ||
        strcmp(cur_state.envelope, prev_state.envelope) != 0 ||
        cur_state.no_data != prev_state.no_data ||
        preselect_flash_visible != rendered_flash_state;

    if (changed) {
        // Drain any detents that arrived during the prior render
        // (SPI blocks the loop for ms at a time).
        poll_encoders();
        render(cur_state, prev_state);
        prev_state = cur_state;
    }

    // --- NO DATA dot overlay (independent 1Hz cadence) -----------
    // Runs outside the render() "changed" gate because the dot flashes
    // even when no state change triggers a redraw.
    update_nodata_dot(now);
}

// ====================================================================
// End of firmware
// ====================================================================
