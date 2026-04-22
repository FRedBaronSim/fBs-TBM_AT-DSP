// ====================================================================
// FBSiM AT-DSP v0.4.2 — Encoder responsiveness: tighter poll + pre-render drain
//
// Changes from v0.4.1:
//   - ENCODER_POLL_MS 10→2 for lower detent latency
//   - poll_encoders() extracted; second call before render() drains detents
//     that arrived during the prior (SPI-blocking) render
// ====================================================================

#include <SPI.h>
#include <HardwareTimer.h>

// ---- Version ------------------------------------------------------
static const char* FW_VERSION = "0.4.2";

// ---- Display pins -------------------------------------------------
#define TFT_CS   PA3
#define TFT_DC   PA2
#define TFT_RST  PB0

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
#define COLOR_MAGENTA  0xF81F
#define COLOR_GREEN    0x07E0
#define COLOR_AMBER    0xFD20   // G3000 caution amber
#define COLOR_RED      0xF800
#define COLOR_GRAY_LT  0xC618
#define COLOR_GRAY_DK  0x4208

// ---- Encoder tuning ----------------------------------------------
// x4 quadrature decoding — 4 counts per mechanical detent on most E37s.
// If your unit reads 2 detents per click, change to 2.
static const int DETENTS_PER_COUNT = 4;

// ---- Timings ------------------------------------------------------
static const uint32_t HEARTBEAT_INTERVAL_MS    = 2000;   // send @ATD:ACK
static const uint32_t TIMEOUT_MS               = 5000;   // PC silent → NO DATA
static const uint32_t DEMO_START_IF_NO_PC_MS   = 10000;  // no PC ever → demo
static const uint32_t ENCODER_POLL_MS          = 2;      // hardware counter poll
static const uint32_t BUTTON_DEBOUNCE_MS       = 20;

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
    int  target;
    int  ias;
    char phase[12];
    char envelope[8];
    bool no_data;    // overlay NO DATA on top of last state
};

DisplayState cur_state  = {"OFF", "-", 0, 0, "OFF", "OK", false};
DisplayState prev_state = {"----", "-", -1, -1, "----", "----", true};  // force initial redraw

// ====================================================================
// LAYOUT CONSTANTS (v0.4.1 retuned for phase row)
// ====================================================================
#define MODE_Y            16      // mode label (scale 3)
#define MODE_SCALE        3
#define MODE_H_PX         (MODE_SCALE * 7)   // 21

#define TGT_CELL_W        40
#define TGT_CELL_H        60      // was 64 — trimmed to make room
#define TGT_CELL_T        8
#define TGT_SPACING       10
#define TGT_Y             50      // was 78
#define TGT_RIGHT_X       190

#define PHASE_Y           120     // NEW — phase label, small, dim white
#define PHASE_SCALE       1
#define PHASE_H_PX        7

#define CUR_Y             140     // was 160
#define CUR_SCALE         2
#define CUR_H_PX          (CUR_SCALE * 7)    // 14

#define ENG_DOT_Y         180     // was 200
#define ENG_DOT_R         4
#define ENG_LBL_SCALE     2

#define NODATA_Y          210     // was 220
#define NODATA_SCALE      1

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

// Substitute "SPD" for MAN when rendering (real TBM 940 labels it SPD)
const char* display_label(const char* s) {
    if (!strcmp(s, "MAN")) return "SPD";
    return s;
}

// --- Per-region renderers -----------------------------------------

void render_mode_label(const DisplayState& s) {
    uint16_t color = color_for_state(s.state);
    tft_fill_rect(0, MODE_Y - 2, 240, MODE_H_PX + 4, COLOR_BLACK);
    draw_string_centered(120, MODE_Y, display_label(s.state),
                         MODE_SCALE, color, COLOR_BLACK);
}

void render_target_digits(const DisplayState& s) {
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

void render_phase(const DisplayState& s) {
    tft_fill_rect(0, PHASE_Y - 1, 240, PHASE_H_PX + 2, COLOR_BLACK);
    // Don't draw phase for OFF/ARMED (nothing meaningful)
    if (!state_is_active(s.state)) return;
    if (!strcmp(s.phase, "OFF") || strlen(s.phase) == 0) return;
    draw_string_centered(120, PHASE_Y, s.phase, PHASE_SCALE,
                         COLOR_GRAY_LT, COLOR_BLACK);
}

void render_ias(const DisplayState& s) {
    tft_fill_rect(0, CUR_Y - 2, 240, CUR_H_PX + 4, COLOR_BLACK);
    char buf[16];
    snprintf(buf, sizeof(buf), "IAS %d", s.ias);
    draw_string_centered(120, CUR_Y, buf, CUR_SCALE,
                         COLOR_WHITE, COLOR_BLACK);
}

void render_engagement(const DisplayState& s) {
    bool active = state_is_active(s.state);
    bool envelope_alert = strcmp(s.envelope, "OK") != 0;

    // Clear the dot + label area
    tft_fill_rect(80, ENG_DOT_Y - 6, 120, ENG_LBL_SCALE * 7 + 4, COLOR_BLACK);

    uint16_t dot_color = active ? COLOR_GREEN : COLOR_GRAY_DK;
    tft_fill_rect(100 - ENG_DOT_R, ENG_DOT_Y - ENG_DOT_R,
                  2 * ENG_DOT_R, 2 * ENG_DOT_R, dot_color);

    if (envelope_alert && active) {
        draw_string(115, ENG_DOT_Y - 6, s.envelope,
                    ENG_LBL_SCALE, COLOR_AMBER, COLOR_BLACK);
    } else {
        draw_string(115, ENG_DOT_Y - 6, "ENG",
                    ENG_LBL_SCALE, dot_color, COLOR_BLACK);
    }
}

void render_no_data(const DisplayState& s) {
    tft_fill_rect(40, NODATA_Y - 2, 160, 7 + 4, COLOR_BLACK);
    if (s.no_data) {
        draw_string_centered(120, NODATA_Y, "NO DATA",
                             NODATA_SCALE, COLOR_AMBER, COLOR_BLACK);
    }
}

// --- Top-level render orchestrator with dirty tracking ------------

void render(const struct DisplayState& s, const struct DisplayState& prev) {
    // Compute per-region dirty flags
    bool state_changed    = strcmp(s.state, prev.state) != 0;
    bool mode_changed     = strcmp(s.mode, prev.mode) != 0;
    bool target_changed   = s.target != prev.target;
    bool ias_changed      = s.ias != prev.ias;
    bool phase_changed    = strcmp(s.phase, prev.phase) != 0;
    bool envelope_changed = strcmp(s.envelope, prev.envelope) != 0;
    bool nodata_changed   = s.no_data != prev.no_data;

    // Mode label — redraw on state change (color + label content both depend on state)
    if (state_changed) {
        render_mode_label(s);
    }

    // Target digits — redraw on state, target, or envelope change (color shift on envelope)
    if (state_changed || target_changed || envelope_changed) {
        render_target_digits(s);
    }

    // Phase row — redraw on phase or state change (hidden in OFF/ARMED)
    if (state_changed || phase_changed) {
        render_phase(s);
    }

    // IAS — redraw on change only
    if (ias_changed) {
        render_ias(s);
    }

    // Engagement + envelope label — redraw on state, envelope changes
    if (state_changed || envelope_changed) {
        render_engagement(s);
    }

    // NO DATA banner — redraw only when flag changes
    if (nodata_changed) {
        render_no_data(s);
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
// Format: @ATD:<state>,<mode>,<target>,<ias>,<phase>,<envelope>\n
// ====================================================================
bool parse_compound_state(const char* msg) {
    // msg starts after "@ATD:"
    // Example: "OFF,MAN,120,145,OFF,OK"
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

    // Default: attempt to parse as compound state message
    if (!parse_compound_state(payload)) {
        emit_error("PARSE");
        return;
    }

    // Valid state message — update PC liveness tracking
    extern uint32_t last_pc_msg_ms;
    extern bool     pc_ever_connected;
    extern bool     in_demo_mode;
    last_pc_msg_ms = millis();
    pc_ever_connected = true;
    in_demo_mode = false;  // PC took over, exit demo if we were in it
    cur_state.no_data = false;  // clear NO DATA overlay
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
// DEMO CYCLE (only runs if PC never connected within DEMO_START timeout)
// ====================================================================
struct DemoFrame {
    const char* state; const char* mode;
    int target; int ias;
    const char* phase; const char* envelope;
    uint32_t hold_ms;
};
const DemoFrame demo_seq[] = {
    {"OFF",   "-",   0,   0,   "OFF",    "OK",  1500},
    {"ARMED", "MAN", 180, 120, "OFF",    "OK",  2500},   // NEW — shows ARMED distinct
    {"MAN",   "MAN", 120, 75,  "CLIMB",  "OK",  2500},
    {"MAN",   "MAN", 140, 118, "CLIMB",  "OK",  2500},
    {"FLC",   "FLC", 180, 165, "CLIMB",  "OK",  2500},
    {"FLC",   "FLC", 220, 198, "CRUISE", "OK",  2500},
    {"FLC",   "FLC", 220, 235, "CRUISE", "OVR", 2500},   // envelope alert
    {"TO",    "-",   110, 95,  "TOGA",   "OK",  2500},
    {"GA",    "-",   130, 115, "GA",     "OK",  2500},
    {"ARMED", "FLC", 200, 210, "OFF",    "OK",  2500},   // NEW — second ARMED variant
    {"OFF",   "-",   0,   165, "OFF",    "OK",  2000},
};
const int demo_count = sizeof(demo_seq) / sizeof(DemoFrame);

void apply_demo_frame(int idx) {
    const DemoFrame& f = demo_seq[idx];
    strncpy(cur_state.state,    f.state,    sizeof(cur_state.state) - 1);
    strncpy(cur_state.mode,     f.mode,     sizeof(cur_state.mode) - 1);
    cur_state.target = f.target;
    cur_state.ias    = f.ias;
    strncpy(cur_state.phase,    f.phase,    sizeof(cur_state.phase) - 1);
    strncpy(cur_state.envelope, f.envelope, sizeof(cur_state.envelope) - 1);
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
uint32_t demo_last_frame_ms = 0;
int      demo_frame_idx    = 0;
bool     pc_ever_connected = false;
bool     in_demo_mode      = false;

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

void setup() {
    Serial.begin(250000);
    delay(1000);

    tft_init_hw();

    // Splash screen
    tft_fill_screen(COLOR_BLACK);
    draw_string_centered(120,  55, "FBSIM",   4, COLOR_CYAN,    COLOR_BLACK);
    draw_string_centered(120, 105, "AT-DSP",  3, COLOR_WHITE,   COLOR_BLACK);
    char ver[16];
    snprintf(ver, sizeof(ver), "V%s", FW_VERSION);
    draw_string_centered(120, 150, ver, 2, COLOR_GRAY_LT, COLOR_BLACK);
    delay(1200);
    tft_fill_screen(COLOR_BLACK);

    // Encoder + button setup
    encoder_init();
    pinMode(BTN_PIN, INPUT_PULLUP);

    // Emit version immediately so PC knows we're alive
    emit_version();
    last_heartbeat_ms = millis();
    last_enc_poll_ms  = millis();

    // Initial render of default state
    render(cur_state, prev_state);
    prev_state = cur_state;
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

    // --- Demo mode management ------------------------------------
    if (!pc_ever_connected && (now > DEMO_START_IF_NO_PC_MS) && !in_demo_mode) {
        in_demo_mode = true;
        demo_frame_idx = 0;
        demo_last_frame_ms = now;
        apply_demo_frame(demo_frame_idx);
    }
    if (in_demo_mode) {
        const DemoFrame& f = demo_seq[demo_frame_idx];
        if (now - demo_last_frame_ms >= f.hold_ms) {
            demo_frame_idx = (demo_frame_idx + 1) % demo_count;
            demo_last_frame_ms = now;
            apply_demo_frame(demo_frame_idx);
        }
    }

    // --- TIMEOUT detection ---------------------------------------
    if (pc_ever_connected) {
        bool want_no_data = (now - last_pc_msg_ms > TIMEOUT_MS);
        if (want_no_data != cur_state.no_data) {
            cur_state.no_data = want_no_data;
            if (want_no_data) {
                emit_error("TIMEOUT");
            }
        }
    }

    // --- Render when state changes -------------------------------
    bool changed =
        strcmp(cur_state.state,    prev_state.state)    != 0 ||
        strcmp(cur_state.mode,     prev_state.mode)     != 0 ||
        cur_state.target != prev_state.target ||
        cur_state.ias    != prev_state.ias    ||
        strcmp(cur_state.phase,    prev_state.phase)    != 0 ||
        strcmp(cur_state.envelope, prev_state.envelope) != 0 ||
        cur_state.no_data != prev_state.no_data;

    if (changed) {
        // Drain any detents that arrived during the prior render
        // (SPI blocks the loop for ms at a time).
        poll_encoders();
        render(cur_state, prev_state);
        prev_state = cur_state;
    }
}

// ====================================================================
// End of firmware
// ====================================================================
