/*********************************************************************************
 * SKODA FABIA MK2 (2008) - MOTOR BZG 1.2 12V - TEST CONEXIUNE ELM327 BLUETOOTH
 *********************************************************************************
 * Scop:
 *   Sketch de TEST pentru a verifica daca ESP32-ul se poate conecta prin
 *   Bluetooth Classic (SPP) la un adaptor ELM327 introdus in portul OBD2
 *   al masinii, si daca poate citi PID-uri OBD-II STANDARD (Mode 01).
 *
 *   Spre deosebire de proiectul BMW din acest repo (care citeste direct CAN-ul
 *   pentru date proprietare precum presiune ulei/baterie), aici folosim EXCLUSIV
 *   PID-uri OBD-II standard, pentru ca astea sunt tot ce poate oferi un ELM327
 *   "din cutie" fara sa stim ID-urile CAN proprietare Skoda/VAG. Motorul BZG
 *   foloseste senzor MAP (speed-density), deci PID-ul MAF (0x10) NU va avea
 *   date valide - de aceea citim MAP (0x0B) in loc de MAF.
 *
 * Hardware (identic cu Factory_samples_without_touch):
 *   - ESP32 WROOM-32 (Bluetooth Classic obligatoriu -> NU merge pe ESP32-S3/C3!)
 *   - TFT ST7789 240x320, bus paralel 8-bit, acelasi pinout ca restul proiectului
 *   - Adaptor ELM327 Bluetooth in portul OBD2 al masinii
 *
 * Librarii necesare (Arduino Library Manager / ESP32 core):
 *   - LovyanGFX
 *   - lvgl (v8.x) - foloseste lv_conf.h din "Projects/LVGL configuration file"
 *     (copiaza-l in folderul librariilor Arduino, langa folderul lvgl)
 *   - BluetoothSerial - vine inclus in ESP32 core, nu trebuie instalat separat
 *
 * Pasi de test (vezi si README.md din acest folder):
 *   1. Seteaza ELM_DEVICE_NAME / USE_MAC_ADDRESS mai jos.
 *   2. Deschide Serial Monitor la 115200 baud - tot ce se intampla e logat acolo.
 *   3. Poti testa mai intai FARA masina: doar impaierea Bluetooth si comenzile
 *      AT (ATZ/ATRV) merg fara alimentare de la OBD2 daca adaptorul are baterie
 *      proprie; altfel ELM327-ul are nevoie de +12V de la portul OBD2 ca sa
 *      porneasca, deci contactul masinii trebuie sa fie macar pe pozitia ON.
 *   4. In masina: contact ON (nu neaparat motor pornit) -> ESP32-ul se conecteaza
 *      automat si incepe sa afiseze RPM/temperatura/etc.
 *********************************************************************************/

#include <Arduino.h>
#include <lvgl.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth Classic nu e activat pe acest target. Foloseste o placa ESP32 clasica (WROOM/WROVER), nu S3/C3/C6.
#endif

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

/******************** CONFIG ELM327 ********************/
// Numele tipic difuzat de clonele ieftine ELM327. Daca al tau se numeste altfel
// (ex. "OBDLink", "Vlink", "V-LINK"), schimba-l aici - il vezi in setarile
// Bluetooth ale telefonului cand faci pairing manual prima data.
#define ELM_DEVICE_NAME "OBDII"
#define ELM_PIN "1234"          // PIN implicit; unele clone folosesc "0000" sau "6789"

// Daca "dupa nume" nu functioneaza fiabil, pune adresa MAC exacta gasita prin
// telefon (Setari > Bluetooth > pairing > detalii dispozitiv) si seteaza true.
#define USE_MAC_ADDRESS false
uint8_t ELM_MAC[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

BluetoothSerial SerialBT;
bool elmLinkUp = false;      // conexiune Bluetooth SPP activa
bool elmEcuOk = false;       // ECU a raspuns la interogari PID
uint32_t lastReconnectAttempt = 0;
const uint32_t RECONNECT_INTERVAL_MS = 5000;

/******************** DISPLAY (acelasi wiring ca Factory_samples_without_touch) ****/
#define TFT_W 320
#define TFT_H 240

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_Parallel8 _bus;
public:
  LGFX(void) {
    {
      auto cfg = _bus.config();
      cfg.freq_write = 25000000;
      cfg.pin_wr = 4;
      cfg.pin_rd = 2;
      cfg.pin_rs = 16;
      cfg.pin_d0 = 15; cfg.pin_d1 = 13; cfg.pin_d2 = 12; cfg.pin_d3 = 14;
      cfg.pin_d4 = 27; cfg.pin_d5 = 25; cfg.pin_d6 = 33; cfg.pin_d7 = 32;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs = 17;
      cfg.pin_rst = -1;
      cfg.pin_busy = -1;
      cfg.panel_width = 240;
      cfg.panel_height = 320;
      cfg.offset_x = 0; cfg.offset_y = 0; cfg.offset_rotation = 0;
      cfg.readable = false; cfg.invert = false; cfg.rgb_order = false;
      cfg.dlen_16bit = false; cfg.bus_shared = true;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};
static LGFX tft;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1, *buf2;

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  int w = (area->x2 - area->x1 + 1);
  int h = (area->y2 - area->y1 + 1);
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.writePixels(&color_p->full, w * h, false);
  tft.endWrite();
  lv_disp_flush_ready(disp);
}

/******************** UI ********************/
lv_obj_t *lbl_status;
lv_style_t style_title, style_value, style_status;

struct Cell {
  lv_obj_t *val;
  const char *unit;
};
Cell cell_rpm, cell_coolant, cell_speed, cell_load;
Cell cell_intake, cell_map, cell_throttle, cell_batt;

static Cell create_cell(lv_obj_t *parent, const char *name, const char *unit, int x, int y) {
  lv_obj_t *t = lv_label_create(parent);
  lv_label_set_text(t, name);
  lv_obj_add_style(t, &style_title, 0);
  lv_obj_set_pos(t, x, y);

  lv_obj_t *v = lv_label_create(parent);
  lv_label_set_text(v, "--");
  lv_obj_add_style(v, &style_value, 0);
  lv_obj_set_pos(v, x, y + 16);

  Cell c;
  c.val = v;
  c.unit = unit;
  return c;
}

static void set_cell(Cell &c, int value) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%d%s", value, c.unit);
  lv_label_set_text(c.val, buf);
}

static void set_cell_err(Cell &c) {
  lv_label_set_text(c.val, "--");
}

static void create_dashboard() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  lv_style_init(&style_status);
  lv_style_set_text_color(&style_status, lv_color_hex(0xFFFF00));
  lv_style_set_text_font(&style_status, &lv_font_montserrat_14);
  lv_style_set_bg_opa(&style_status, LV_OPA_TRANSP);
  lv_style_set_border_width(&style_status, 0);

  lv_style_init(&style_title);
  lv_style_set_text_color(&style_title, lv_color_hex(0xFF6A00));
  lv_style_set_text_font(&style_title, &lv_font_montserrat_12);
  lv_style_set_bg_opa(&style_title, LV_OPA_TRANSP);
  lv_style_set_border_width(&style_title, 0);

  lv_style_init(&style_value);
  lv_style_set_text_color(&style_value, lv_color_hex(0xFFFFFF));
  lv_style_set_text_font(&style_value, &lv_font_montserrat_20);
  lv_style_set_bg_opa(&style_value, LV_OPA_TRANSP);
  lv_style_set_border_width(&style_value, 0);

  lbl_status = lv_label_create(scr);
  lv_label_set_text(lbl_status, "Pornire...");
  lv_obj_add_style(lbl_status, &style_status, 0);
  lv_obj_set_pos(lbl_status, 5, 4);

  int col1 = 5, col2 = 85, col3 = 165, col4 = 245;
  int row1 = 40, row2 = 130;

  cell_rpm      = create_cell(scr, "RPM",      "",      col1, row1);
  cell_coolant  = create_cell(scr, "COOLANT",  "C",     col2, row1);
  cell_speed    = create_cell(scr, "SPEED",    "km/h",  col3, row1);
  cell_load     = create_cell(scr, "LOAD",     "%",     col4, row1);

  cell_intake   = create_cell(scr, "INTAKE",   "C",     col1, row2);
  cell_map      = create_cell(scr, "MAP",      "kPa",   col2, row2);
  cell_throttle = create_cell(scr, "THROTTLE", "%",     col3, row2);
  cell_batt     = create_cell(scr, "BATTERY",  "V",     col4, row2);
}

static void set_status(const String &s) {
  lv_label_set_text(lbl_status, s.c_str());
  Serial.println(s);
}

/******************** ELM327 - COMUNICATIE ********************/
static String elmSendCommand(const String &cmd, uint16_t timeoutMs = 2000) {
  while (SerialBT.available()) SerialBT.read();   // curata buffer vechi

  SerialBT.print(cmd);
  SerialBT.print('\r');

  String resp;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') break;   // promptul ELM327 = raspuns complet
      resp += c;
    }
  }
  resp.replace("\r", " ");
  resp.replace("\n", " ");
  resp.trim();
  while (resp.indexOf("  ") >= 0) resp.replace("  ", " ");
  return resp;
}

static bool elmInit() {
  struct { const char *cmd; const char *label; } initSeq[] = {
    { "ATZ",  "Reset adaptor" },
    { "ATE0", "Echo OFF" },
    { "ATL0", "Linefeed OFF" },
    { "ATH0", "Headere OFF" },
    { "ATSP0", "Protocol AUTO" },
  };
  for (auto &s : initSeq) {
    String r = elmSendCommand(s.cmd, 3000);
    Serial.printf("[ELM] %-14s -> %s\n", s.label, r.c_str());
    delay(100);
  }

  String r = elmSendCommand("0100", 3000);   // PID-uri suportate 01-20 = "ping" catre ECU
  Serial.printf("[ELM] PID-uri suportate    -> %s\n", r.c_str());
  return r.indexOf("NO DATA") < 0 && r.indexOf("UNABLE") < 0 && r.length() > 0;
}

static bool queryPidBytes(const String &pid, uint8_t *out, int expectedBytes) {
  String r = elmSendCommand(pid, 2000);
  String marker = "41 " + pid.substring(2);
  int idx = r.indexOf(marker);
  if (idx < 0) return false;

  int pos = idx + marker.length();
  for (int i = 0; i < expectedBytes; i++) {
    while (pos < (int)r.length() && r[pos] == ' ') pos++;
    if (pos + 1 >= (int)r.length()) return false;
    out[i] = (uint8_t)strtol(r.substring(pos, pos + 2).c_str(), nullptr, 16);
    pos += 2;
  }
  return true;
}

static bool readRPM(int &v)      { uint8_t b[2]; if (!queryPidBytes("010C", b, 2)) return false; v = ((b[0] * 256) + b[1]) / 4; return true; }
static bool readCoolant(int &v)  { uint8_t b[1]; if (!queryPidBytes("0105", b, 1)) return false; v = b[0] - 40; return true; }
static bool readSpeed(int &v)    { uint8_t b[1]; if (!queryPidBytes("010D", b, 1)) return false; v = b[0]; return true; }
static bool readLoad(int &v)     { uint8_t b[1]; if (!queryPidBytes("0104", b, 1)) return false; v = (b[0] * 100) / 255; return true; }
static bool readIntake(int &v)   { uint8_t b[1]; if (!queryPidBytes("010F", b, 1)) return false; v = b[0] - 40; return true; }
static bool readMAP(int &v)      { uint8_t b[1]; if (!queryPidBytes("010B", b, 1)) return false; v = b[0]; return true; }
static bool readThrottle(int &v) { uint8_t b[1]; if (!queryPidBytes("0111", b, 1)) return false; v = (b[0] * 100) / 255; return true; }

static bool readBatteryVoltage(float &v) {
  // ATRV = comanda specifica adaptorului ELM327 (nu PID OBD), citeste direct
  // tensiunea de pe pinul 16 al OBD2 - functioneaza chiar daca ECU-ul nu raspunde.
  String r = elmSendCommand("ATRV", 2000);
  int idx = r.indexOf('V');
  if (idx < 1) return false;
  v = r.substring(0, idx).toFloat();
  return v > 0.0f;
}

/******************** CONEXIUNE BLUETOOTH ********************/
static bool connectELM() {
  SerialBT.setPin(ELM_PIN);
  Serial.printf("[BT] Conectare la '%s'...\n", ELM_DEVICE_NAME);

  bool ok = USE_MAC_ADDRESS ? SerialBT.connect(ELM_MAC) : SerialBT.connect(ELM_DEVICE_NAME);
  if (!ok) {
    Serial.println("[BT] Conectare esuata.");
    return false;
  }

  Serial.println("[BT] Legatura SPP stabilita, initializez ELM327...");
  return true;
}

/******************** SETUP ********************/
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Skoda Fabia Mk2 (BZG 1.2) - Test ELM327 Bluetooth ===");

  lv_init();
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  buf1 = (lv_color_t *)heap_caps_malloc(TFT_W * 80 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  buf2 = (lv_color_t *)heap_caps_malloc(TFT_W * 80 * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, TFT_W * 80);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = TFT_W;
  disp_drv.ver_res = TFT_H;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  create_dashboard();
  set_status("Bluetooth: pornire...");
  lv_timer_handler();   // forteaza afisarea statusului pe ecran inainte de pasii blocanti de mai jos

  SerialBT.begin("ESP32_Skoda_Test", true);   // true = master, initiaza el conexiunea

  elmLinkUp = connectELM();
  if (elmLinkUp) {
    set_status("Conectat, initializez ELM327...");
    lv_timer_handler();
    elmEcuOk = elmInit();
    set_status(elmEcuOk ? "ECU OK - citesc date" : "Conectat, dar ECU nu raspunde");
  } else {
    set_status("Nu am gasit ELM327 - reincerc...");
  }
  lv_timer_handler();
  lastReconnectAttempt = millis();
}

/******************** LOOP ********************/
void loop() {
  // reconectare automata daca legatura Bluetooth cade
  if (!SerialBT.connected()) {
    elmLinkUp = false;
    elmEcuOk = false;
    if (millis() - lastReconnectAttempt > RECONNECT_INTERVAL_MS) {
      lastReconnectAttempt = millis();
      set_status("Reconectare la ELM327...");
      elmLinkUp = connectELM();
      if (elmLinkUp) {
        elmEcuOk = elmInit();
        set_status(elmEcuOk ? "ECU OK - citesc date" : "Conectat, dar ECU nu raspunde");
      }
    }
  } else if (elmLinkUp) {
    set_status(elmEcuOk ? "OK - live data" : "Conectat, dar ECU nu raspunde");

    int v;
    if (readRPM(v))      set_cell(cell_rpm, v);      else set_cell_err(cell_rpm);
    if (readCoolant(v))  set_cell(cell_coolant, v);  else set_cell_err(cell_coolant);
    if (readSpeed(v))    set_cell(cell_speed, v);    else set_cell_err(cell_speed);
    if (readLoad(v))     set_cell(cell_load, v);     else set_cell_err(cell_load);
    if (readIntake(v))   set_cell(cell_intake, v);   else set_cell_err(cell_intake);
    if (readMAP(v))      set_cell(cell_map, v);      else set_cell_err(cell_map);
    if (readThrottle(v)) set_cell(cell_throttle, v); else set_cell_err(cell_throttle);

    float volt;
    if (readBatteryVoltage(volt)) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%.1fV", volt);
      lv_label_set_text(cell_batt.val, buf);
    } else {
      set_cell_err(cell_batt);
    }
  }

  lv_timer_handler();
  delay(5);
}
