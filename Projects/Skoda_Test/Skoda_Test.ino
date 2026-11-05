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

// Definit AICI, inainte de orice functie: Arduino IDE genereaza automat
// prototipuri pentru functii si le pune inaintea primei functii din fisier.
// Daca struct-ul e definit mai jos, prototipurile care folosesc "Cell" nu compileaza.
struct Cell {
  lv_obj_t *val;
  const char *unit;
};

/******************** CONFIG ELM327 ********************/
// Numele tipic difuzat de clonele ieftine ELM327. Daca al tau se numeste altfel
// (ex. "OBDLink", "Vlink", "V-LINK"), schimba-l aici - il vezi in setarile
// Bluetooth ale telefonului cand faci pairing manual prima data.
#define ELM_DEVICE_NAME "OBDII"
#define ELM_PIN "1234"          // PIN implicit; unele clone folosesc "0000" sau "6789"

// Daca "dupa nume" nu functioneaza fiabil, pune adresa MAC exacta gasita prin
// telefon (Setari > Bluetooth > pairing > detalii dispozitiv) si seteaza true.
#define USE_MAC_ADDRESS true
uint8_t ELM_MAC[6] = { 0x00, 0x1D, 0xA5, 0x0B, 0x00, 0xB4 };   // "OBDII" gasit la scanare

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
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x00FF00), 0);   // fundal verde
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  lv_style_init(&style_status);
  lv_style_set_text_color(&style_status, lv_color_hex(0x000000));
  lv_style_set_text_font(&style_status, &lv_font_montserrat_14);
  lv_style_set_bg_opa(&style_status, LV_OPA_TRANSP);
  lv_style_set_border_width(&style_status, 0);

  lv_style_init(&style_title);
  lv_style_set_text_color(&style_title, lv_color_hex(0x000000));
  lv_style_set_text_font(&style_title, &lv_font_montserrat_12);
  lv_style_set_bg_opa(&style_title, LV_OPA_TRANSP);
  lv_style_set_border_width(&style_title, 0);

  lv_style_init(&style_value);
  lv_style_set_text_color(&style_value, lv_color_hex(0x000000));
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
String lastEcuMsg = "";   // ultimul raspuns/eroare de la ECU, afisat pe ecran
String pidSuffix = "";    // "1" daca adaptorul suporta raspuns rapid (vezi elmFindEcu)

// Trimite o comanda si asteapta promptul '>' al ELM327.
// IMPORTANT: ELM327 abandoneaza comanda curenta daca primeste orice caracter
// inainte sa termine. Daca nu asteptam '>', urmatoarea comanda o strica pe cea
// in curs si toate raspunsurile se decaleaza. De aceea, la timeout mai asteptam
// promptul inca cateva secunde ca sa resincronizam.
static String elmSendCommand(const String &cmd, uint32_t timeoutMs = 4000) {
  while (SerialBT.available()) SerialBT.read();   // curata buffer vechi

  SerialBT.print(cmd);
  SerialBT.print('\r');

  String resp;
  bool gotPrompt = false;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (SerialBT.available()) {
      char c = SerialBT.read();
      if (c == '>') { gotPrompt = true; break; }   // promptul ELM327 = raspuns complet
      resp += c;
    } else {
      delay(1);
    }
  }

  if (!gotPrompt) {
    Serial.printf("[ELM] TIMEOUT la '%s' - astept resincronizare...\n", cmd.c_str());
    uint32_t t = millis();
    while (millis() - t < 8000) {
      if (SerialBT.available() && SerialBT.read() == '>') break;
      delay(1);
    }
  }

  resp.replace("\r", " ");
  resp.replace("\n", " ");
  resp.replace("SEARCHING...", "");
  resp.trim();
  while (resp.indexOf("  ") >= 0) resp.replace("  ", " ");
  Serial.printf("[ELM] %s -> %s\n", cmd.c_str(), resp.c_str());
  return resp;
}

// Incearca sa vorbeasca cu ECU-ul. Prima data protocol AUTO; daca nu merge,
// fortam pe rand protocoalele posibile pe o Fabia 2008 (linie K sau CAN).
static bool elmFindEcu() {
  const char *protocols[] = { "0", "5", "4", "3", "6" };
  // 0=AUTO, 5=KWP2000 fast init, 4=KWP2000 5-baud, 3=ISO9141, 6=CAN 500k
  for (const char *p : protocols) {
    elmSendCommand(String("ATSP") + p, 2000);
    String r = elmSendCommand("0100", 15000);   // initializarea pe linia K poate dura ~10s
    if (r.indexOf("41 00") >= 0) {
      String dp = elmSendCommand("ATDPN", 2000);
      lastEcuMsg = "ECU OK, protocol " + dp;

      // VITEZA: timing adaptiv agresiv + sufixul "1" la PID-uri
      // ("010C1" = asteapta 1 singur raspuns -> ELM nu mai asteapta timeout dupa el).
      // Unele clone nu stiu sufixul, asa ca il testam inainte sa-l folosim.
      elmSendCommand("ATAT2", 2000);
      String t = elmSendCommand("010C1", 4000);
      pidSuffix = (t.indexOf("41 0C") >= 0) ? "1" : "";
      Serial.printf("[ELM] Sufix rapid PID: %s\n", pidSuffix.length() ? "DA" : "NU (adaptorul nu-l suporta)");
      return true;
    }
    lastEcuMsg = "ECU: " + (r.length() ? r : String("fara raspuns"));
  }
  return false;
}

static bool elmInit() {
  elmSendCommand("ATZ", 5000);   // reset
  delay(500);
  elmSendCommand("ATE0", 2000);  // echo OFF
  elmSendCommand("ATL0", 2000);  // linefeed OFF
  elmSendCommand("ATS1", 2000);  // spatii intre octeti (parserul de mai jos se bazeaza pe ele)
  elmSendCommand("ATH0", 2000);  // headere OFF
  return elmFindEcu();
}

static bool queryPidBytes(const String &pid, uint8_t *out, int expectedBytes) {
  String r = elmSendCommand(pid + pidSuffix, 4000);
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
  String r = elmSendCommand("ATRV", 3000);
  int idx = r.indexOf('V');
  if (idx < 1) return false;
  v = r.substring(0, idx).toFloat();
  return v > 0.0f;
}

/******************** CONEXIUNE BLUETOOTH ********************/
static bool connectELM() {
  SerialBT.setPin(ELM_PIN, strlen(ELM_PIN));   // ESP32 core 3.x cere si lungimea PIN-ului
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
  // Diagnostic: daca placa se reseteaza in bucla, aici vezi de ce
  // (1=pornire normala, 4=crash/panic, 9=BROWNOUT -> alimentare insuficienta)
  Serial.printf("[SYS] Motiv reset: %d | RAM libera: %u bytes\n",
                (int)esp_reset_reason(), (unsigned)ESP.getFreeHeap());

  // Bluetooth-ul porneste PRIMUL, cat timp e inca destula RAM interna libera
  // (Bluedroid are nevoie de ~100KB+; daca bufferele LVGL sunt alocate inainte, poate crapa)
  if (!SerialBT.begin("ESP32_Skoda_Test", true)) {   // true = master, initiaza el conexiunea
    Serial.println("[BT] SerialBT.begin() a ESUAT - memorie insuficienta?");
  }
  Serial.printf("[SYS] RAM libera dupa pornire BT: %u bytes\n", (unsigned)ESP.getFreeHeap());

  lv_init();
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_GREEN);

  // Buffere mai mici (20 linii in loc de 80) ca sa ramana RAM pentru Bluetooth.
  // 2 x 12.8KB in loc de 2 x 51KB - pentru un ecran static ca asta nu se vede diferenta.
  const uint32_t BUF_PIXELS = TFT_W * 20;
  buf1 = (lv_color_t *)heap_caps_malloc(BUF_PIXELS * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  buf2 = (lv_color_t *)heap_caps_malloc(BUF_PIXELS * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!buf1 || !buf2) Serial.println("[SYS] EROARE: nu am putut aloca bufferele de afisare!");
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, BUF_PIXELS);

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

  lastReconnectAttempt = millis() - RECONNECT_INTERVAL_MS;   // conectare imediat in loop()
}

/******************** LOOP ********************/
uint32_t lastEcuRetry = 0;
const uint32_t ECU_RETRY_MS = 15000;
int ecuFailCycles = 0;

// Status scurt, ca sa incapa pe un rand de ecran (320px)
static void set_status_short(const String &s) {
  set_status(s.length() > 40 ? s.substring(0, 40) : s);
  lv_timer_handler();
}

static void updateBattery() {
  float volt;
  if (readBatteryVoltage(volt)) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1fV", volt);
    lv_label_set_text(cell_batt.val, buf);
  } else {
    set_cell_err(cell_batt);
  }
}

// Citeste un PID si actualizeaza ecranul imediat (fiecare PID poate dura ~0.1-4s)
static bool readCell(bool (*fn)(int &), Cell &c) {
  int v;
  bool ok = fn(v);
  if (ok) set_cell(c, v); else set_cell_err(c);
  lv_timer_handler();
  return ok;
}

void loop() {
  // 1) Bluetooth cazut -> reconectare
  if (!SerialBT.connected()) {
    elmLinkUp = false;
    elmEcuOk = false;
    if (millis() - lastReconnectAttempt > RECONNECT_INTERVAL_MS) {
      lastReconnectAttempt = millis();
      set_status_short("Conectare la ELM327...");
      elmLinkUp = connectELM();
      if (!elmLinkUp) {
        set_status_short("Nu gasesc ELM327 - reincerc...");
      } else {
        set_status_short("Conectat, caut ECU (poate dura 1 min)...");
        elmEcuOk = elmInit();
        set_status_short(lastEcuMsg);
        lastEcuRetry = millis();
      }
    }
    lv_timer_handler();
    delay(5);
    return;
  }

  // 2) Bluetooth OK dar ECU nu raspunde -> doar tensiune + reincercare periodica
  if (!elmEcuOk) {
    updateBattery();
    if (millis() - lastEcuRetry > ECU_RETRY_MS) {
      lastEcuRetry = millis();
      set_status_short("Reincerc ECU... (contact pe ON?)");
      elmEcuOk = elmFindEcu();
      ecuFailCycles = 0;
    }
    set_status_short(lastEcuMsg);
    delay(500);
    return;
  }

  // 3) ECU OK -> date live
  // Valorile care se schimba rapid se citesc la FIECARE ciclu; temperaturile si
  // tensiunea se schimba lent, deci doar la fiecare al 5-lea ciclu -> refresh mai des.
  static uint8_t cycle = 0;
  uint32_t cycleStart = millis();
  int okCount = 0, total = 5;
  okCount += readCell(readRPM,      cell_rpm);
  okCount += readCell(readSpeed,    cell_speed);
  okCount += readCell(readThrottle, cell_throttle);
  okCount += readCell(readMAP,      cell_map);
  okCount += readCell(readLoad,     cell_load);
  if (cycle++ % 5 == 0) {
    okCount += readCell(readCoolant, cell_coolant);
    okCount += readCell(readIntake,  cell_intake);
    total += 2;
    updateBattery();
  }
  Serial.printf("[LOOP] ciclu %u ms, %d/%d PID OK\n", (unsigned)(millis() - cycleStart), okCount, total);

  if (okCount == 0) {
    // 3 cicluri la rand fara niciun raspuns -> ECU pierdut (ex. contact OFF)
    if (++ecuFailCycles >= 3) {
      elmEcuOk = false;
      lastEcuMsg = "ECU nu mai raspunde";
      lastEcuRetry = millis();
    }
  } else {
    ecuFailCycles = 0;
  }
  set_status_short(elmEcuOk ? "OK - live (" + String(okCount) + "/" + String(total) + " PID)" : lastEcuMsg);
  delay(5);
}
