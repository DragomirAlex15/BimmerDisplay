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
 *   - BluetoothSerial, WiFi, WebServer, DNSServer, Preferences, LittleFS
 *     - vin toate incluse in ESP32 core, nu trebuie instalate separat
 *
 * Arduino IDE: Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)"
 *   (partitia SPIFFS de 1MB e folosita pentru imaginea de start)
 *
 * MOD SETARI (culori + imagine de start, din telefon):
 *   - La pornire apare imaginea de start ~3 secunde. Daca in timpul ei tii
 *     apasat butonul BOOT ~1 secunda, displayul intra in modul setari.
 *     (NU tine BOOT apasat in momentul alimentarii - atunci placa intra in
 *     modul de programare. Apasa-l DUPA ce apare imaginea pe ecran.)
 *   - Conecteaza telefonul la WiFi "Skoda_Display" (parola "12345678"),
 *     pagina de setari se deschide singura (sau intra pe http://192.168.4.1).
 *   - In modul setari Bluetooth-ul e OPRIT (nu se citeste masina).
 *   - Iesire: butonul "Salveaza" din pagina, apasare pe BOOT (fara salvare),
 *     sau automat dupa 10 minute fara niciun telefon conectat.
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
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <LittleFS.h>   // trebuie inclus INAINTE de LovyanGFX ca sa mearga drawJpgFile(LittleFS, ...)

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth Classic nu e activat pe acest target. Foloseste o placa ESP32 clasica (WROOM/WROVER), nu S3/C3/C6.
#endif

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// Tipurile de mai jos sunt definite AICI, inainte de orice functie: Arduino IDE
// genereaza automat prototipuri pentru functii si le pune inaintea primei functii
// din fisier. Daca un struct e definit mai jos, prototipurile care il folosesc nu compileaza.

/******************** PARAMETRI CARE POT FI AFISATI ********************/
enum : uint8_t { F_INT, F_1DEC, F_2DEC, F_MMSS };   // cum se formateaza valoarea
enum : uint8_t {                                      // formula de calcul din octetii A,B
  X_A, X_A_M40, X_PCT, X_AB_4, X_AB, X_TIMING, X_TRIM, X_O2, X_AB_1000, X_ABS_LOAD, X_LAMBDA, X_DTC
};

struct ParamDef {
  const char *name;   // titlul de pe ecran
  const char *unit;
  uint8_t pid;        // PID OBD Mode 01; PID_SPECIAL = nu e PID (ex. ATRV)
  uint8_t bytes;      // cati octeti de date intoarce PID-ul
  uint8_t formula;
  uint8_t fmt;
  bool slow;          // valoare care se schimba lent -> citita doar la fiecare al 5-lea ciclu
  const char *desc;   // descrierea din pagina de setari
};

#define PID_SPECIAL 0xFF
#define P_NONE      0
#define P_BATTERY   8
#define P_BARO      14
#define P_VACUUM    15

// ATENTIE: ordinea conteaza - indexul e salvat in setari. Parametri noi se adauga DOAR la final.
const ParamDef PARAMS[] = {
  /* 0*/ { "",         "",     0,           0, X_A,        F_INT,  false, "- gol -" },
  /* 1*/ { "RPM",      "",     0x0C,        2, X_AB_4,     F_INT,  false, "Turatie motor" },
  /* 2*/ { "SPEED",    "km/h", 0x0D,        1, X_A,        F_INT,  false, "Viteza" },
  /* 3*/ { "COOLANT",  "C",    0x05,        1, X_A_M40,    F_INT,  true,  "Temperatura lichid racire" },
  /* 4*/ { "INTAKE",   "C",    0x0F,        1, X_A_M40,    F_INT,  true,  "Temperatura aer admisie" },
  /* 5*/ { "LOAD",     "%",    0x04,        1, X_PCT,      F_INT,  false, "Sarcina motor calculata" },
  /* 6*/ { "MAP",      "kPa",  0x0B,        1, X_A,        F_INT,  false, "Presiune galerie admisie" },
  /* 7*/ { "THROTTLE", "%",    0x11,        1, X_PCT,      F_INT,  false, "Pozitie clapeta" },
  /* 8*/ { "BATTERY",  "V",    PID_SPECIAL, 0, X_A,        F_1DEC, true,  "Tensiune baterie (citita de adaptor)" },
  /* 9*/ { "TIMING",   "grd",  0x0E,        1, X_TIMING,   F_1DEC, false, "Avans aprindere" },
  /*10*/ { "STFT",     "%",    0x06,        1, X_TRIM,     F_1DEC, false, "Corectie combustibil pe termen scurt" },
  /*11*/ { "LTFT",     "%",    0x07,        1, X_TRIM,     F_1DEC, true,  "Corectie combustibil pe termen lung" },
  /*12*/ { "O2 S1",    "V",    0x14,        1, X_O2,       F_2DEC, false, "Sonda lambda 1 (inainte de catalizator)" },
  /*13*/ { "O2 S2",    "V",    0x15,        1, X_O2,       F_2DEC, false, "Sonda lambda 2 (dupa catalizator)" },
  /*14*/ { "BARO",     "kPa",  0x33,        1, X_A,        F_INT,  true,  "Presiune atmosferica" },
  /*15*/ { "VACUUM",   "kPa",  0x0B,        1, X_A,        F_INT,  false, "Depresiune galerie (MAP minus atmosferica)" },
  /*16*/ { "RUN TIME", "",     0x1F,        2, X_AB,       F_MMSS, true,  "Timp de la pornirea motorului" },
  /*17*/ { "ECU VOLT", "V",    0x42,        2, X_AB_1000,  F_1DEC, true,  "Tensiune vazuta de calculatorul motor" },
  /*18*/ { "PEDAL",    "%",    0x49,        1, X_PCT,      F_INT,  false, "Pozitie pedala acceleratie" },
  /*19*/ { "ABS LOAD", "%",    0x43,        2, X_ABS_LOAD, F_INT,  false, "Sarcina absoluta" },
  /*20*/ { "LAMBDA",   "",     0x44,        2, X_LAMBDA,   F_2DEC, false, "Lambda comandata" },
  /*21*/ { "ERORI",    "",     0x01,        1, X_DTC,      F_INT,  true,  "Numar coduri de eroare motor" },
  /*22*/ { "KM ERORI", "km",   0x31,        2, X_AB,       F_INT,  true,  "Km de la stergerea erorilor" },
};
const uint8_t PARAM_COUNT = sizeof(PARAMS) / sizeof(PARAMS[0]);

#define MAX_SLOTS 8
#define SLOT_HAS_MIN 0x01
#define SLOT_HAS_MAX 0x02

// O casuta de pe ecran, asa cum e salvata in setari
struct SlotCfg {
  uint8_t param;   // index in PARAMS
  uint8_t flags;   // SLOT_HAS_MIN / SLOT_HAS_MAX
  float vmin;      // sub minim -> culoare de avertizare
  float vmax;      // peste maxim -> culoare de avertizare
};

// O casuta afisata (obiectele LVGL). Bold-ul e simulat cu o a doua eticheta
// decalata cu 1px (fonturile LVGL incluse nu au varianta bold).
struct Cell {
  lv_obj_t *title;
  lv_obj_t *titleB;   // copia pentru bold (nullptr daca nu e bold)
  lv_obj_t *val;
  lv_obj_t *valB;
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

/******************** CONFIG MOD SETARI ********************/
#define BOOT_PIN        0                 // butonul BOOT de pe placa
#define AP_SSID         "Skoda_Display"
#define AP_PASS         "12345678"        // minim 8 caractere
#define SPLASH_PATH     "/splash.jpg"
const uint32_t SPLASH_MS          = 3000;            // cat sta imaginea de start
const uint32_t BOOT_HOLD_MS       = 1000;            // cat tii BOOT ca sa intri in setari
const uint32_t CONFIG_TIMEOUT_MS  = 10UL * 60 * 1000; // iesire automata fara telefon conectat
const size_t   MAX_IMAGE_BYTES    = 300 * 1024;

// Nota: pe aceasta placa backlight-ul e legat fix la alimentare (testat cu
// Projects/Backlight_Test - niciun pin liber nu il controleaza), deci
// luminozitatea NU se poate regla din software.

// Setari salvate permanent (NVS) - valorile implicite sunt cele de pana acum
uint32_t cfgBg = 0x00FF00;   // fundal verde
uint32_t cfgFg = 0x000000;   // scris negru
uint32_t cfgWarn = 0xFF0000; // culoare de avertizare (valoare in afara limitelor)
SlotCfg cfgSlots[MAX_SLOTS];
uint8_t cfgValSize = 0;        // marime cifre in px (10-48), 0 = automat dupa numarul de casute
uint8_t cfgTitleSize = 0;      // marime titluri in px (10-48), 0 = automat
bool cfgValBold = false;
bool cfgTitleBold = false;

// PID-uri suportate de ECU (raspunsurile la 0100 / 0120 / 0140), salvate la prima conectare
uint32_t supMask[3] = { 0, 0, 0 };
bool supKnown = false;

bool fsOk = false;          // LittleFS montat (necesar pentru imaginea de start)
bool configMode = false;

Preferences prefs;
WebServer *server = nullptr;  // creat doar in modul setari, ca sa nu ocupe RAM in mers
DNSServer dnsServer;
File uploadFile;
bool uploadOk = false;
size_t uploadSize = 0;

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

// 0xRRGGBB -> culoare RGB565 pentru desenare directa cu LovyanGFX
static uint16_t rgb565(uint32_t c) {
  return tft.color565((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF);
}

/******************** SETARI (NVS) ********************/
// Casutele implicite = cele 8 valori de pana acum, cu cateva limite utile
static void defaultSlots() {
  const uint8_t def[MAX_SLOTS] = { 1, 3, 2, 5, 4, 6, 7, P_BATTERY };
  for (int i = 0; i < MAX_SLOTS; i++) cfgSlots[i] = { def[i], 0, 0.0f, 0.0f };
  cfgSlots[1].flags = SLOT_HAS_MAX;                cfgSlots[1].vmax = 105.0f;   // COOLANT > 105 C
  cfgSlots[7].flags = SLOT_HAS_MIN | SLOT_HAS_MAX; cfgSlots[7].vmin = 11.8f;    // BATTERY < 11.8 V
                                                   cfgSlots[7].vmax = 15.0f;    //         > 15 V
}

static void loadSettings() {
  prefs.begin("display", true);
  cfgBg = prefs.getUInt("bg", cfgBg);
  cfgFg = prefs.getUInt("fg", cfgFg);
  cfgWarn = prefs.getUInt("wc", cfgWarn);
  cfgValSize = prefs.getUChar("vs", cfgValSize);
  cfgTitleSize = prefs.getUChar("ts", cfgTitleSize);
  cfgValBold = prefs.getBool("vb", cfgValBold);
  cfgTitleBold = prefs.getBool("tb", cfgTitleBold);

  defaultSlots();
  if (prefs.getBytesLength("slots") == sizeof(cfgSlots)) {
    prefs.getBytes("slots", cfgSlots, sizeof(cfgSlots));
    for (auto &s : cfgSlots) if (s.param >= PARAM_COUNT) s.param = P_NONE;
  }

  supKnown = prefs.getBool("supok", false);
  if (supKnown && prefs.getBytesLength("sup") == sizeof(supMask)) prefs.getBytes("sup", supMask, sizeof(supMask));
  else supKnown = false;
  prefs.end();

  Serial.printf("[CFG] fundal #%06X, scris #%06X, avertizare #%06X\n",
                (unsigned)cfgBg, (unsigned)cfgFg, (unsigned)cfgWarn);
  Serial.print("[CFG] casute:");
  for (auto &s : cfgSlots) Serial.printf(" %s", s.param ? PARAMS[s.param].name : "-");
  Serial.println();
}

static void saveSettings() {
  prefs.begin("display", false);
  prefs.putUInt("bg", cfgBg);
  prefs.putUInt("fg", cfgFg);
  prefs.putUInt("wc", cfgWarn);
  prefs.putUChar("vs", cfgValSize);
  prefs.putUChar("ts", cfgTitleSize);
  prefs.putBool("vb", cfgValBold);
  prefs.putBool("tb", cfgTitleBold);
  prefs.putBytes("slots", cfgSlots, sizeof(cfgSlots));
  prefs.end();
}

// Scrie in NVS doar daca lista s-a schimbat (protejeaza memoria flash)
static void saveSupported(const uint32_t m[3]) {
  if (supKnown && memcmp(m, supMask, sizeof(supMask)) == 0) return;
  memcpy(supMask, m, sizeof(supMask));
  supKnown = true;
  prefs.begin("display", false);
  prefs.putBytes("sup", supMask, sizeof(supMask));
  prefs.putBool("supok", true);
  prefs.end();
  Serial.printf("[ELM] PID-uri suportate salvate: %08X %08X %08X\n",
                (unsigned)supMask[0], (unsigned)supMask[1], (unsigned)supMask[2]);
}

// true daca ECU-ul suporta PID-ul (sau daca inca nu stim - atunci incercam oricum)
static bool pidSupported(uint8_t pid) {
  if (pid == PID_SPECIAL || !supKnown) return true;
  if (pid < 1 || pid > 0x60) return false;
  uint8_t i = (pid - 1) / 32;
  uint8_t bit = 31 - ((pid - 1) % 32);
  return (supMask[i] >> bit) & 1;
}

static bool paramSupported(uint8_t id) {
  if (id == P_NONE) return false;
  if (id == P_VACUUM) return pidSupported(0x0B) && pidSupported(0x33);
  return pidSupported(PARAMS[id].pid);
}

/******************** UI ********************/
lv_obj_t *lbl_status;
lv_style_t style_status;

Cell cells[MAX_SLOTS];
uint8_t activeSlot[MAX_SLOTS];   // pentru fiecare casuta afisata: indexul ei in cfgSlots
uint8_t activeCount = 0;

// Fonturile Montserrat activate in lv_conf.h (10..48, din 2 in 2)
static const lv_font_t *fontBySize(uint8_t px) {
  switch (px) {
    case 10: return &lv_font_montserrat_10;
    case 12: return &lv_font_montserrat_12;
    case 14: return &lv_font_montserrat_14;
    case 16: return &lv_font_montserrat_16;
    case 18: return &lv_font_montserrat_18;
    case 20: return &lv_font_montserrat_20;
    case 22: return &lv_font_montserrat_22;
    case 24: return &lv_font_montserrat_24;
    case 26: return &lv_font_montserrat_26;
    case 28: return &lv_font_montserrat_28;
    case 30: return &lv_font_montserrat_30;
    case 32: return &lv_font_montserrat_32;
    case 34: return &lv_font_montserrat_34;
    case 36: return &lv_font_montserrat_36;
    case 38: return &lv_font_montserrat_38;
    case 40: return &lv_font_montserrat_40;
    case 42: return &lv_font_montserrat_42;
    case 44: return &lv_font_montserrat_44;
    case 46: return &lv_font_montserrat_46;
    case 48: return &lv_font_montserrat_48;
    default: return &lv_font_montserrat_14;
  }
}

// Marimi automate: cea mai mare care incape in latimea casutei
static uint8_t autoValSize(uint8_t cols)   { return cols <= 1 ? 48 : cols == 2 ? 44 : cols == 3 ? 32 : 24; }
static uint8_t autoTitleSize(uint8_t cols) { return cols <= 2 ? 16 : cols == 3 ? 14 : 12; }

static lv_obj_t *mkLabel(lv_obj_t *parent, const char *txt, const lv_font_t *f, int x, int y) {
  lv_obj_t *l = lv_label_create(parent);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, f, 0);
  lv_obj_set_style_text_color(l, lv_color_hex(cfgFg), 0);
  lv_obj_set_pos(l, x, y);
  return l;
}

static void create_dashboard() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(cfgBg), 0);   // culoare fundal din setari
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  lv_style_init(&style_status);
  lv_style_set_text_color(&style_status, lv_color_hex(cfgFg));
  lv_style_set_text_font(&style_status, &lv_font_montserrat_14);
  lv_style_set_bg_opa(&style_status, LV_OPA_TRANSP);
  lv_style_set_border_width(&style_status, 0);

  lbl_status = lv_label_create(scr);
  lv_label_set_text(lbl_status, "Pornire...");
  lv_obj_add_style(lbl_status, &style_status, 0);
  lv_obj_set_pos(lbl_status, 5, 4);

  // casutele alese in setari, in ordine, fara cele goale
  activeCount = 0;
  for (uint8_t i = 0; i < MAX_SLOTS; i++)
    if (cfgSlots[i].param != P_NONE) activeSlot[activeCount++] = i;

  if (activeCount == 0) {
    mkLabel(scr, "Niciun parametru ales.\nIntra in setari (BOOT la pornire).", &lv_font_montserrat_16, 10, 100);
    return;
  }

  // aranjament automat dupa numarul de casute
  uint8_t cols, rows;
  if (activeCount == 1)      { cols = 1; rows = 1; }
  else if (activeCount == 2) { cols = 2; rows = 1; }
  else if (activeCount <= 4) { cols = 2; rows = 2; }
  else if (activeCount <= 6) { cols = 3; rows = 2; }
  else                       { cols = 4; rows = 2; }

  const int top = 24;   // sub randul de status
  const int cw = TFT_W / cols;
  const int ch = (TFT_H - top) / rows;
  const lv_font_t *tf = fontBySize(cfgTitleSize ? cfgTitleSize : autoTitleSize(cols));
  const lv_font_t *vf = fontBySize(cfgValSize ? cfgValSize : autoValSize(cols));
  const int th = lv_font_get_line_height(tf);
  const int vh = lv_font_get_line_height(vf);

  for (uint8_t ci = 0; ci < activeCount; ci++) {
    uint8_t id = cfgSlots[activeSlot[ci]].param;
    const ParamDef &p = PARAMS[id];
    int x = (ci % cols) * cw + 6;
    int y = top + (ci / cols) * ch + 4;

    char title[24];
    if (p.unit[0]) snprintf(title, sizeof(title), "%s %s", p.name, p.unit);
    else snprintf(title, sizeof(title), "%s", p.name);

    // valoarea centrata vertical in spatiul ramas sub titlu
    int vy = y + th + max(0, (ch - 8 - th - vh) / 2);
    const char *init = paramSupported(id) ? "--" : "n/a";

    Cell &c = cells[ci];
    c.titleB = cfgTitleBold ? mkLabel(scr, title, tf, x + 1, y) : nullptr;
    c.title  = mkLabel(scr, title, tf, x, y);
    c.valB   = cfgValBold ? mkLabel(scr, init, vf, x + 1, vy) : nullptr;
    c.val    = mkLabel(scr, init, vf, x, vy);
  }
}

static void setCellText(Cell &c, const char *txt) {
  lv_label_set_text(c.val, txt);
  if (c.valB) lv_label_set_text(c.valB, txt);
}

static void setCellColor(Cell &c, uint32_t rgb) {
  lv_color_t k = lv_color_hex(rgb);
  lv_obj_set_style_text_color(c.val, k, 0);
  lv_obj_set_style_text_color(c.title, k, 0);
  if (c.valB) lv_obj_set_style_text_color(c.valB, k, 0);
  if (c.titleB) lv_obj_set_style_text_color(c.titleB, k, 0);
}

static void formatValue(const ParamDef &p, float v, char *buf, size_t n) {
  switch (p.fmt) {
    case F_1DEC: snprintf(buf, n, "%.1f", v); break;
    case F_2DEC: snprintf(buf, n, "%.2f", v); break;
    case F_MMSS: {
      int s = (int)v;
      if (s >= 3600) snprintf(buf, n, "%dh%02d", s / 3600, (s / 60) % 60);
      else snprintf(buf, n, "%d:%02d", s / 60, s % 60);
      break;
    }
    default: snprintf(buf, n, "%d", (int)lroundf(v)); break;
  }
}

// Afiseaza valoarea in casuta ci; culoarea de avertizare daca iese din limite
static void showValue(uint8_t ci, bool ok, float v) {
  Cell &c = cells[ci];
  const SlotCfg &s = cfgSlots[activeSlot[ci]];
  if (!ok) {
    setCellText(c, "--");
    setCellColor(c, cfgFg);
    return;
  }
  char buf[16];
  formatValue(PARAMS[s.param], v, buf, sizeof(buf));
  setCellText(c, buf);
  bool warn = ((s.flags & SLOT_HAS_MIN) && v < s.vmin) || ((s.flags & SLOT_HAS_MAX) && v > s.vmax);
  setCellColor(c, warn ? cfgWarn : cfgFg);
}

static void set_status(const String &s) {
  lv_label_set_text(lbl_status, s.c_str());
  Serial.println(s);
}

/******************** IMAGINE DE START ********************/
// Deseneaza imaginea de start (sau un ecran simplu daca nu exista imagine) si
// urmareste butonul BOOT. Returneaza true daca utilizatorul vrea modul setari.
static bool showSplashAndCheckBoot() {
  tft.fillScreen(rgb565(cfgBg));

  bool drawn = false;
  if (fsOk && LittleFS.exists(SPLASH_PATH)) {
    drawn = tft.drawJpgFile(LittleFS, SPLASH_PATH, 0, 0);
    if (!drawn) Serial.println("[SPLASH] imaginea nu a putut fi desenata");
  }
  if (!drawn) {
    tft.setTextColor(rgb565(cfgFg));
    tft.setTextDatum(middle_center);
    tft.setFont(&fonts::FreeSansBold18pt7b);
    tft.drawString("Skoda Fabia", TFT_W / 2, TFT_H / 2 - 12);
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.drawString("1.2 12V BZG", TFT_W / 2, TFT_H / 2 + 22);
  }

  // indiciu jos, cu fundal ca sa se citeasca si peste poza
  tft.setFont(&fonts::Font2);
  tft.setTextDatum(bottom_center);
  tft.setTextColor(rgb565(cfgFg), rgb565(cfgBg));
  tft.drawString(" Tine apasat BOOT pentru setari ", TFT_W / 2, TFT_H - 2);

  uint32_t start = millis();
  uint32_t pressStart = 0;
  // daca butonul e apasat cand expira timpul, mai asteptam pana se implineste 1s
  while (millis() - start < SPLASH_MS || pressStart) {
    if (digitalRead(BOOT_PIN) == LOW) {
      if (!pressStart) pressStart = millis();
      else if (millis() - pressStart >= BOOT_HOLD_MS) return true;
    } else {
      pressStart = 0;
    }
    delay(10);
  }
  return false;
}

/******************** MOD SETARI - ECRAN ********************/
static void drawConfigScreen(const char *footer) {
  tft.fillScreen(tft.color565(20, 20, 20));
  tft.setTextDatum(top_left);
  tft.setTextColor(TFT_WHITE);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.drawString("MOD SETARI", 10, 8);

  tft.setFont(&fonts::FreeSans9pt7b);
  tft.drawString("1. Conecteaza telefonul la WiFi:", 10, 48);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("   " AP_SSID "   parola: " AP_PASS, 10, 70);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("2. Pagina se deschide singura, sau:", 10, 102);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("   http://192.168.4.1", 10, 124);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("BOOT = iesire fara salvare", 10, 164);

  tft.setFont(&fonts::Font2);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(footer, 10, TFT_H - 22);
}

// Previzualizare culori direct pe display, cat timp le alegi din telefon
static void drawColorPreview(uint32_t bg, uint32_t fg, uint32_t wc) {
  tft.fillScreen(rgb565(bg));
  tft.setTextDatum(top_left);
  tft.setTextColor(rgb565(fg));
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.drawString("Previzualizare culori (nesalvat)", 5, 4);
  tft.drawString("RPM", 5, 40);
  tft.setFont(&fonts::FreeSansBold18pt7b);
  tft.drawString("2500", 5, 62);
  tft.setTextColor(rgb565(wc));
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.drawString("COOLANT C", 165, 40);
  tft.setFont(&fonts::FreeSansBold18pt7b);
  tft.drawString("112", 165, 62);
  tft.setTextColor(rgb565(fg));
  tft.setFont(&fonts::Font2);
  tft.drawString("Apasa Salveaza in pagina ca sa pastrezi setarile", 5, TFT_H - 22);
}

/******************** MOD SETARI - PAGINA WEB ********************/
static const char PAGE_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="ro"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Skoda Display</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;margin:0;background:#111;color:#eee}
main{max-width:480px;margin:0 auto;padding:16px}
h1{font-size:22px;margin:4px 0 16px}
section{background:#1d1d1d;border-radius:12px;padding:14px;margin-bottom:14px}
h2{font-size:16px;margin:0 0 12px;color:#9cf}
.row{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.row label{flex:1}
input[type=color]{width:56px;height:40px;border:none;padding:0;background:none}
input[type=text]{width:88px;padding:8px;border-radius:8px;border:1px solid #444;background:#000;color:#eee;font-family:monospace}
#prev{position:relative;aspect-ratio:4/3;border-radius:8px;overflow:hidden;font-family:Montserrat,Arial,sans-serif}
#prev div{position:absolute;white-space:nowrap;line-height:1.15}
#wex{margin-top:8px;font-weight:bold}
select{flex:1;min-width:0;padding:8px;border-radius:8px;border:1px solid #444;background:#000;color:#eee;font-size:15px}
input[type=checkbox]{width:24px;height:24px}
input[type=number]{flex:1;min-width:0;padding:8px;border-radius:8px;border:1px solid #444;background:#000;color:#eee;font-size:15px}
.slot{border-top:1px solid #333;padding:10px 0}.slot label{display:block;margin-bottom:6px;color:#bbb}
.slot select{width:100%}.lim{display:flex;gap:8px;margin-top:6px}
canvas,img{width:100%;border-radius:8px;display:none;margin-top:10px}
button{width:100%;padding:13px;border:none;border-radius:10px;font-size:16px;margin-top:10px;background:#2a6;color:#fff}
button.sec{background:#444}button.del{background:#a33}button:disabled{opacity:.4}
#msg{min-height:20px;text-align:center;margin-top:10px;color:#fd6}
small{color:#999}
</style></head><body><main>
<h1>Skoda Display &ndash; Setări</h1>

<section><h2>Culori</h2>
<div class="row"><label for="bg">Fundal</label><input type="color" id="bg"><input type="text" id="bgh" maxlength="7"></div>
<div class="row"><label for="fg">Scris</label><input type="color" id="fg"><input type="text" id="fgh" maxlength="7"></div>
<div class="row"><label for="wc">Avertizare (în afara limitelor)</label><input type="color" id="wc"><input type="text" id="wch" maxlength="7"></div>
<small>Culorile apar imediat și pe display, dar se păstrează doar după „Salvează”.</small>
</section>

<section><h2>Previzualizare ecran</h2>
<div id="prev"></div>
<div id="wex">Exemplu valoare în afara limitelor: 112</div>
<small>Aproximativ &ndash; pe display fontul arată puțin diferit.</small>
</section>

<section><h2>Mărime și stil text</h2>
<div class="row"><label for="vs">Mărime valori</label><select id="vs"></select></div>
<div class="row"><label for="ts">Mărime titluri</label><select id="ts"></select></div>
<div class="row"><label for="vb">Valori bold</label><input type="checkbox" id="vb"></div>
<div class="row"><label for="tb">Titluri bold</label><input type="checkbox" id="tb"></div>
<small>„Automat” alege cea mai mare mărime care încape în căsuță. Dacă alegi mai mare, textul poate ieși din căsuță &ndash; verifică în previzualizare.</small>
</section>

<section><h2>Parametri afișați</h2>
<div id="slots"></div>
<small>Ecranul se aranjează singur după câte căsuțe folosești (1&ndash;8). Cu mai puține căsuțe, valorile se actualizează mai des. Min / max sunt opționale: când valoarea iese din interval, căsuța devine în culoarea de avertizare.</small><br>
<small id="supnote"></small>
</section>
<section><h2>Imagine de start</h2>
<img id="cur" alt="imaginea curentă">
<input type="file" id="file" accept="image/*">
<canvas id="cv" width="320" height="240"></canvas>
<button id="up" class="sec" disabled>Trimite imaginea</button>
<button id="del" class="del" style="display:none">Șterge imaginea de start</button>
<small>Poza e decupată automat la 320×240. Dacă alegerea pozei nu merge din fereastra care s-a deschis singură, deschide http://192.168.4.1 în Safari / Chrome.</small>
</section>

<button id="save">Salvează și repornește</button>
<div id="msg"></div>
</main><script>
const $=id=>document.getElementById(id);
const msg=t=>$('msg').textContent=t;
let timer,blob=null,P=[];
const SAMPLE=['88','12.5','0.45','12:34'];   // exemplu de valoare dupa format (int, 1 zecimala, 2 zecimale, mm:ss)
function fillSize(id){const s=$(id);s.add(new Option('Automat',0));for(let n=10;n<=48;n+=2)s.add(new Option(n+' px',n));}
fillSize('vs');fillSize('ts');
function body(){const b=new URLSearchParams({bg:$('bg').value,fg:$('fg').value,wc:$('wc').value,
 vs:$('vs').value,ts:$('ts').value,vb:$('vb').checked?1:0,tb:$('tb').checked?1:0});
 for(let i=0;i<8;i++){if(!$('s'+i))break;b.append('s'+i,$('s'+i).value);b.append('n'+i,$('n'+i).value);b.append('x'+i,$('x'+i).value);}
 return b;}
function sendPreview(){clearTimeout(timer);timer=setTimeout(()=>fetch('/preview',{method:'POST',body:body()}),300);}
function render(){const p=$('prev'),fg=$('fg').value;p.style.background=$('bg').value;p.innerHTML='';
 $('wex').style.color=$('wc').value;
 const k=p.clientWidth/320;
 function add(t,x,y,s,b,col){const d=document.createElement('div');d.textContent=t;d.style.left=x*k+'px';d.style.top=y*k+'px';
  d.style.fontSize=s*k+'px';d.style.fontWeight=b?'bold':'normal';d.style.color=col;p.appendChild(d);}
 const act=[];for(let i=0;i<8;i++){const e=$('s'+i);if(e&&+e.value)act.push(+e.value);}
 add('OK - live',5,4,14,false,fg);
 const n=act.length;if(!n){add('Niciun parametru ales',10,100,16,false,fg);return;}
 let c,r;if(n==1){c=1;r=1}else if(n==2){c=2;r=1}else if(n<=4){c=2;r=2}else if(n<=6){c=3;r=2}else{c=4;r=2}
 const vs=+$('vs').value||(c<=1?48:c==2?44:c==3?32:24),ts=+$('ts').value||(c<=2?16:c==3?14:12);
 const cw=320/c,chh=(240-24)/r,th=ts*1.2,vh=vs*1.2;
 act.forEach((id,i)=>{const q=P[id],x=(i%c)*cw+6,y=24+Math.floor(i/c)*chh+4,vy=y+th+Math.max(0,(chh-8-th-vh)/2);
  add(q.n+(q.u?' '+q.u:''),x,y,ts,$('tb').checked,fg);add(q.s?SAMPLE[q.f]:'n/a',x,vy,vs,$('vb').checked,fg);});}
function limVis(i){$('n'+i).parentNode.style.display=$('s'+i).value=='0'?'none':'flex';}
function buildSlots(slots){const box=$('slots');box.innerHTML='';
 for(let i=0;i<8;i++){const d=document.createElement('div');d.className='slot';
  d.innerHTML='<label for="s'+i+'">Căsuța '+(i+1)+'</label><select id="s'+i+'"></select>'+
   '<div class="lim"><input type="number" step="any" id="n'+i+'" placeholder="min"><input type="number" step="any" id="x'+i+'" placeholder="max"></div>';
  box.appendChild(d);
  const s=$('s'+i);
  P.forEach((q,k)=>{let t=k?q.n+(q.u?' ('+q.u+')':'')+' – '+q.d:'— gol —';if(k&&!q.s)t+=' ✕ nesuportat';s.add(new Option(t,k));});
  const c=slots[i]||{p:0};s.value=c.p;if(c.mn!=null)$('n'+i).value=c.mn;if(c.mx!=null)$('x'+i).value=c.mx;
  s.addEventListener('change',()=>{limVis(i);render();});limVis(i);}}
['bg','fg','wc'].forEach(k=>{
 $(k).addEventListener('input',()=>{$(k+'h').value=$(k).value;render();sendPreview();});
 $(k+'h').addEventListener('change',()=>{let v=$(k+'h').value.trim();if(v[0]!='#')v='#'+v;
  if(/^#[0-9a-fA-F]{6}$/.test(v)){$(k).value=v.toLowerCase();$(k+'h').value=$(k).value;render();sendPreview();}else $(k+'h').value=$(k).value;});
});
['vs','ts','vb','tb'].forEach(k=>$(k).addEventListener('change',render));
window.addEventListener('resize',render);
function showCur(){$('cur').src='/splash.jpg?'+Date.now();$('cur').style.display='block';$('del').style.display='block';}
async function load(){try{const c=await (await fetch('/config')).json();P=c.params;
 ['bg','fg','wc'].forEach(k=>{$(k).value=c[k];$(k+'h').value=c[k];});
 $('vs').value=c.vs;$('ts').value=c.ts;$('vb').checked=c.vb;$('tb').checked=c.tb;
 buildSlots(c.slots);
 $('supnote').textContent=c.sup?'„✕ nesuportat” = calculatorul mașinii a raportat că nu are acest parametru (pe ecran apare n/a).'
  :'Lista parametrilor suportați de mașină apare după prima conectare la ECU.';
 render();if(c.img)showCur();}catch(e){msg('Nu pot citi setările');}}
$('file').addEventListener('change',e=>{const f=e.target.files[0];if(!f)return;const img=new Image();
 img.onload=()=>{const c=$('cv'),x=c.getContext('2d');const s=Math.max(320/img.width,240/img.height);
  const w=img.width*s,h=img.height*s;x.fillStyle='#000';x.fillRect(0,0,320,240);
  x.drawImage(img,(320-w)/2,(240-h)/2,w,h);c.style.display='block';URL.revokeObjectURL(img.src);
  c.toBlob(b=>{blob=b;$('up').disabled=false;msg('Imagine pregătită ('+Math.round(b.size/1024)+' KB)');},'image/jpeg',0.85);};
 img.onerror=()=>msg('Formatul pozei nu e suportat');img.src=URL.createObjectURL(f);});
$('up').onclick=async()=>{if(!blob)return;const fd=new FormData();fd.append('file',blob,'splash.jpg');
 msg('Se trimite...');$('up').disabled=true;
 try{const r=await fetch('/upload',{method:'POST',body:fd});
  if(r.ok){msg('Imagine salvată ✔ (o vezi acum pe display)');showCur();$('cv').style.display='none';blob=null;}
  else{msg('Eroare: '+await r.text());$('up').disabled=false;}}catch(e){msg('Eroare la trimitere');$('up').disabled=false;}};
$('del').onclick=async()=>{if(!confirm('Ștergi imaginea de start?'))return;
 await fetch('/delimg',{method:'POST'});$('cur').style.display='none';$('del').style.display='none';msg('Imagine ștearsă');};
$('save').onclick=async()=>{try{await fetch('/save',{method:'POST',body:body()});}catch(e){}
 msg('Salvat ✔ Displayul repornește. Te poți deconecta de la WiFi.');};
load();
</script></body></html>)rawliteral";

static bool parseColor(const String &s, uint32_t &out) {
  if (s.length() != 7 || s[0] != '#') return false;
  char *end;
  uint32_t v = strtoul(s.c_str() + 1, &end, 16);
  if (*end) return false;
  out = v;
  return true;
}

static void handleRoot() {
  server->send_P(200, "text/html; charset=utf-8", PAGE_HTML);
}

static void handleGetConfig() {
  char tmp[160];
  bool img = fsOk && LittleFS.exists(SPLASH_PATH);
  String j;
  j.reserve(2600);
  snprintf(tmp, sizeof(tmp),
           "{\"bg\":\"#%06x\",\"fg\":\"#%06x\",\"wc\":\"#%06x\",\"vs\":%u,\"ts\":%u,\"vb\":%s,\"tb\":%s,\"img\":%s,\"sup\":%s,",
           (unsigned)cfgBg, (unsigned)cfgFg, (unsigned)cfgWarn, cfgValSize, cfgTitleSize,
           cfgValBold ? "true" : "false", cfgTitleBold ? "true" : "false",
           img ? "true" : "false", supKnown ? "true" : "false");
  j += tmp;

  // lista parametrilor: n=nume, u=unitate, d=descriere, f=format, s=suportat de ECU
  j += "\"params\":[";
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    const ParamDef &p = PARAMS[i];
    snprintf(tmp, sizeof(tmp), "%s{\"n\":\"%s\",\"u\":\"%s\",\"d\":\"%s\",\"f\":%u,\"s\":%s}",
             i ? "," : "", p.name, p.unit, p.desc, p.fmt, (i == P_NONE || paramSupported(i)) ? "true" : "false");
    j += tmp;
  }

  // casutele: p=parametru, mn/mx=limite (null = fara limita)
  j += "],\"slots\":[";
  for (uint8_t i = 0; i < MAX_SLOTS; i++) {
    const SlotCfg &s = cfgSlots[i];
    char mn[16] = "null", mx[16] = "null";
    if (s.flags & SLOT_HAS_MIN) snprintf(mn, sizeof(mn), "%g", s.vmin);
    if (s.flags & SLOT_HAS_MAX) snprintf(mx, sizeof(mx), "%g", s.vmax);
    snprintf(tmp, sizeof(tmp), "%s{\"p\":%u,\"mn\":%s,\"mx\":%s}", i ? "," : "", s.param, mn, mx);
    j += tmp;
  }
  j += "]}";
  server->send(200, "application/json", j);
}

static void handlePreview() {
  uint32_t bg, fg, wc = cfgWarn;
  if (!parseColor(server->arg("bg"), bg) || !parseColor(server->arg("fg"), fg)) {
    server->send(400, "text/plain", "culoare invalida");
    return;
  }
  parseColor(server->arg("wc"), wc);
  drawColorPreview(bg, fg, wc);
  server->send(200, "text/plain", "ok");
}

// marime font valida: 0 (automat) sau numar par 10..48
static uint8_t parseFontSize(const String &s) {
  int v = s.toInt();
  if (v < 10 || v > 48) return 0;
  return (uint8_t)(v & ~1);
}

static void handleSave() {
  uint32_t bg, fg, wc = cfgWarn;
  if (!parseColor(server->arg("bg"), bg) || !parseColor(server->arg("fg"), fg)) {
    server->send(400, "text/plain", "culoare invalida");
    return;
  }
  parseColor(server->arg("wc"), wc);
  cfgBg = bg;
  cfgFg = fg;
  cfgWarn = wc;
  cfgValSize = parseFontSize(server->arg("vs"));
  cfgTitleSize = parseFontSize(server->arg("ts"));
  cfgValBold = server->arg("vb") == "1";
  cfgTitleBold = server->arg("tb") == "1";

  if (server->hasArg("s0")) {
    for (uint8_t i = 0; i < MAX_SLOTS; i++) {
      SlotCfg &s = cfgSlots[i];
      int p = server->arg("s" + String(i)).toInt();
      s.param = (p > 0 && p < PARAM_COUNT) ? p : P_NONE;
      s.flags = 0;
      s.vmin = s.vmax = 0.0f;
      String mn = server->arg("n" + String(i)), mx = server->arg("x" + String(i));
      mn.trim(); mx.trim();
      if (s.param && mn.length()) { s.flags |= SLOT_HAS_MIN; s.vmin = mn.toFloat(); }
      if (s.param && mx.length()) { s.flags |= SLOT_HAS_MAX; s.vmax = mx.toFloat(); }
    }
  }
  saveSettings();
  Serial.printf("[CFG] Salvat: fundal #%06X, scris #%06X -> repornire\n", (unsigned)cfgBg, (unsigned)cfgFg);
  server->send(200, "text/plain", "ok");
  drawConfigScreen("Salvat! Repornire...");
  delay(800);   // lasam raspunsul sa ajunga la telefon
  ESP.restart();
}

static void handleGetSplash() {
  if (!fsOk || !LittleFS.exists(SPLASH_PATH)) {
    server->send(404, "text/plain", "nu exista");
    return;
  }
  File f = LittleFS.open(SPLASH_PATH, "r");
  server->sendHeader("Cache-Control", "no-cache");
  server->streamFile(f, "image/jpeg");
  f.close();
}

// Primeste imaginea bucata cu bucata si o scrie intr-un fisier temporar
static void handleUploadChunk() {
  HTTPUpload &up = server->upload();
  if (up.status == UPLOAD_FILE_START) {
    uploadOk = false;
    uploadSize = 0;
    if (fsOk) uploadFile = LittleFS.open("/splash.tmp", "w");
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) {
      uploadSize += up.currentSize;
      if (uploadSize > MAX_IMAGE_BYTES || uploadFile.write(up.buf, up.currentSize) != up.currentSize) {
        uploadFile.close();
        LittleFS.remove("/splash.tmp");
      }
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (uploadFile) {
      uploadFile.close();
      LittleFS.remove(SPLASH_PATH);
      uploadOk = LittleFS.rename("/splash.tmp", SPLASH_PATH);
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (uploadFile) uploadFile.close();
    LittleFS.remove("/splash.tmp");
  }
}

static void handleUploadDone() {
  if (!fsOk) {
    server->send(500, "text/plain", "memoria pentru imagini (LittleFS) nu e disponibila - verifica Partition Scheme");
    return;
  }
  if (!uploadOk) {
    server->send(500, "text/plain", "imaginea nu a putut fi salvata (prea mare?)");
    return;
  }
  Serial.printf("[CFG] Imagine de start salvata: %u bytes\n", (unsigned)uploadSize);
  server->send(200, "text/plain", "ok");
  tft.fillScreen(TFT_BLACK);
  tft.drawJpgFile(LittleFS, SPLASH_PATH, 0, 0);   // previzualizare pe display
}

static void handleDeleteImage() {
  if (fsOk) LittleFS.remove(SPLASH_PATH);
  server->send(200, "text/plain", "ok");
  drawConfigScreen("Imaginea de start a fost stearsa");
}

// Orice alta adresa (ex. verificarile automate de internet ale iPhone/Android)
// e redirectionata spre pagina de setari -> telefonul o deschide singur.
static void handleNotFound() {
  server->sendHeader("Location", "http://192.168.4.1/", true);
  server->send(302, "text/plain", "");
}

static void startConfigMode() {
  configMode = true;
  Serial.println("[CFG] Intru in MOD SETARI (Bluetooth oprit, WiFi pornit)");
  drawConfigScreen("Pornesc WiFi...");

  // asteptam sa fie eliberat butonul, ca apasarea de intrare sa nu fie luata drept iesire
  while (digitalRead(BOOT_PIN) == LOW) delay(10);
  delay(50);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  dnsServer.start(53, "*", WiFi.softAPIP());

  server = new WebServer(80);
  server->on("/", HTTP_GET, handleRoot);
  server->on("/config", HTTP_GET, handleGetConfig);
  server->on("/preview", HTTP_POST, handlePreview);
  server->on("/save", HTTP_POST, handleSave);
  server->on("/splash.jpg", HTTP_GET, handleGetSplash);
  server->on("/upload", HTTP_POST, handleUploadDone, handleUploadChunk);
  server->on("/delimg", HTTP_POST, handleDeleteImage);
  server->onNotFound(handleNotFound);
  server->begin();

  Serial.printf("[CFG] WiFi '%s' pornit, IP %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  drawConfigScreen(fsOk ? "Astept telefonul..." : "ATENTIE: LittleFS indisponibil - doar culori");
}

static void configLoop() {
  static uint32_t lastActivity = millis();
  static uint32_t pressStart = 0;

  dnsServer.processNextRequest();
  server->handleClient();

  if (WiFi.softAPgetStationNum() > 0) lastActivity = millis();
  if (millis() - lastActivity > CONFIG_TIMEOUT_MS) {
    Serial.println("[CFG] Niciun telefon conectat de 10 minute -> repornire normala");
    ESP.restart();
  }

  // BOOT apasat ~0.2s -> iesire fara salvare
  if (digitalRead(BOOT_PIN) == LOW) {
    if (!pressStart) pressStart = millis();
    else if (millis() - pressStart > 200) {
      drawConfigScreen("Iesire fara salvare...");
      delay(300);
      ESP.restart();
    }
  } else {
    pressStart = 0;
  }
  delay(2);
}

/******************** ELM327 - COMUNICATIE ********************/
String lastEcuMsg = "";   // ultimul raspuns/eroare de la ECU, afisat pe ecran
String pidSuffix = "";    // "1" daca adaptorul suporta raspuns rapid (vezi elmFindEcu)
float baroKpa = 101.0f;   // presiune atmosferica, folosita la VACUUM (actualizata periodic)

// Cauta in raspuns marker-ul (ex. "41 0C") si extrage urmatorii n octeti hex
static bool parseResponseBytes(const String &r, const String &marker, uint8_t *out, int n) {
  int idx = r.indexOf(marker);
  if (idx < 0) return false;
  int pos = idx + marker.length();
  for (int i = 0; i < n; i++) {
    while (pos < (int)r.length() && r[pos] == ' ') pos++;
    if (pos + 1 >= (int)r.length()) return false;
    out[i] = (uint8_t)strtol(r.substring(pos, pos + 2).c_str(), nullptr, 16);
    pos += 2;
  }
  return true;
}

static uint32_t bytesToMask(const uint8_t b[4]) {
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
}

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

      // Lista PID-urilor suportate: 0100 -> PID 01-20, 0120 -> 21-40, 0140 -> 41-60.
      // Ultimul bit al fiecarui raspuns spune daca exista urmatorul grup.
      uint32_t m[3] = { 0, 0, 0 };
      uint8_t b[4];
      if (parseResponseBytes(r, "41 00", b, 4)) {
        m[0] = bytesToMask(b);
        if ((m[0] & 1) && parseResponseBytes(elmSendCommand("0120", 4000), "41 20", b, 4)) {
          m[1] = bytesToMask(b);
          if ((m[1] & 1) && parseResponseBytes(elmSendCommand("0140", 4000), "41 40", b, 4)) m[2] = bytesToMask(b);
        }
        saveSupported(m);
      }

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
  return parseResponseBytes(r, "41 " + pid.substring(2), out, expectedBytes);
}

static bool readBatteryVoltage(float &v) {
  // ATRV = comanda specifica adaptorului ELM327 (nu PID OBD), citeste direct
  // tensiunea de pe pinul 16 al OBD2 - functioneaza chiar daca ECU-ul nu raspunde.
  String r = elmSendCommand("ATRV", 3000);
  int idx = r.indexOf('V');
  if (idx < 1) return false;
  v = r.substring(0, idx).toFloat();
  return v > 0.0f;
}

// Citeste orice parametru din tabelul PARAMS si aplica formula OBD-II standard
static bool readParam(uint8_t id, float &v) {
  if (id == P_NONE || id >= PARAM_COUNT) return false;
  if (id == P_BATTERY) return readBatteryVoltage(v);

  const ParamDef &p = PARAMS[id];
  uint8_t b[2] = { 0, 0 };
  char pid[5];
  snprintf(pid, sizeof(pid), "01%02X", p.pid);
  if (!queryPidBytes(pid, b, p.bytes)) return false;

  float A = b[0], B = b[1];
  switch (p.formula) {
    case X_A:        v = A; break;
    case X_A_M40:    v = A - 40; break;
    case X_PCT:      v = A * 100.0f / 255.0f; break;
    case X_AB_4:     v = (A * 256 + B) / 4.0f; break;
    case X_AB:       v = A * 256 + B; break;
    case X_TIMING:   v = A / 2.0f - 64; break;
    case X_TRIM:     v = (A - 128) * 100.0f / 128.0f; break;
    case X_O2:       v = A / 200.0f; break;
    case X_AB_1000:  v = (A * 256 + B) / 1000.0f; break;
    case X_ABS_LOAD: v = (A * 256 + B) * 100.0f / 255.0f; break;
    case X_LAMBDA:   v = (A * 256 + B) / 32768.0f; break;
    case X_DTC:      v = (float)(b[0] & 0x7F); break;   // bitul 7 = martor "check engine"
    default:         v = A; break;
  }
  if (id == P_BARO) baroKpa = v;
  if (id == P_VACUUM) v -= baroKpa;   // negativ = depresiune
  return true;
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

  pinMode(BOOT_PIN, INPUT_PULLUP);
  loadSettings();
  fsOk = LittleFS.begin(true);   // true = formateaza la prima utilizare
  if (!fsOk) Serial.println("[SYS] LittleFS indisponibil - verifica Partition Scheme (Huge APP)");

  tft.init();
  tft.setRotation(1);

  // Imaginea de start + verificare buton BOOT. Daca intram in setari, NU pornim
  // Bluetooth-ul deloc (WiFi si Bluetooth impreuna nu incap bine in RAM).
  if (showSplashAndCheckBoot()) {
    startConfigMode();
    return;
  }

  // Bluetooth-ul porneste inaintea LVGL, cat timp e inca destula RAM interna libera
  // (Bluedroid are nevoie de ~100KB+; daca bufferele LVGL sunt alocate inainte, poate crapa)
  if (!SerialBT.begin("ESP32_Skoda_Test", true)) {   // true = master, initiaza el conexiunea
    Serial.println("[BT] SerialBT.begin() a ESUAT - memorie insuficienta?");
  }
  Serial.printf("[SYS] RAM libera dupa pornire BT: %u bytes\n", (unsigned)ESP.getFreeHeap());

  lv_init();

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

// Tensiunea bateriei (ATRV) merge si fara ECU -> actualizam casutele BATTERY, daca exista
static void updateBattery() {
  bool any = false;
  for (uint8_t ci = 0; ci < activeCount; ci++) any |= (cfgSlots[activeSlot[ci]].param == P_BATTERY);
  if (!any) return;
  float v;
  bool ok = readBatteryVoltage(v);
  for (uint8_t ci = 0; ci < activeCount; ci++)
    if (cfgSlots[activeSlot[ci]].param == P_BATTERY) showValue(ci, ok, v);
  lv_timer_handler();
}

void loop() {
  // 0) Mod setari: doar pagina web, fara Bluetooth/LVGL
  if (configMode) {
    configLoop();
    return;
  }

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
  bool slowCycle = (cycle++ % 5 == 0);
  uint32_t cycleStart = millis();
  int okCount = 0, total = 0;

  // VACUUM are nevoie de presiunea atmosferica -> o reimprospatam pe ciclurile lente
  if (slowCycle && pidSupported(0x33)) {
    for (uint8_t ci = 0; ci < activeCount; ci++)
      if (cfgSlots[activeSlot[ci]].param == P_VACUUM) { float b; readParam(P_BARO, b); break; }
  }

  for (uint8_t ci = 0; ci < activeCount; ci++) {
    uint8_t id = cfgSlots[activeSlot[ci]].param;
    const ParamDef &p = PARAMS[id];
    if (p.slow && !slowCycle) continue;   // temperaturi, tensiuni etc. - doar la fiecare al 5-lea ciclu
    if (!paramSupported(id)) continue;    // ECU-ul nu il are -> ramane "n/a", nu pierdem timp
    float v;
    bool ok = readParam(id, v);
    showValue(ci, ok, v);
    lv_timer_handler();                   // fiecare PID poate dura ~0.1-4s -> afisam imediat
    if (p.pid != PID_SPECIAL) { total++; okCount += ok; }   // ATRV nu spune nimic despre ECU
  }
  Serial.printf("[LOOP] ciclu %u ms, %d/%d PID OK\n", (unsigned)(millis() - cycleStart), okCount, total);

  if (total > 0 && okCount == 0) {
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
