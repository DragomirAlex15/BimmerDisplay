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
  X_A, X_A_M40, X_PCT, X_AB_4, X_AB, X_TIMING, X_TRIM, X_O2, X_AB_1000, X_ABS_LOAD, X_LAMBDA, X_DTC,
  X_AB10, X_EGT, X_S16_100, X_AB_DIV10, X_AB_20, X_AB_100, X_A_M125, X_AB_32
};

struct ParamDef {
  const char *name;   // titlul de pe ecran
  const char *unit;
  uint8_t pid;        // PID OBD Mode 01; PID_SPECIAL = nu e PID (ex. ATRV)
  uint8_t bytes;      // cati octeti de date folosesc formula (A, sau A+B)
  uint8_t formula;
  uint8_t fmt;
  bool slow;          // valoare care se schimba lent -> citita doar la fiecare al 5-lea ciclu
  const char *desc;   // descrierea din pagina de setari
  uint8_t off;        // de la al catelea octet al raspunsului incepe valoarea (PID-urile 0x70+
                      // au intai un octet care spune ce senzori exista); implicit 0
  uint8_t grp;        // grupa in pagina: 0 = generale, 1 = turbo / diesel
};

#define PID_SPECIAL 0xFF
#define P_NONE      0
#define P_BATTERY   8
#define P_BARO      14
#define P_VACUUM    15
#define P_BOOST     23
#define P_BOOST_CMD 24
#define P_BOOST_ACT 25

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

  // --- turbo / diesel (OBD standard; le au mai ales masinile noi, ~2010+ / Euro 5-6) ---
  // Formulele pentru PID-urile 0x70+ sunt dupa SAE J1979, dar NU au fost inca testate pe o
  // masina reala - verifica valorile cand le folosesti prima data.
  /*23*/ { "BOOST",    "bar",  0x0B,        1, X_A,        F_2DEC, false, "Presiune turbo (MAP minus atmosferica)", 0, 1 },
  /*24*/ { "BST CMD",  "bar",  0x70,        2, X_AB_32,    F_2DEC, false, "Presiune turbo ceruta de calculator", 1, 1 },
  /*25*/ { "BST ACT",  "bar",  0x70,        2, X_AB_32,    F_2DEC, false, "Presiune turbo reala (senzor)", 3, 1 },
  /*26*/ { "TURBO",    "rpm",  0x74,        2, X_AB10,     F_INT,  false, "Turatie turbina", 1, 1 },
  /*27*/ { "CAC TEMP", "C",    0x77,        1, X_A_M40,    F_INT,  true,  "Temperatura aer dupa intercooler", 1, 1 },
  /*28*/ { "EGT",      "C",    0x78,        2, X_EGT,      F_INT,  false, "Temperatura gaze evacuare", 1, 1 },
  /*29*/ { "DPF DP",   "kPa",  0x7A,        2, X_S16_100,  F_1DEC, false, "Filtru particule: presiune diferentiala (mai mare = mai incarcat)", 1, 1 },
  /*30*/ { "DPF TEMP", "C",    0x7C,        2, X_EGT,      F_INT,  true,  "Filtru particule: temperatura la intrare", 1, 1 },
  /*31*/ { "RAIL",     "bar",  0x23,        2, X_AB_DIV10, F_INT,  false, "Presiune rampa combustibil (diesel)", 0, 1 },
  /*32*/ { "EGR CMD",  "%",    0x2C,        1, X_PCT,      F_INT,  false, "Recirculare gaze (EGR) comandata", 0, 1 },
  /*33*/ { "EGR ERR",  "%",    0x2D,        1, X_TRIM,     F_INT,  false, "Eroare recirculare gaze (EGR)", 0, 1 },
  /*34*/ { "EXH PRES", "kPa",  0x73,        2, X_AB_100,   F_1DEC, false, "Presiune gaze evacuare", 1, 1 },

  // --- alte generale ---
  /*35*/ { "OIL TEMP", "C",    0x5C,        1, X_A_M40,    F_INT,  true,  "Temperatura ulei motor" },
  /*36*/ { "FUEL L/H", "L/h",  0x5E,        2, X_AB_20,    F_1DEC, false, "Consum instantaneu" },
  /*37*/ { "MAF",      "g/s",  0x10,        2, X_AB_100,   F_1DEC, false, "Debit aer (senzor MAF)" },
  /*38*/ { "AMBIENT",  "C",    0x46,        1, X_A_M40,    F_INT,  true,  "Temperatura exterioara" },
  /*39*/ { "FUEL LVL", "%",    0x2F,        1, X_PCT,      F_INT,  true,  "Nivel combustibil" },
  /*40*/ { "TORQUE",   "%",    0x62,        1, X_A_M125,   F_INT,  false, "Cuplu motor real (% din cuplul de referinta)" },
};
const uint8_t PARAM_COUNT = sizeof(PARAMS) / sizeof(PARAMS[0]);

#define MAX_SLOTS 9   // grila maxima 3 x 3
#define SLOT_HAS_MIN 0x01
#define SLOT_HAS_MAX 0x02

// O casuta de pe ecran, asa cum e salvata in setari
struct SlotCfg {
  uint8_t param;   // index in PARAMS
  uint8_t flags;   // SLOT_HAS_MIN / SLOT_HAS_MAX
  float vmin;      // sub minim -> culoare de avertizare
  float vmax;      // peste maxim -> culoare de avertizare
};

// O casuta afisata (obiectele LVGL)
struct Cell {
  lv_obj_t *title;
  lv_obj_t *val;
  lv_obj_t *line;   // linia de sub valoare (stil afisaj LCD de bord)
};

// Fonturi stil LCD de bord (pixelate, ingrosate) - generate in lcd_fonts.c
LV_FONT_DECLARE(lcd_font_16);   // titluri, status
LV_FONT_DECLARE(lcd_font_24);   // valori (3 coloane), coduri de eroare
LV_FONT_DECLARE(lcd_font_32);   // valori (2 coloane)
LV_FONT_DECLARE(lcd_font_40);   // valori (1 coloana)

/******************** CODURI DE EROARE (DTC) ********************/
struct Dtc {
  uint16_t raw;     // cei 2 octeti de la ECU (litera + 4 cifre), ex. P0301 = 0x0301
  bool pending;     // true = eroare "in asteptare" (Mode 07), false = memorata (Mode 03)
};

#define MAX_DTC 16
Dtc dtcList[MAX_DTC];
uint8_t dtcCount = 0;
bool dtcKnown = false;                 // am citit erorile macar o data
bool milOn = false;                    // martorul "check engine" aprins
uint32_t dtcReadAt = 0;                // cand au fost citite ultima data
bool dtcRefreshRequested = false;      // citire imediata (la intrarea pe ecranul de erori)
const uint32_t DTC_INTERVAL_MS = 60000;      // pe ecranul cu date live: o data pe minut
const uint32_t DTC_INTERVAL_ERR_MS = 30000;  // pe ecranul de erori: la 30 secunde

// Butonul BOOT in timpul mersului: comuta intre date live si lista de erori.
// Butonul e citit periodic (si in timp ce asteptam raspunsuri de la ELM327) si
// conteaza doar daca e tinut apasat minim BOOT_TOGGLE_MS -> impulsurile scurte de
// pe GPIO0 (circuitul de programare USB, paraziti) nu mai schimba ecranul singure.
const uint32_t BOOT_TOGGLE_MS = 60;
bool errScreen = false;

// Codul ca text, ex. "P0301", din cei 2 octeti primiti de la ECU:
// primii 2 biti = litera (00=P, 01=C, 10=B, 11=U), restul = cele 4 cifre
static void dtcToStr(uint16_t raw, char *out, size_t n) {
  static const char LETTERS[] = "PCBU";
  snprintf(out, n, "%c%04X", LETTERS[raw >> 14], raw & 0x3FFF);
}

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

// PID-uri suportate de ECU (raspunsurile la 0100 / 0120 / ... / 01A0 = PID 01..C0),
// salvate la prima conectare
#define SUP_GROUPS 6
uint32_t supMask[SUP_GROUPS] = { 0 };
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
// Casutele implicite = cele 8 valori de pana acum (a 9-a goala), cu cateva limite utile
static void defaultSlots() {
  const uint8_t def[MAX_SLOTS] = { 1, 3, 2, 5, 4, 6, 7, P_BATTERY, P_NONE };
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

  defaultSlots();
  size_t slotBytes = prefs.getBytesLength("slots");
  // accepta si setarile vechi cu 8 casute (a 9-a ramane goala)
  if (slotBytes == sizeof(cfgSlots) || slotBytes == 8 * sizeof(SlotCfg)) {
    if (slotBytes != sizeof(cfgSlots)) cfgSlots[MAX_SLOTS - 1] = { P_NONE, 0, 0.0f, 0.0f };
    prefs.getBytes("slots", cfgSlots, slotBytes);
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
  prefs.putBytes("slots", cfgSlots, sizeof(cfgSlots));
  prefs.end();
}

// Scrie in NVS doar daca lista s-a schimbat (protejeaza memoria flash)
static void saveSupported(const uint32_t m[SUP_GROUPS]) {
  if (supKnown && memcmp(m, supMask, sizeof(supMask)) == 0) return;
  memcpy(supMask, m, sizeof(supMask));
  supKnown = true;
  prefs.begin("display", false);
  prefs.putBytes("sup", supMask, sizeof(supMask));
  prefs.putBool("supok", true);
  prefs.end();
  Serial.print("[ELM] PID-uri suportate salvate:");
  for (int i = 0; i < SUP_GROUPS; i++) Serial.printf(" %08X", (unsigned)supMask[i]);
  Serial.println();
}

// true daca ECU-ul suporta PID-ul (sau daca inca nu stim - atunci incercam oricum)
static bool pidSupported(uint8_t pid) {
  if (pid == PID_SPECIAL || !supKnown) return true;
  if (pid < 1 || pid > SUP_GROUPS * 32) return false;
  uint8_t i = (pid - 1) / 32;
  uint8_t bit = 31 - ((pid - 1) % 32);
  return (supMask[i] >> bit) & 1;
}

// parametrii calculati fata de presiunea atmosferica (au nevoie si de PID 0x33)
static bool needsBaro(uint8_t id) {
  return id == P_VACUUM || id == P_BOOST || id == P_BOOST_CMD || id == P_BOOST_ACT;
}

static bool paramSupported(uint8_t id) {
  if (id == P_NONE || id >= PARAM_COUNT) return false;
  if (needsBaro(id) && !pidSupported(0x33)) return false;
  return pidSupported(PARAMS[id].pid);
}

/******************** UI ********************/
lv_obj_t *lbl_status;
lv_style_t style_status;
lv_obj_t *scr_dash = nullptr;      // ecranul cu date live
lv_obj_t *scr_err = nullptr;       // ecranul cu lista de erori
lv_obj_t *lbl_dtc_badge = nullptr; // "! 2 ERORI" sus in dreapta pe ecranul live
lv_obj_t *lbl_err_info = nullptr;
lv_obj_t *lbl_err_list = nullptr;      // mesaj cand nu sunt coduri
lv_obj_t *lbl_codes[MAX_DTC];          // cate o eticheta pentru fiecare cod, in grila 3 coloane

Cell cells[MAX_SLOTS];
uint8_t activeSlot[MAX_SLOTS];   // pentru fiecare casuta afisata: indexul ei in cfgSlots
uint8_t activeCount = 0;

// Marimea valorilor depinde doar de cate coloane are grila (fix, nu se mai alege din pagina)
static const lv_font_t *valueFontFor(uint8_t cols) {
  return cols <= 1 ? &lcd_font_40 : cols == 2 ? &lcd_font_32 : &lcd_font_24;
}

// Linie plina (bara de sub valoare)
static lv_obj_t *mkBar(lv_obj_t *parent, int x, int y, int w, int h) {
  lv_obj_t *b = lv_obj_create(parent);
  lv_obj_remove_style_all(b);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(b, lv_color_hex(cfgFg), 0);
  lv_obj_set_pos(b, x, y);
  lv_obj_set_size(b, w, h);
  lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
  return b;
}

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
  scr_dash = scr;
  lv_obj_set_style_bg_color(scr, lv_color_hex(cfgBg), 0);   // culoare fundal din setari
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  lv_style_init(&style_status);
  lv_style_set_text_color(&style_status, lv_color_hex(cfgFg));
  lv_style_set_text_font(&style_status, &lcd_font_16);
  lv_style_set_bg_opa(&style_status, LV_OPA_TRANSP);
  lv_style_set_border_width(&style_status, 0);

  lbl_status = lv_label_create(scr);
  lv_label_set_text(lbl_status, "PORNIRE...");
  lv_obj_add_style(lbl_status, &style_status, 0);
  lv_obj_set_pos(lbl_status, 4, 4);
  lv_obj_set_width(lbl_status, 208);                     // loc in dreapta pentru indicatorul de erori
  lv_label_set_long_mode(lbl_status, LV_LABEL_LONG_CLIP);

  // indicator erori (gol cat timp nu sunt erori)
  lbl_dtc_badge = mkLabel(scr, "", &lcd_font_16, 0, 4);
  lv_obj_set_style_text_color(lbl_dtc_badge, lv_color_hex(cfgWarn), 0);
  lv_obj_align(lbl_dtc_badge, LV_ALIGN_TOP_RIGHT, -4, 4);

  // casutele alese in setari, in ordine, fara cele goale
  activeCount = 0;
  for (uint8_t i = 0; i < MAX_SLOTS; i++)
    if (cfgSlots[i].param != P_NONE) activeSlot[activeCount++] = i;

  if (activeCount == 0) {
    mkLabel(scr, "NICIUN PARAMETRU\nALES - INTRA IN\nSETARI (BOOT)", &lcd_font_24, 10, 70);
    return;
  }

  // aranjament automat dupa numarul de casute (maxim 3 x 3)
  uint8_t cols, rows;
  if (activeCount == 1)      { cols = 1; rows = 1; }
  else if (activeCount == 2) { cols = 2; rows = 1; }
  else if (activeCount <= 4) { cols = 2; rows = 2; }
  else if (activeCount <= 6) { cols = 3; rows = 2; }
  else                       { cols = 3; rows = 3; }

  const int top = 24;   // sub randul de status
  const int cw = TFT_W / cols;
  const int ch = (TFT_H - top) / rows;
  const lv_font_t *tf = &lcd_font_16;
  const lv_font_t *vf = valueFontFor(cols);
  const int th = lv_font_get_line_height(tf);
  const int vh = lv_font_get_line_height(vf);
  const int block = th + 4 + vh + 2 + 2;   // titlu + spatiu + valoare + spatiu + linie
  const int offY = max(0, (ch - block) / 2);

  for (uint8_t ci = 0; ci < activeCount; ci++) {
    uint8_t id = cfgSlots[activeSlot[ci]].param;
    int x = (ci % cols) * cw + 4;
    int y = top + (ci / cols) * ch + offY;
    int vy = y + th + 4;

    // ca pe afisajele de bord: titlul fara unitate de masura
    Cell &c = cells[ci];
    c.title = mkLabel(scr, PARAMS[id].name, tf, x, y);
    // "--" = astept prima citire; "X" = masina nu are acest parametru / nu raspunde
    c.val   = mkLabel(scr, paramSupported(id) ? "--" : "X", vf, x, vy);
    c.line  = mkBar(scr, x, vy + vh + 2, cw - 14, 2);
  }
}

/******************** ECRAN ERORI ********************/
static void create_error_screen() {
  scr_err = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr_err, lv_color_hex(cfgBg), 0);
  lv_obj_set_style_bg_opa(scr_err, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr_err, LV_OBJ_FLAG_SCROLLABLE);

  mkLabel(scr_err, "ERORI MOTOR", &lcd_font_24, 4, 4);
  mkBar(scr_err, 4, 30, TFT_W - 8, 2);
  lbl_err_info = mkLabel(scr_err, "", &lcd_font_16, 4, 36);
  lv_obj_set_width(lbl_err_info, TFT_W - 8);
  lv_label_set_long_mode(lbl_err_info, LV_LABEL_LONG_CLIP);

  lbl_err_list = mkLabel(scr_err, "", &lcd_font_24, 4, 100);

  // grila de coduri: 3 coloane x 6 randuri (incap toate cele MAX_DTC = 16)
  for (uint8_t i = 0; i < MAX_DTC; i++) {
    lbl_codes[i] = mkLabel(scr_err, "", &lcd_font_24, 2 + (i % 3) * 106, 58 + (i / 3) * 26);
    lv_obj_set_style_text_color(lbl_codes[i], lv_color_hex(cfgWarn), 0);
  }

  mkLabel(scr_err, "BOOT=INAPOI ?=ASTEPTARE", &lcd_font_16, 4, TFT_H - 18);
}

static void refreshErrorScreen() {
  if (!scr_err) return;

  // randul de info: starea conexiunii + martor + cand am citit
  char info[64];
  if (!dtcKnown) {
    snprintf(info, sizeof(info), "%s", elmEcuOk ? "CITESC ERORILE..." : "NECONECTAT LA ECU");
  } else if (elmEcuOk) {
    snprintf(info, sizeof(info), "MARTOR: %s  ACUM %uS", milOn ? "APRINS" : "STINS",
             (unsigned)((millis() - dtcReadAt) / 1000));
  } else {
    snprintf(info, sizeof(info), "MARTOR: %s (DECONECTAT)", milOn ? "APRINS" : "STINS");
  }
  lv_label_set_text(lbl_err_info, info);

  // doar codurile; "?" dupa cod = eroare in asteptare (Mode 07)
  for (uint8_t i = 0; i < MAX_DTC; i++) {
    if (dtcKnown && i < dtcCount) {
      char code[8];
      dtcToStr(dtcList[i].raw, code, sizeof(code));
      if (dtcList[i].pending) strcat(code, "?");
      lv_label_set_text(lbl_codes[i], code);
    } else {
      lv_label_set_text(lbl_codes[i], "");
    }
  }
  lv_label_set_text(lbl_err_list, (dtcKnown && dtcCount == 0) ? "NICIO EROARE" : "");
}

// Indicatorul de pe ecranul live
static void updateDtcBadge() {
  if (!lbl_dtc_badge) return;
  if (dtcKnown && dtcCount > 0) {
    char b[24];
    snprintf(b, sizeof(b), "!%u %s", dtcCount, dtcCount == 1 ? "EROARE" : "ERORI");
    lv_label_set_text(lbl_dtc_badge, b);
  } else {
    lv_label_set_text(lbl_dtc_badge, "");
  }
  lv_obj_align(lbl_dtc_badge, LV_ALIGN_TOP_RIGHT, -4, 4);
}

// Daca BOOT a fost tinut apasat minim BOOT_TOGGLE_MS, comuta ecranul (o singura data
// pe apasare - trebuie eliberat inainte de urmatoarea). Returneaza true daca s-a comutat.
static bool checkScreenToggle() {
  static uint32_t lowSince = 0;   // de cand e apasat (0 = eliberat)
  static bool handled = false;    // apasarea curenta a fost deja folosita
  if (!scr_err) return false;

  if (digitalRead(BOOT_PIN) != LOW) {
    lowSince = 0;
    handled = false;
    return false;
  }
  if (!lowSince) lowSince = millis();
  if (handled || millis() - lowSince < BOOT_TOGGLE_MS) return false;
  handled = true;

  errScreen = !errScreen;
  if (errScreen) {
    refreshErrorScreen();
    lv_scr_load(scr_err);
    dtcRefreshRequested = true;   // citim erorile proaspat cand intram pe ecran
  } else {
    lv_scr_load(scr_dash);
  }
  lv_timer_handler();
  return true;
}

static void setCellText(Cell &c, const char *txt) {
  lv_label_set_text(c.val, txt);
}

static void setCellColor(Cell &c, uint32_t rgb) {
  lv_color_t k = lv_color_hex(rgb);
  lv_obj_set_style_text_color(c.val, k, 0);
  lv_obj_set_style_text_color(c.title, k, 0);
  lv_obj_set_style_bg_color(c.line, k, 0);
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
    setCellText(c, "X");   // selectat, dar fara date de la masina
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
  String up = s;
  up.toUpperCase();   // stil afisaj de bord: totul cu majuscule
  lv_label_set_text(lbl_status, up.c_str());
  Serial.println(s);
}

/******************** TEXT STIL LCD (desenat direct, fara LVGL) ********************/
// Acelasi font 5x7 ca lcd_fonts.c, marit de `size` ori si ingrosat cu 1px,
// pentru ecranele desenate direct (start, mod setari, previzualizare).
static void lcdText(const String &s, int x, int y, uint8_t size, uint32_t rgb,
                    textdatum_t datum = top_left, bool upper = true) {
  String up = s;
  if (upper) up.toUpperCase();
  tft.setFont(&fonts::Font0);
  tft.setTextSize(size);
  tft.setTextDatum(datum);
  tft.setTextColor(rgb565(rgb));
  tft.drawString(up, x, y);
  tft.drawString(up, x + 1, y);   // ingrosare
  tft.setTextSize(1);
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
    lcdText("Skoda Fabia", TFT_W / 2, TFT_H / 2 - 14, 4, cfgFg, middle_center);
    lcdText("1.2 12V BZG", TFT_W / 2, TFT_H / 2 + 24, 2, cfgFg, middle_center);
  }

  // (fara text de indiciu - butonul BOOT functioneaza in continuare, doar nu mai e afisat)
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
// Max ~25 caractere pe rand la marimea 2
static void drawConfigScreen(const char *footer) {
  tft.fillScreen(rgb565(cfgBg));
  lcdText("MOD SETARI", 4, 6, 3, cfgFg);
  tft.fillRect(4, 32, TFT_W - 8, 2, rgb565(cfgFg));

  lcdText("1. WIFI PE TELEFON:", 4, 42, 2, cfgFg);
  lcdText("   " AP_SSID, 4, 62, 2, cfgWarn, top_left, false);   // numele retelei e sensibil la majuscule
  lcdText("   PAROLA " AP_PASS, 4, 82, 2, cfgWarn);
  lcdText("2. PAGINA SE DESCHIDE", 4, 106, 2, cfgFg);
  lcdText("   SINGURA, SAU:", 4, 126, 2, cfgFg);
  lcdText("   192.168.4.1", 4, 146, 2, cfgWarn);
  lcdText("BOOT = IESIRE", 4, 174, 2, cfgFg);

  tft.fillRect(4, TFT_H - 26, TFT_W - 8, 2, rgb565(cfgFg));
  lcdText(footer, 4, TFT_H - 20, 2, cfgFg);
}

// Previzualizare culori direct pe display, cat timp le alegi din telefon
static void drawColorPreview(uint32_t bg, uint32_t fg, uint32_t wc) {
  tft.fillScreen(rgb565(bg));
  lcdText("PREVIZUALIZARE", 4, 4, 2, fg);

  lcdText("RPM", 4, 40, 2, fg);
  lcdText("2500", 4, 60, 3, fg);
  tft.fillRect(4, 86, 92, 2, rgb565(fg));

  lcdText("COOLANT", 110, 40, 2, wc);   // exemplu de valoare in afara limitelor
  lcdText("112", 110, 60, 3, wc);
  tft.fillRect(110, 86, 92, 2, rgb565(wc));

  lcdText("SALVEAZA IN PAGINA", 4, TFT_H - 20, 2, fg);
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
canvas#prev,canvas#wex{display:block;width:100%;image-rendering:pixelated;image-rendering:crisp-edges;border-radius:8px}
canvas#wex{margin-top:8px}
select{flex:1;min-width:0;padding:8px;border-radius:8px;border:1px solid #444;background:#000;color:#eee;font-size:15px}
input[type=checkbox]{width:24px;height:24px}
input[type=number]{flex:1;min-width:0;padding:8px;border-radius:8px;border:1px solid #444;background:#000;color:#eee;font-size:15px}
.slot{border-top:1px solid #333;padding:10px 0}
.slot .hd{display:flex;align-items:center;gap:6px;margin-bottom:6px}.slot .hd label{flex:1;color:#bbb}
.slot .hd button{width:auto;margin:0;padding:6px 14px;font-size:15px;background:#444}
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
<canvas id="prev" width="320" height="240" aria-label="Previzualizarea ecranului"></canvas>
<canvas id="wex" width="320" height="52" aria-label="Exemplu valoare în afara limitelor"></canvas>
<small>Desenat pixel cu pixel, cu același font și aranjament ca pe display. Jos: cum arată o valoare în afara limitelor.</small>
</section>

<section><h2>Parametri afișați</h2>
<div id="slots"></div>
<small>Ecranul se aranjează singur după câte căsuțe folosești (1&ndash;9, maxim 3 &times; 3), în ordinea de aici; cu ▲ ▼ muți o căsuță mai sus sau mai jos. Căsuțele goale sunt sărite. Dacă mașina nu are un parametru ales, pe ecran apare X. Cu mai puține căsuțe, valorile se actualizează mai des. Min / max sunt opționale: când valoarea iese din interval, căsuța devine în culoarea de avertizare.</small><br>
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
const NS=9;                                  // numar de casute (grila maxima 3 x 3)
function body(){const b=new URLSearchParams({bg:$('bg').value,fg:$('fg').value,wc:$('wc').value});
 for(let i=0;i<NS;i++){if(!$('s'+i))break;b.append('s'+i,$('s'+i).value);b.append('n'+i,$('n'+i).value);b.append('x'+i,$('x'+i).value);}
 return b;}
function sendPreview(){clearTimeout(timer);timer=setTimeout(()=>fetch('/preview',{method:'POST',body:body()}),300);}
// Fontul 5x7 de pe display (acelasi ca lcd_fonts.c): 5 octeti pe caracter (coloane,
// bitul 0 = randul de sus), pentru caracterele ASCII 0x20..0x7E.
const F='000000000000005f00000007000700147f147f14242a7f2a12231308646236495620500008070300001c2241000041221c002a1c7f1c2a08083e080800807030000808080808000060600020100804023e5149453e00427f400072494949462141494d331814127f1027454545393c4a49493141211109073649494936464949291e0000140000004034000000081422411414141414004122140802015909063e415d594e7c1211127c7f494949363e414141227f4141413e7f494949417f090909013e414151737f0808087f00417f41002040413f017f081422417f404040407f021c027f7f0408107f3e4141413e7f090909063e4151215e7f09192946264949493203017f01033f4040403f1f2040201f3f4038403f631408146303047804036159494d43007f4141410204081020004141417f04020102044040404040000307080020545478407f284444383844444428384444287f385454541800087e090218a4a49c787f0804047800447d40002040403d007f1028440000417f40007c047804787c080404783844444438fc1824241818242418fc7c08040408485454542404043f44243c4040207c1c2040201c3c4030403c44281028444c9090907c4464544c440008364100000077000000413608000201020402';
// Text ca pe display: fiecare pixel al fontului marit de s ori, ingrosat cu b pixeli spre dreapta
function txt(g,t,x,y,s,b,col){g.fillStyle=col;t=String(t);
 for(let n=0;n<t.length;n++){const i=t.charCodeAt(n)-32;if(i<0||i>94)continue;const ox=x+n*6*s;
  for(let cx=0;cx<5;cx++){const c=parseInt(F.substr((i*5+cx)*2,2),16);
   for(let ry=0;ry<8;ry++)if((c>>ry)&1)g.fillRect(ox+cx*s,y+ry*s,s+b,s);}}}
// Previzualizare: acelasi aranjament si aceleasi marimi ca create_dashboard() din firmware
function render(){const g=$('prev').getContext('2d'),fg=$('fg').value,bg=$('bg').value,wc=$('wc').value;
 g.fillStyle=bg;g.fillRect(0,0,320,240);   // fara randul de status: in mers dispare dupa 5s
 const act=[];for(let i=0;i<NS;i++){const e=$('s'+i);if(e&&+e.value)act.push(+e.value);}
 const n=act.length;
 if(!n){['NICIUN PARAMETRU','ALES - INTRA IN','SETARI (BOOT)'].forEach((l,k)=>txt(g,l,10,70+k*24,3,1,fg));}
 else{let c,r;if(n==1){c=1;r=1}else if(n==2){c=2;r=1}else if(n<=4){c=2;r=2}else if(n<=6){c=3;r=2}else{c=3;r=3}
  const s=c<=1?5:c==2?4:3,b=s>=4?2:1,cw=Math.floor(320/c),ch=Math.floor(216/r),vh=8*s,blk=16+4+vh+2+2,oy=Math.max(0,Math.floor((ch-blk)/2));
  act.forEach((id,i)=>{const q=P[id],x=(i%c)*cw+4,y=24+Math.floor(i/c)*ch+oy,vy=y+20;
   txt(g,q.n,x,y,2,1,fg);txt(g,q.s?SAMPLE[q.f]:'X',x,vy,s,b,fg);g.fillStyle=fg;g.fillRect(x,vy+vh+2,cw-14,2);});}
 // exemplu de valoare in afara limitelor, in culoarea de avertizare
 const w=$('wex').getContext('2d');w.fillStyle=bg;w.fillRect(0,0,320,52);
 txt(w,'PESTE LIMITA',4,2,2,1,wc);txt(w,'112',4,22,3,1,wc);w.fillStyle=wc;w.fillRect(4,48,92,2);}
function limVis(i){$('n'+i).parentNode.style.display=$('s'+i).value=='0'?'none':'flex';}
// interschimba doua casute (parametru + limite) -> schimba ordinea pe ecran
function swap(i,j){if(j<0||j>=NS)return;
 ['s','n','x'].forEach(p=>{const a=$(p+i),b=$(p+j),t=a.value;a.value=b.value;b.value=t;});
 limVis(i);limVis(j);render();}
function buildSlots(slots){const box=$('slots');box.innerHTML='';
 const GR=['Generale','Turbo / Diesel'];
 for(let i=0;i<NS;i++){const d=document.createElement('div');d.className='slot';
  d.innerHTML='<div class="hd"><label for="s'+i+'">Căsuța '+(i+1)+'</label>'+
   '<button type="button" id="u'+i+'" aria-label="Mută în sus">▲</button><button type="button" id="w'+i+'" aria-label="Mută în jos">▼</button></div>'+
   '<select id="s'+i+'"></select>'+
   '<div class="lim"><input type="number" step="any" id="n'+i+'" placeholder="min"><input type="number" step="any" id="x'+i+'" placeholder="max"></div>';
  box.appendChild(d);
  const s=$('s'+i);
  s.add(new Option('— gol —',0));
  GR.forEach((gn,g)=>{const og=document.createElement('optgroup');og.label=gn;
   P.forEach((q,k)=>{if(!k||(q.g|0)!=g)return;let t=q.n+(q.u?' ('+q.u+')':'')+' – '+q.d;if(!q.s)t+=' ✕ nesuportat';og.appendChild(new Option(t,k));});
   s.appendChild(og);});
  const c=slots[i]||{p:0};s.value=c.p;if(c.mn!=null)$('n'+i).value=c.mn;if(c.mx!=null)$('x'+i).value=c.mx;
  s.addEventListener('change',()=>{limVis(i);render();});limVis(i);
  $('u'+i).onclick=()=>swap(i,i-1);$('w'+i).onclick=()=>swap(i,i+1);}
 $('u0').disabled=true;$('w'+(NS-1)).disabled=true;}
['bg','fg','wc'].forEach(k=>{
 $(k).addEventListener('input',()=>{$(k+'h').value=$(k).value;render();sendPreview();});
 $(k+'h').addEventListener('change',()=>{let v=$(k+'h').value.trim();if(v[0]!='#')v='#'+v;
  if(/^#[0-9a-fA-F]{6}$/.test(v)){$(k).value=v.toLowerCase();$(k+'h').value=$(k).value;render();sendPreview();}else $(k+'h').value=$(k).value;});
});
window.addEventListener('resize',render);
function showCur(){$('cur').src='/splash.jpg?'+Date.now();$('cur').style.display='block';$('del').style.display='block';}
async function load(){try{const c=await (await fetch('/config')).json();P=c.params;
 ['bg','fg','wc'].forEach(k=>{$(k).value=c[k];$(k+'h').value=c[k];});
 buildSlots(c.slots);
 $('supnote').textContent=c.sup?'„✕ nesuportat” = calculatorul mașinii a raportat că nu are acest parametru (pe ecran apare X).'
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
  char tmp[240];
  bool img = fsOk && LittleFS.exists(SPLASH_PATH);
  String j;
  j.reserve(6000);
  snprintf(tmp, sizeof(tmp),
           "{\"bg\":\"#%06x\",\"fg\":\"#%06x\",\"wc\":\"#%06x\",\"img\":%s,\"sup\":%s,",
           (unsigned)cfgBg, (unsigned)cfgFg, (unsigned)cfgWarn,
           img ? "true" : "false", supKnown ? "true" : "false");
  j += tmp;

  // lista parametrilor: n=nume, u=unitate, d=descriere, f=format, s=suportat de ECU
  j += "\"params\":[";
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    const ParamDef &p = PARAMS[i];
    snprintf(tmp, sizeof(tmp), "%s{\"n\":\"%s\",\"u\":\"%s\",\"d\":\"%s\",\"f\":%u,\"g\":%u,\"s\":%s}",
             i ? "," : "", p.name, p.unit, p.desc, p.fmt, p.grp,
             (i == P_NONE || paramSupported(i)) ? "true" : "false");
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
  drawConfigScreen("SALVAT! REPORNIRE...");
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
  drawConfigScreen("IMAGINE STEARSA");
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
  drawConfigScreen("PORNESC WIFI...");

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
  drawConfigScreen(fsOk ? "ASTEPT TELEFONUL..." : "FARA MEMORIE IMAGINI");
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
      drawConfigScreen("IESIRE FARA SALVARE");
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
bool isCanProtocol = false;   // raspunsul la erori are alt format pe CAN fata de linia K

// Extrage din raspuns doar octetii hex de 2 caractere. Sare peste tot restul:
// lungimea "00C" si numerele de rand "0:", "1:" ale raspunsurilor CAN pe mai multe
// randuri, textele "NO DATA", "SEARCHING" etc.
static int responseToBytes(const String &r, uint8_t *bytes, int maxBytes) {
  int nb = 0, i = 0, len = r.length();
  while (i < len && nb < maxBytes) {
    while (i < len && r[i] == ' ') i++;
    int start = i;
    while (i < len && r[i] != ' ') i++;
    if (i - start == 2 && isxdigit(r[start]) && isxdigit(r[start + 1]))
      bytes[nb++] = (uint8_t)strtol(r.substring(start, i).c_str(), nullptr, 16);
  }
  return nb;
}

// Cauta in raspuns marker-ul (ex. "41 0C") si extrage urmatorii n octeti
static bool parseResponseBytes(const String &r, const String &marker, uint8_t *out, int n) {
  uint8_t mk[2];
  if (responseToBytes(marker, mk, 2) != 2) return false;
  uint8_t bytes[64];
  int nb = responseToBytes(r, bytes, sizeof(bytes));
  for (int k = 0; k + 1 < nb; k++) {
    if (bytes[k] != mk[0] || bytes[k + 1] != mk[1]) continue;
    if (k + 2 + n > nb) return false;
    memcpy(out, bytes + k + 2, n);
    return true;
  }
  return false;
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
      checkScreenToggle();   // BOOT raspunde imediat, chiar daca asteptam ELM327 (pana la 15s/comanda)
      delay(1);
    }
  }

  if (!gotPrompt) {
    Serial.printf("[ELM] TIMEOUT la '%s' - astept resincronizare...\n", cmd.c_str());
    uint32_t t = millis();
    while (millis() - t < 8000) {
      if (SerialBT.available() && SerialBT.read() == '>') break;
      checkScreenToggle();
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
  const char *protocols[] = { "0", "5", "4", "3", "6", "8", "7", "9" };
  // 0=AUTO, 5=KWP2000 fast init, 4=KWP2000 5-baud, 3=ISO9141 (linie K),
  // 6=CAN 11bit 500k (VAG pe CAN), 8=CAN 11bit 250k, 7=CAN 29bit 500k, 9=CAN 29bit 250k
  // Pe CAN nu exista initializare lunga ca pe linia K -> incercarile esuate dureaza putin.
  for (const char *p : protocols) {
    elmSendCommand(String("ATSP") + p, 2000);
    String r = elmSendCommand("0100", 15000);   // initializarea pe linia K poate dura ~10s
    if (r.indexOf("41 00") >= 0) {
      String dp = elmSendCommand("ATDPN", 2000);
      lastEcuMsg = "ECU OK PROT " + dp;
      // ATDPN: "6".."9" (sau "A6".."A9") = CAN; restul = linie K / J1850
      char pc = dp.length() ? dp[dp.length() - 1] : '0';
      isCanProtocol = (pc >= '6' && pc <= '9') || (pc >= 'A' && pc <= 'C');

      // Lista PID-urilor suportate: 0100 -> PID 01-20, 0120 -> 21-40, ... 01A0 -> A1-C0.
      // Ultimul bit al fiecarui raspuns spune daca exista urmatorul grup.
      uint32_t m[SUP_GROUPS] = { 0 };
      uint8_t b[4];
      if (parseResponseBytes(r, "41 00", b, 4)) {
        m[0] = bytesToMask(b);
        for (int g = 1; g < SUP_GROUPS && (m[g - 1] & 1); g++) {
          char cmd[5], mk[6];
          snprintf(cmd, sizeof(cmd), "01%02X", g * 0x20);
          snprintf(mk, sizeof(mk), "41 %02X", g * 0x20);
          if (!parseResponseBytes(elmSendCommand(cmd, 4000), mk, b, 4)) break;
          m[g] = bytesToMask(b);
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
  uint8_t b[8] = { 0 };
  char pid[5];
  snprintf(pid, sizeof(pid), "01%02X", p.pid);
  // citim octetii de dinaintea valorii (p.off) + octetii valorii
  if (!queryPidBytes(pid, b, p.off + max((int)p.bytes, 1))) return false;

  float A = b[p.off], B = b[p.off + 1];
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
    case X_DTC:      v = (float)(b[p.off] & 0x7F); break;   // bitul 7 = martor "check engine"
    case X_AB10:     v = (A * 256 + B) * 10; break;
    case X_EGT:      v = (A * 256 + B) / 10.0f - 40; break;
    case X_S16_100:  v = (int16_t)((b[p.off] << 8) | b[p.off + 1]) / 100.0f; break;
    case X_AB_DIV10: v = (A * 256 + B) / 10.0f; break;     // rampa: (AB*10 kPa) -> bar
    case X_AB_20:    v = (A * 256 + B) / 20.0f; break;
    case X_AB_100:   v = (A * 256 + B) / 100.0f; break;
    case X_A_M125:   v = A - 125; break;
    case X_AB_32:    v = (A * 256 + B) * 0.03125f; break;  // presiune turbo absoluta, kPa
    default:         v = A; break;
  }
  if (id == P_BARO) baroKpa = v;
  if (id == P_VACUUM) v -= baroKpa;   // negativ = depresiune
  if (id == P_BOOST || id == P_BOOST_CMD || id == P_BOOST_ACT)
    v = (v - baroKpa) / 100.0f;       // kPa absolut -> bar fata de atmosfera (ca un ceas de turbo)
  return true;
}

/******************** CODURI DE EROARE - CITIRE ********************/
static void addDtc(Dtc *list, uint8_t &n, uint16_t raw, bool pending) {
  if (raw == 0 || n >= MAX_DTC) return;
  for (uint8_t i = 0; i < n; i++) if (list[i].raw == raw) return;   // fara dubluri
  list[n++] = { raw, pending };
}

// Raspunsul la Mode 03/07, cu headere oprite (ATH0):
//  - linie K (ISO9141/KWP): fiecare rand = "43 AA BB CC DD EE FF" -> 3 coduri, 00 00 = gol
//  - CAN: "43 NN AA BB ..." cu NN = numar coduri; la mai multe coduri raspunsul vine pe
//    mai multe randuri numerotate ("0:", "1:") precedate de lungime ("00A")
static void parseDtcResponse(const String &r, uint8_t modeByte, bool pending, Dtc *list, uint8_t &n) {
  uint8_t bytes[96];
  int nb = 0;
  int i = 0, len = r.length();
  while (i < len && nb < (int)sizeof(bytes)) {
    while (i < len && r[i] == ' ') i++;
    int start = i;
    while (i < len && r[i] != ' ') i++;
    String tok = r.substring(start, i);
    // pastram doar octetii hex de 2 caractere (sarim "0:", "00A", "NO", "DATA" etc.)
    if (tok.length() == 2 && isxdigit(tok[0]) && isxdigit(tok[1]))
      bytes[nb++] = (uint8_t)strtol(tok.c_str(), nullptr, 16);
  }

  for (int k = 0; k < nb; k++) {
    if (bytes[k] != modeByte) continue;
    if (isCanProtocol) {
      if (k + 1 >= nb) break;
      int cnt = bytes[k + 1];
      int j = k + 2;
      for (int c = 0; c < cnt && j + 1 < nb; c++, j += 2) addDtc(list, n, (bytes[j] << 8) | bytes[j + 1], pending);
      k = j - 1;
    } else {
      int j = k + 1;
      for (int c = 0; c < 3 && j + 1 < nb; c++, j += 2) addDtc(list, n, (bytes[j] << 8) | bytes[j + 1], pending);
      k = j - 1;
    }
  }
}

// Mode 03 = erori memorate, Mode 07 = erori in asteptare, 0101 = martor check engine.
// NU stergem niciodata erorile (Mode 04 nu e folosit).
static void readDtcs() {
  Dtc tmp[MAX_DTC];
  uint8_t n = 0;
  parseDtcResponse(elmSendCommand("03", 6000), 0x43, false, tmp, n);
  parseDtcResponse(elmSendCommand("07", 6000), 0x47, true, tmp, n);
  uint8_t b[1];
  if (queryPidBytes("0101", b, 1)) milOn = b[0] & 0x80;

  memcpy(dtcList, tmp, sizeof(Dtc) * n);
  dtcCount = n;
  dtcKnown = true;
  dtcReadAt = millis();

  Serial.printf("[DTC] %u erori, martor %s:", n, milOn ? "APRINS" : "stins");
  char code[8];
  for (uint8_t i = 0; i < n; i++) {
    dtcToStr(dtcList[i].raw, code, sizeof(code));
    Serial.printf(" %s%s", code, dtcList[i].pending ? "(asteptare)" : "");
  }
  Serial.println();

  updateDtcBadge();
  refreshErrorScreen();
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
  create_error_screen();
  set_status("PORNIRE BT...");
  lv_timer_handler();   // forteaza afisarea statusului pe ecran inainte de pasii blocanti de mai jos

  // de acum, o apasare pe BOOT comuta intre date live si lista de erori (vezi checkScreenToggle)

  lastReconnectAttempt = millis() - RECONNECT_INTERVAL_MS;   // conectare imediat in loop()
}

/******************** LOOP ********************/
uint32_t lastEcuRetry = 0;
const uint32_t ECU_RETRY_MS = 15000;
int ecuFailCycles = 0;

// Randul de status de sus (CONECTARE ELM..., LIVE 5/5 etc.) dispare dupa
// LIVE_STATUS_MS de date live si reapare imediat la orice problema.
const uint32_t LIVE_STATUS_MS = 5000;
uint32_t liveSince = 0;   // de cand citim date live (0 = nu citim)

static void statusVisible(bool v) {
  if (v) lv_obj_clear_flag(lbl_status, LV_OBJ_FLAG_HIDDEN);
  else lv_obj_add_flag(lbl_status, LV_OBJ_FLAG_HIDDEN);
}

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

  // BOOT apasat -> comuta ecranul; pe ecranul de erori actualizam "citit acum Xs"
  checkScreenToggle();
  static uint32_t lastErrInfo = 0;
  if (errScreen && millis() - lastErrInfo > 1000) {
    lastErrInfo = millis();
    refreshErrorScreen();
    lv_timer_handler();
  }

  // 1) Bluetooth cazut -> reconectare
  if (!SerialBT.connected()) {
    elmLinkUp = false;
    elmEcuOk = false;
    liveSince = 0;
    statusVisible(true);
    if (millis() - lastReconnectAttempt > RECONNECT_INTERVAL_MS) {
      lastReconnectAttempt = millis();
      set_status_short("CONECTARE ELM...");
      elmLinkUp = connectELM();
      if (!elmLinkUp) {
        set_status_short("ELM327 NEGASIT");
      } else {
        set_status_short("CAUT ECU...");
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
    liveSince = 0;
    statusVisible(true);
    updateBattery();
    if (millis() - lastEcuRetry > ECU_RETRY_MS) {
      lastEcuRetry = millis();
      set_status_short("REINCERC ECU...");
      elmEcuOk = elmFindEcu();
      ecuFailCycles = 0;
    }
    set_status_short(lastEcuMsg);
    for (int i = 0; i < 50; i++) { checkScreenToggle(); delay(10); }   // 500ms, dar cu BOOT activ
    return;
  }

  // 3a) ECU OK + ecranul de erori -> doar erorile, fara date live
  if (errScreen) {
    if (dtcRefreshRequested || !dtcKnown || millis() - dtcReadAt > DTC_INTERVAL_ERR_MS) {
      dtcRefreshRequested = false;
      lv_label_set_text(lbl_err_info, "Citesc erorile...");
      lv_timer_handler();
      readDtcs();
    }
    lv_timer_handler();
    delay(20);
    return;
  }

  // erorile se verifica si in fundal (o data pe minut) pentru indicatorul de sus
  if (!dtcKnown || millis() - dtcReadAt > DTC_INTERVAL_MS) readDtcs();

  // 3b) ECU OK -> date live
  // Valorile care se schimba rapid se citesc la FIECARE ciclu; temperaturile si
  // tensiunea se schimba lent, deci doar la fiecare al 5-lea ciclu -> refresh mai des.
  static uint8_t cycle = 0;
  bool slowCycle = (cycle++ % 5 == 0);
  uint32_t cycleStart = millis();
  int okCount = 0, total = 0;

  // VACUUM / BOOST au nevoie de presiunea atmosferica -> o reimprospatam pe ciclurile lente
  if (slowCycle && pidSupported(0x33)) {
    for (uint8_t ci = 0; ci < activeCount; ci++)
      if (needsBaro(cfgSlots[activeSlot[ci]].param)) { float b; readParam(P_BARO, b); break; }
  }

  for (uint8_t ci = 0; ci < activeCount; ci++) {
    if (checkScreenToggle()) return;      // BOOT apasat in timpul ciclului -> trecem imediat pe erori
    uint8_t id = cfgSlots[activeSlot[ci]].param;
    const ParamDef &p = PARAMS[id];
    if (p.slow && !slowCycle) continue;   // temperaturi, tensiuni etc. - doar la fiecare al 5-lea ciclu
    if (!paramSupported(id)) continue;    // ECU-ul nu il are -> ramane "X", nu pierdem timp
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
      lastEcuMsg = "ECU NU RASPUNDE";
      lastEcuRetry = millis();
    }
  } else {
    ecuFailCycles = 0;
  }
  if (elmEcuOk) {
    if (!liveSince) liveSince = millis();
    if (millis() - liveSince < LIVE_STATUS_MS) {
      statusVisible(true);
      set_status_short("LIVE " + String(okCount) + "/" + String(total));
    } else {
      statusVisible(false);   // totul merge -> ascundem randul de status
      lv_timer_handler();
    }
  } else {
    liveSince = 0;            // ECU pierdut -> statusul reapare imediat
    statusVisible(true);
    set_status_short(lastEcuMsg);
  }
  delay(5);
}
