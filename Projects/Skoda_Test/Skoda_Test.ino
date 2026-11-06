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
bool fsOk = false;           // LittleFS montat (necesar pentru imaginea de start)
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
static void loadSettings() {
  prefs.begin("display", true);
  cfgBg = prefs.getUInt("bg", cfgBg);
  cfgFg = prefs.getUInt("fg", cfgFg);
  prefs.end();
  Serial.printf("[CFG] fundal #%06X, scris #%06X\n", (unsigned)cfgBg, (unsigned)cfgFg);
}

static void saveSettings() {
  prefs.begin("display", false);
  prefs.putUInt("bg", cfgBg);
  prefs.putUInt("fg", cfgFg);
  prefs.end();
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
  lv_obj_set_style_bg_color(scr, lv_color_hex(cfgBg), 0);   // culoare fundal din setari
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  lv_style_init(&style_status);
  lv_style_set_text_color(&style_status, lv_color_hex(cfgFg));
  lv_style_set_text_font(&style_status, &lv_font_montserrat_14);
  lv_style_set_bg_opa(&style_status, LV_OPA_TRANSP);
  lv_style_set_border_width(&style_status, 0);

  lv_style_init(&style_title);
  lv_style_set_text_color(&style_title, lv_color_hex(cfgFg));
  lv_style_set_text_font(&style_title, &lv_font_montserrat_12);
  lv_style_set_bg_opa(&style_title, LV_OPA_TRANSP);
  lv_style_set_border_width(&style_title, 0);

  lv_style_init(&style_value);
  lv_style_set_text_color(&style_value, lv_color_hex(cfgFg));
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
static void drawColorPreview(uint32_t bg, uint32_t fg) {
  tft.fillScreen(rgb565(bg));
  tft.setTextDatum(top_left);
  tft.setTextColor(rgb565(fg));
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.drawString("Previzualizare (nesalvat)", 5, 4);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.drawString("RPM", 5, 40);
  tft.drawString("COOLANT", 165, 40);
  tft.setFont(&fonts::FreeSansBold18pt7b);
  tft.drawString("2500", 5, 62);
  tft.drawString("90C", 165, 62);
  tft.setFont(&fonts::Font2);
  tft.drawString("Apasa Salveaza in pagina ca sa pastrezi culorile", 5, TFT_H - 22);
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
#prev{border-radius:8px;padding:14px;aspect-ratio:4/3;box-sizing:border-box}
#prev .t{font-size:13px}#prev .v{font-size:30px;font-weight:bold;margin-bottom:8px}
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
<div id="prev"><div class="t">RPM</div><div class="v">2500</div><div class="t">COOLANT</div><div class="v">90C</div></div>
<small>Culorile apar imediat și pe display, dar se păstrează doar după „Salvează”.</small>
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
let timer,blob=null;
function body(){return new URLSearchParams({bg:$('bg').value,fg:$('fg').value});}
function upd(send){const p=$('prev');p.style.background=$('bg').value;p.style.color=$('fg').value;
 if(send){clearTimeout(timer);timer=setTimeout(()=>fetch('/preview',{method:'POST',body:body()}),300);}}
['bg','fg'].forEach(k=>{
 $(k).addEventListener('input',()=>{$(k+'h').value=$(k).value;upd(true);});
 $(k+'h').addEventListener('change',()=>{let v=$(k+'h').value.trim();if(v[0]!='#')v='#'+v;
  if(/^#[0-9a-fA-F]{6}$/.test(v)){$(k).value=v.toLowerCase();$(k+'h').value=$(k).value;upd(true);}else $(k+'h').value=$(k).value;});
});
function showCur(){$('cur').src='/splash.jpg?'+Date.now();$('cur').style.display='block';$('del').style.display='block';}
async function load(){try{const c=await (await fetch('/config')).json();
 ['bg','fg'].forEach(k=>{$(k).value=c[k];$(k+'h').value=c[k];});upd(false);if(c.img)showCur();}catch(e){msg('Nu pot citi setările');}}
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
  char json[80];
  bool img = fsOk && LittleFS.exists(SPLASH_PATH);
  snprintf(json, sizeof(json), "{\"bg\":\"#%06x\",\"fg\":\"#%06x\",\"img\":%s}",
           (unsigned)cfgBg, (unsigned)cfgFg, img ? "true" : "false");
  server->send(200, "application/json", json);
}

static void handlePreview() {
  uint32_t bg, fg;
  if (!parseColor(server->arg("bg"), bg) || !parseColor(server->arg("fg"), fg)) {
    server->send(400, "text/plain", "culoare invalida");
    return;
  }
  drawColorPreview(bg, fg);
  server->send(200, "text/plain", "ok");
}

static void handleSave() {
  uint32_t bg, fg;
  if (!parseColor(server->arg("bg"), bg) || !parseColor(server->arg("fg"), fg)) {
    server->send(400, "text/plain", "culoare invalida");
    return;
  }
  cfgBg = bg;
  cfgFg = fg;
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
