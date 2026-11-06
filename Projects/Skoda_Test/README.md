# Skoda Test — Fabia Mk2 (2008) 1.2 12V BZG — ELM327 Bluetooth

Sketch de test pentru verificarea conexiunii ESP32 <-> ELM327 (Bluetooth Classic
SPP) direct pe mașina ta, folosind exclusiv PID-uri OBD-II standard (Mode 01).

Spre deosebire de proiectul BMW din acest repo (care citește CAN-ul direct,
pentru date proprietare precum presiune/temperatură ulei), aici nu avem acces
la ID-uri CAN proprietare Skoda/VAG, deci testăm doar ce oferă un ELM327
"din cutie": PID-uri standardizate, suportate de orice mașină EOBD.

## De ce aceste PID-uri și nu altele

Motorul **BZG** (1.2 12V, 3 cilindri, familia EA111) folosește un senzor
**MAP** (speed-density), nu MAF. De asta sketch-ul citește PID `010B` (MAP,
kPa) în loc de `0x10` (MAF), care la acest motor ar întoarce date invalide
sau "NO DATA".

PID-uri interogate:

| PID    | Ce citește          | Formulă                          |
|--------|----------------------|-----------------------------------|
| `010C` | Turație motor (RPM)  | `((A*256)+B)/4`                   |
| `0105` | Temp. lichid răcire  | `A-40` °C                         |
| `010D` | Viteză               | `A` km/h                          |
| `0104` | Sarcină motor calc.  | `A*100/255` %                     |
| `010F` | Temp. aer admisie    | `A-40` °C                         |
| `010B` | Presiune galerie (MAP)| `A` kPa                          |
| `0111` | Poziție clapetă (TPS)| `A*100/255` %                     |
| `ATRV` | Tensiune baterie     | citită direct de ELM327 de pe pin 16 OBD2, nu trece prin ECU |

`ATRV` e util ca prim test: răspunde chiar dacă ECU-ul nu comunică încă
(motorul oprit, contact abia pus pe ON), pentru că adaptorul citește
tensiunea direct de pe portul OBD2.

## Pași de test

1. **Pe birou, fără mașină** (opțional, dar recomandat primul pas):
   dacă adaptorul ELM327 se alimentează separat (powerbank pe USB, unele
   clone au și mufă micro-USB), poți testa doar împerecherea Bluetooth și
   comenzile `AT*`. PID-urile OBD (`01xx`) vor răspunde `NO DATA` fără ECU
   conectat — e normal, înseamnă că legătura Bluetooth funcționează.

2. **Găsește numele/adresa adaptorului**: pe telefon, Setări > Bluetooth >
   pairing manual cu adaptorul. Cele mai multe clone ieftine apar ca
   `OBDII` cu PIN `1234` (uneori `0000` sau `6789`). Dacă numele diferă,
   modifică `ELM_DEVICE_NAME` din [Skoda_Test.ino](Skoda_Test.ino). Dacă
   nici pe nume nu se conectează fiabil, notează adresa MAC din telefon,
   completeaz-o în `ELM_MAC[]` și pune `USE_MAC_ADDRESS` pe `true`.

3. **În mașină**: contact pe poziția **ON** (nu neapărat motorul pornit —
   ELM327 are nevoie doar de +12V de pe OBD2 ca să pornească). ESP32 se
   conectează automat, rulează secvența de inițializare (`ATZ`, `ATE0`,
   `ATSP0` = auto-detect protocol) și începe interogarea PID-urilor.

4. **Serial Monitor la 115200 baud** — tot ce trimite/primește ELM327 e
   logat acolo (`[ELM] ...`), util pentru diagnoză dacă un PID nu merge.

## Probleme cunoscute la clonele ieftine ELM327

- Firmware-urile v1.5 "clonă" (nu original ELM327 chip) au bug-uri
  cunoscute: checksum greșit pe unele răspunsuri, timeout-uri, sau
  protocol mismatch la `ATSP0` (auto-detect). Dacă auto-detect nu merge,
  încearcă manual în cod protocolul: `ATSP5` (ISO 14230-4 KWP fast init)
  sau `ATSP3` (ISO 9141-2) — mașinile pre-2008 sunt uneori pe K-Line în
  loc de CAN. Fabia Mk2 din 2008 e la limită — poate fi oricare din cele
  două, de asta `ATSP0` (auto) e alegerea implicită.
- Bluetooth Classic (SPP) e obligatoriu — dacă ai la îndemână o placă
  ESP32-S3/C3/C6, acestea au DOAR BLE, nu Bluetooth Classic, și acest
  sketch nu va compila/funcționa pe ele. Trebuie ESP32 "clasic"
  (WROOM-32/WROVER, Xtensa LX6) — aceeași placă folosită și în restul
  proiectului.

## Hardware

Identic cu wiring-ul din `Factory_samples_without_touch`: TFT ST7789
240x320, bus paralel 8-bit, aceiași pini. Nu e nevoie de tranceiver CAN
sau placă a doua — un singur ESP32, împerecheat direct cu adaptorul
ELM327 din portul OBD2.

## Mod setări (din telefon, fără aplicație)

Din telefon (Android sau iPhone) poți schimba culoarea fundalului, culoarea
scrisului și imaginea de start.

1. Alimentează placa. Cât timp e afișată imaginea de start (~3 s), ține
   apăsat **BOOT** ~1 secundă. Nu ține BOOT apăsat în momentul alimentării,
   pentru că atunci placa intră în modul de programare.
2. Conectează telefonul la WiFi **`Skoda_Display`**, parola **`12345678`**.
   Pagina se deschide singură. Dacă nu, intră pe `http://192.168.4.1`.
3. Alegi setările. Culorile se văd imediat pe display.
   Apeși **Salvează și repornește**.

În modul setări Bluetooth-ul e oprit, deci nu se citesc date de la mașină.
Ieși apăsând **Salvează**, apăsând **BOOT** (fără salvare) sau automat, după
10 minute fără niciun telefon conectat.

- **Partition Scheme** trebuie să fie *Huge APP (3MB No OTA/1MB SPIFFS)*.
  Imaginea de start se salvează în partiția SPIFFS.
- **Luminozitate**: nu se poate regla din software. Pe această placă
  backlight-ul e legat fix la alimentare. Am verificat cu
  `Projects/Backlight_Test`: niciun pin liber nu îl controlează.

## Librării necesare

- LovyanGFX
- lvgl v8.x — folosește `lv_conf.h` din `Projects/LVGL configuration file/`
  (se pune în folderul librăriilor Arduino, lângă folderul `lvgl`, nu în
  folderul sketch-ului — la fel ca la celelalte proiecte din acest repo)
- `BluetoothSerial` — inclusă în ESP32 core, nu necesită instalare
