/*********************************************************************************
 * TEST PIN BACKLIGHT
 * Trece pe rand prin pinii liberi ai ESP32 si ii pune LOW apoi HIGH cate 3 secunde.
 * Pe ecran scrie mare numarul pinului testat. Daca ecranul se stinge / se
 * intuneca la un pin, acela e pinul de backlight (BL_PIN in Skoda_Test.ino).
 * Daca nu se intampla nimic la niciun pin -> backlight-ul e legat fix la alimentare.
 *********************************************************************************/
#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

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

const int PINS[] = { 5, 18, 19, 21, 22, 23, 26 };

static void show(int pin, const char *state) {
  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextDatum(middle_center);
  tft.setFont(&fonts::FreeSansBold24pt7b);
  tft.drawString(String("PIN ") + pin, 160, 100);
  tft.setFont(&fonts::FreeSans12pt7b);
  tft.drawString(state, 160, 160);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  tft.init();
  tft.setRotation(1);
  Serial.println("\n=== TEST PIN BACKLIGHT ===");
}

void loop() {
  for (int pin : PINS) {
    show(pin, "LOW");
    Serial.printf("PIN %d -> LOW\n", pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delay(3000);

    show(pin, "HIGH");
    Serial.printf("PIN %d -> HIGH\n", pin);
    digitalWrite(pin, HIGH);
    delay(3000);

    pinMode(pin, INPUT);   // il lasam liber inainte de urmatorul
  }

  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextDatum(middle_center);
  tft.setFont(&fonts::FreeSansBold18pt7b);
  tft.drawString("GATA - reiau", 160, 120);
  Serial.println("--- gata, reiau ---");
  delay(3000);
}
