#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
// STM32F401CCU6 Black Pill - basic hardware test
// Tests ONLY: STM32 + SSD1306 OLED + 2-position DIP + KEY
// Existing PCB connections are kept unchanged.
// ============================================================

#define OLED_SDA PB9
#define OLED_SCL PB8
#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define DIP_ALG0 PB2
#define DIP_ALG1 PB3
#define KEY_PIN PC13

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

const char* algorithmName(uint8_t dip) {
  switch (dip) {
    case 0: return "FLOOD FILL";
    case 1: return "LEFT WALL";
    case 2: return "RIGHT WALL";
    case 3: return "A STAR";
    default: return "UNKNOWN";
  }
}

void showScreen(uint8_t dip, bool keyPressed) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("MICROMOUSE TEST");
  display.println("----------------");

  display.setCursor(0, 20);
  display.print("STM32 : ");
  display.println("OK");

  display.print("OLED  : ");
  display.println("OK");

  display.setCursor(0, 38);
  display.print("DIP   : ");
  display.print((dip >> 1) & 1);
  display.println(dip & 1);

  display.print("ALG   : ");
  display.println(algorithmName(dip));

  display.setCursor(0, 56);
  display.print("KEY   : ");
  display.println(keyPressed ? "PRESSED" : "RELEASED");

  display.display();
}

void setup() {
  // DIP and KEY are wired as active-low inputs.
  pinMode(DIP_ALG0, INPUT_PULLUP);
  pinMode(DIP_ALG1, INPUT_PULLUP);
  pinMode(KEY_PIN, INPUT_PULLUP);

  // Keep the existing PCB I2C pins: PB8=SCL, PB9=SDA.
  Wire.setSCL(OLED_SCL);
  Wire.setSDA(OLED_SDA);
  Wire.begin();
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // OLED failed: blink the onboard PC13 LED/button pin slowly.
    // This is only a failure indication; no other robot hardware is used.
    pinMode(PC13, OUTPUT);
    while (true) {
      digitalWrite(PC13, LOW);
      delay(250);
      digitalWrite(PC13, HIGH);
      delay(250);
    }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(8, 8);
  display.println("STM32 OK");
  display.setTextSize(1);
  display.setCursor(8, 35);
  display.println("OLED + DIP TEST");
  display.display();
  delay(1000);
}

void loop() {
  // Active-low DIP switches.
  uint8_t sw0 = (digitalRead(DIP_ALG0) == LOW) ? 1 : 0;
  uint8_t sw1 = (digitalRead(DIP_ALG1) == LOW) ? 1 : 0;

  // DIP mapping:
  // OFF/OFF = 00 = Flood Fill
  // ON/OFF  = 01 = Left Wall
  // OFF/ON  = 10 = Right Wall
  // ON/ON   = 11 = A*
  uint8_t dip = (sw1 << 1) | sw0;

  bool keyPressed = (digitalRead(KEY_PIN) == LOW);

  showScreen(dip, keyPressed);
  delay(100);
}
