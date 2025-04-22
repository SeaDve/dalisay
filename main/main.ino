#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

#define REC_RADIUS 3

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup()
{
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  display.clearDisplay();
  display.setTextColor(WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TOTAL");

  display.setTextSize(2);
  display.setCursor(0, 9);
  display.println("P23.75");

  display.setTextSize(1);
  display.setCursor(0, 26);
  display.println("(31mL)");

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.println("QUALITY");

  display.setTextSize(2);
  display.setCursor(0, 49);
  display.println("OK");

  int16_t recX = display.width() / 2 + 20;
  int16_t recY = 10;
  int16_t recWidth = display.width() / 2 - 30;
  int16_t recHeight = display.height() - 20;
  display.drawRoundRect(recX, recY, recWidth, recHeight, REC_RADIUS, WHITE);

  display.setTextSize(1);

  int16_t recTextX = recX - 5;
  String fillStatusText = "FILLING";
  int16_t fillStatusX, fillStatusY;
  uint16_t fillStatusW, fillStatusH;
  display.getTextBounds(fillStatusText, recTextX, 0, &fillStatusX, &fillStatusY, &fillStatusW, &fillStatusH);

  display.setCursor(recTextX, 0);
  display.println(fillStatusText);

  display.setCursor(recTextX, recY + recHeight + 2);
  display.println("31/50mL");
  display.display();

  for (int i = 0; i < recHeight; i++)
  {
    display.fillRoundRect(recX, recY + recHeight - i, recWidth, i, REC_RADIUS, WHITE);
    display.display();
    delay(200);
  }

  display.fillRect(fillStatusX, fillStatusY, fillStatusW, fillStatusH, BLACK);
  display.setCursor(recX, 0);
  display.println("FILLED");
  display.display();
}

void loop()
{
}
