#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <algorithm>
#include <math.h>
#include <MFRC522.h>
#include <OneButton.h>
#include <SPI.h>
#include <Wire.h>

const uint8_t FLOW_SENSOR_PIN = 33;
const unsigned long FLOW_SENSOR_INTERVAL_MS = 1000;
const float FLOW_SENSOR_CALIBRATION_FACTOR = 4.5;

const uint8_t TDS_SENSOR_PIN = 32;
const float TDS_SENSOR_VREF = 3.3;
const float TDS_SENSOR_AMBIENT_TEMP = 25.0;
const int TDS_SENSOR_SAMPLE_COUNT = 30;
const unsigned long TDS_SENSOR_SAMPLE_INTERVAL_MS = 40;
const unsigned long TDS_SENSOR_PRINT_INTERVAL_MS = 800;

const byte RFID_READER_SS_PIN = 5;
const byte RFID_READER_RST_PIN = 0;

const uint8_t VALVE_PIN = 2;

OneButton selectButton(27, true, true);
OneButton upButton(14, true, true);
OneButton downButton(12, true, true);

MFRC522 rfidReader(RFID_READER_SS_PIN, RFID_READER_RST_PIN);

Adafruit_SSD1306 display(128, 64, &Wire);

volatile byte flowSensorCurrPulseCount = 0;

int tdsSensorBuffer[TDS_SENSOR_SAMPLE_COUNT];
int tdsSensorCurrBufferIndex = 0;

bool valveIsOpen = false;
bool valveNeedsUpdate = true;

bool displayContainerFillingStatusNeedsUpdate = false;

float totalFlowMl = 0;

float hasContainer = false;
float currContainerVolumeMl = 0.0;
float toFillContainerVolumeMl = 0.0;

const int16_t CONTAINER_RECT_RADIUS = 3;
const int16_t CONTAINER_START_X = display.width() / 2 + 25;
const int16_t CONTAINER_TEXT_START_X = CONTAINER_START_X - 10;

void IRAM_ATTR onFlowSensorInterrupt()
{
  flowSensorCurrPulseCount++;
}

void onSelectButtonClicked()
{
  if (hasContainer)
  {
    if (valveIsOpen)
    {
      containerStopFilling();
    }
    else
    {
      valveIsOpen = true;
      valveNeedsUpdate = true;
    }
  }
  else
  {
    valveIsOpen = !valveIsOpen;
    valveNeedsUpdate = true;
  }
}

void setup()
{
  setupSerial();

  pinMode(TDS_SENSOR_PIN, INPUT);

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), onFlowSensorInterrupt, FALLING);

  selectButton.attachClick(onSelectButtonClicked);

  setupRfidReader();

  pinMode(VALVE_PIN, OUTPUT);

  setupDisplay();
}

void setupSerial()
{
  Serial.begin(9600);

  while (!Serial)
  {
    delay(100);
  }

  Serial.println(F("Serial initialized"));
}

void setupRfidReader()
{
  SPI.begin();

  rfidReader.PCD_Init();
  rfidReader.PCD_DumpVersionToSerial();

  Serial.println(F("RFID reader initialized"));
}

void setupDisplay()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextWrap(false);

  Serial.println(F("Display initialized"));
}

unsigned long flowSensorPrevTimestamp = millis();

unsigned long tdsSensorPrevSampleTimestamp = millis();
unsigned long tdsSensorPrevPrintTimestamp = millis();

void loop()
{
  downButton.tick();
  upButton.tick();
  selectButton.tick();

  unsigned long currentMs = millis();

  if (currentMs - flowSensorPrevTimestamp > FLOW_SENSOR_INTERVAL_MS)
  {

    byte pulseCount = flowSensorCurrPulseCount;
    flowSensorCurrPulseCount = 0;

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    float flowRate = ((1000.0 / (currentMs - flowSensorPrevTimestamp)) * pulseCount) / FLOW_SENSOR_CALIBRATION_FACTOR;
    flowSensorPrevTimestamp = currentMs;

    // Divide the flow rate in L/min by 60 to get water flow per second, then multiply by 1000 to convert to mL.
    float flowMl = (flowRate / 60) * 1000;
    totalFlowMl += flowMl;

    if (hasContainer)
    {
      currContainerVolumeMl += flowMl;

      if (currContainerVolumeMl > toFillContainerVolumeMl)
      {
        containerStopFilling();
      }

      displayContainerFillingStatusNeedsUpdate = true;
    }

    displayDrawTotalFlow(totalFlowMl / 1000);
  }

  if (currentMs - tdsSensorPrevSampleTimestamp > TDS_SENSOR_SAMPLE_INTERVAL_MS)
  {
    tdsSensorPrevSampleTimestamp = currentMs;

    tdsSensorBuffer[tdsSensorCurrBufferIndex] = analogRead(TDS_SENSOR_PIN);
    tdsSensorCurrBufferIndex++;
    if (tdsSensorCurrBufferIndex == TDS_SENSOR_SAMPLE_COUNT)
    {
      tdsSensorCurrBufferIndex = 0;
    }
  }

  if (currentMs - tdsSensorPrevPrintTimestamp > TDS_SENSOR_PRINT_INTERVAL_MS)
  {
    tdsSensorPrevPrintTimestamp = currentMs;

    float averageVoltage = getMedianNum(tdsSensorBuffer, TDS_SENSOR_SAMPLE_COUNT) * TDS_SENSOR_VREF / 4096.0;

    float compensationCoefficient = 1.0 + 0.02 * (TDS_SENSOR_AMBIENT_TEMP - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    float tdsValue = (133.42 * powf(compensationVoltage, 3) - 255.86 * powf(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;

    displayDrawWaterQuality(tdsValue);
  }

  if (rfidReader.PICC_IsNewCardPresent() && rfidReader.PICC_ReadCardSerial())
  {
    String content = "";
    for (byte i = 0; i < rfidReader.uid.size; i++)
    {
      content += rfidReader.uid.uidByte[i] < 0x10 ? "0" : "";
      content += String(rfidReader.uid.uidByte[i], HEX);
    }

    bool hasScanned;
    if (content == "1d1181b8081080")
    {
      hasScanned = true;
      toFillContainerVolumeMl = 250.0;
    }
    else if (content == "1d1081b8081080")
    {
      hasScanned = true;
      toFillContainerVolumeMl = 500.0;
    }
    else if (content == "1d0f81b8081080")
    {
      hasScanned = true;
      toFillContainerVolumeMl = 1000.0;
    }
    else if (content == "1d0e81b8081080")
    {
      hasScanned = true;
      toFillContainerVolumeMl = 1500.0;
    }
    else if (content == "1d0d81b8081080")
    {
      hasScanned = true;
      toFillContainerVolumeMl = 2000.0;
    }
    else
    {
      hasScanned = false;
      Serial.println(F("Unknown card"));
    }

    if (hasScanned)
    {
      hasContainer = true;
      currContainerVolumeMl = 0.0;
      displayContainerFillingStatusNeedsUpdate = true;

      valveIsOpen = false;
      valveNeedsUpdate = true;
    }

    rfidReader.PICC_HaltA();
    rfidReader.PCD_StopCrypto1();
  }

  if (displayContainerFillingStatusNeedsUpdate && hasContainer)
  {
    displayDrawContainerFillingStatus(currContainerVolumeMl, toFillContainerVolumeMl);
  }

  if (valveNeedsUpdate)
  {
    valveNeedsUpdate = false;
    valveUpdate();

    if (!hasContainer)
    {
      displayDrawValve(valveIsOpen);
    }
  }
}

// Returns the median of the array
int getMedianNum(int array[], int filterLen)
{
  int bTab[filterLen];
  std::copy(array, array + filterLen, bTab);

  std::sort(bTab, bTab + filterLen);

  if (filterLen % 2 == 1)
  {
    return bTab[filterLen / 2];
  }
  else
  {
    return (bTab[filterLen / 2] + bTab[filterLen / 2 - 1]) / 2;
  }
}

void containerStopFilling()
{
  hasContainer = false;
  currContainerVolumeMl = 0.0;
  toFillContainerVolumeMl = 0.0;

  valveIsOpen = false;
  valveNeedsUpdate = true;
}

void displayDrawTotalFlow(float flowValueL)
{
  display.fillRect(0, 0, CONTAINER_TEXT_START_X, 40, BLACK);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TOTAL");

  double pricePerL = 36.24 * (flowValueL / 1000);

  display.setTextSize(2);
  display.setCursor(0, 9);
  display.printf("P%.2f", pricePerL);

  display.setTextSize(1);
  display.setCursor(0, 26);
  display.printf("(%.1fL)", flowValueL);

  display.display();
}

void displayDrawWaterQuality(float tdsValue)
{
  display.fillRect(0, 40, CONTAINER_TEXT_START_X, 64, BLACK);

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.println("QUALITY");

  display.setTextSize(2);
  display.setCursor(0, 49);

  String waterQualityText;
  if (tdsValue < 50)
  {
    waterQualityText = "IDEAL";
  }
  else if (tdsValue < 150)
  {
    waterQualityText = "GOOD";
  }
  else if (tdsValue < 300)
  {
    waterQualityText = "FAIR";
  }
  else
  {
    waterQualityText = "BAD";
  }

  display.println(waterQualityText);
  display.display();
}

void displayDrawValve(bool isOpen)
{
  display.fillRect(CONTAINER_TEXT_START_X, 0, display.width() - CONTAINER_TEXT_START_X, 64, BLACK);

  display.setTextSize(1);
  display.setCursor(CONTAINER_TEXT_START_X + 1, 20);
  display.println("VALVE");

  display.setTextSize(2);
  display.setCursor(CONTAINER_TEXT_START_X + 1, 29);
  display.println(isOpen ? "ON" : "OFF");

  display.display();
}

void displayDrawContainerFillingStatus(float currVolumeMl, float maxVolumeMl)
{
  display.fillRect(CONTAINER_TEXT_START_X, 20, display.width() - CONTAINER_TEXT_START_X, 64, BLACK);

  int16_t rectX = display.width() / 2 + 25;
  int16_t rectY = 10;
  int16_t rectWidth = display.width() / 2 - 40;
  int16_t rectHeight = display.height() - 30;
  display.drawRoundRect(rectX, rectY, rectWidth, rectHeight, CONTAINER_RECT_RADIUS, WHITE);

  int16_t filledRectHeight = map(currVolumeMl, 0, maxVolumeMl, 0, rectHeight);
  display.fillRoundRect(rectX, rectY + rectHeight - filledRectHeight, rectWidth, filledRectHeight, CONTAINER_RECT_RADIUS, WHITE);

  display.setTextSize(1);

  String fillStatusText;
  if (maxVolumeMl == 0.0)
  {
    fillStatusText = "START";
  }
  else if (currVolumeMl < maxVolumeMl)
  {
    fillStatusText = "FILLING";
  }
  else if (currVolumeMl >= maxVolumeMl)
  {
    fillStatusText = "DONE";
  }

  int16_t rectTextX = rectX - 10;
  int16_t fillStatusX, fillStatusY;
  uint16_t fillStatusW, fillStatusH;
  display.getTextBounds(fillStatusText, rectTextX, 0, &fillStatusX, &fillStatusY, &fillStatusW, &fillStatusH);

  display.setCursor(rectTextX, 0);
  display.println(fillStatusText);

  display.setCursor(rectTextX, rectY + rectHeight + 2);
  display.printf("%.1f/", currVolumeMl);
  display.setCursor(rectTextX, rectY + rectHeight + 12);
  display.printf("%.1fmL", maxVolumeMl);

  display.display();
}

void valveUpdate()
{
  if (valveIsOpen)
  {
    digitalWrite(VALVE_PIN, HIGH);
  }
  else
  {
    digitalWrite(VALVE_PIN, LOW);
  }
}
