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
const float FLOW_SENSOR_CALIBRATION_FACTOR = 4.5; // TODO Do proper calibration

const uint8_t TDS_SENSOR_PIN = 32;
const float TDS_SENSOR_VREF = 3.3;
const float TDS_SENSOR_AMBIENT_TEMP = 25.0;
const int TDS_SENSOR_SAMPLE_COUNT = 30;
const unsigned long TDS_SENSOR_SAMPLE_INTERVAL_MS = 40;
const unsigned long TDS_SENSOR_PRINT_INTERVAL_MS = 800;

const byte RFID_READER_SS_PIN = 5;
const byte RFID_READER_RST_PIN = 0;

const uint8_t VALVE_PIN = 2; // FIXME Change to proper relay module pin

Adafruit_SSD1306 display(128, 64, &Wire);

const int16_t CONTAINER_RECT_RADIUS = 3;
const int16_t CONTAINER_START_X = display.width() / 2 + 25;
const int16_t CONTAINER_START_Y = 10;
const int16_t CONTAINER_WIDTH = display.width() / 2 - 40;
const int16_t CONTAINER_HEIGHT = display.height() - 30;
const int16_t CONTAINER_TEXT_START_X = CONTAINER_START_X - 10;

const float TO_FILL_CONTAINER_VOLUME_ADJUST_STEP = 0.1;

OneButton selectButton(27, true, true);
OneButton upButton(14, true, true);
OneButton downButton(12, true, true);

MFRC522 rfidReader(RFID_READER_SS_PIN, RFID_READER_RST_PIN);

volatile byte flowSensorCurrPulseCount = 0;

int tdsSensorBuffer[TDS_SENSOR_SAMPLE_COUNT];
int tdsSensorCurrBufferIndex = 0;

bool valveIsOpen = false;
bool valveNeedsUpdate = true;

bool displayContainerFillingStatusNeedsUpdate = false;

float totalFlowMl = 0;

bool hasContainer = false;
float currContainerVolumeMl = 0.0;
float toFillContainerVolumeMl = 0.0;
float maxContainerVolumeMl = 0.0;

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

void onSelectButtonDoubleClicked()
{
  if (hasContainer && !valveIsOpen)
  {
    containerClearFilling();
  }
}

void onUpButtonClicked()
{
  if (hasContainer && !valveIsOpen)
  {
    float delta = maxContainerVolumeMl * TO_FILL_CONTAINER_VOLUME_ADJUST_STEP;

    if (delta + toFillContainerVolumeMl <= maxContainerVolumeMl)
    {
      toFillContainerVolumeMl += delta;
      displayContainerFillingStatusNeedsUpdate = true;
    }
  }
}

void onDownButtonClicked()
{
  if (hasContainer && !valveIsOpen)
  {
    float delta = maxContainerVolumeMl * TO_FILL_CONTAINER_VOLUME_ADJUST_STEP;

    if (toFillContainerVolumeMl - delta >= 0)
    {
      toFillContainerVolumeMl -= delta;
      displayContainerFillingStatusNeedsUpdate = true;
    }
  }
}

void setup()
{
  setupSerial();

  pinMode(TDS_SENSOR_PIN, INPUT);

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), onFlowSensorInterrupt, FALLING);

  selectButton.attachClick(onSelectButtonClicked);
  selectButton.attachDoubleClick(onSelectButtonDoubleClicked);
  upButton.attachClick(onUpButtonClicked);
  downButton.attachClick(onDownButtonClicked);

  setupRfidReader();

  pinMode(VALVE_PIN, OUTPUT);

  setupDisplay();
}

void setupSerial()
{
  Serial.begin(9600);

  while (!Serial)
    ;

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
  bool displayNeedsUpdate = false;

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

    float totalFlowL = totalFlowMl / 1000.0;
    displayDrawTotalFlow(totalFlowL);
    displayNeedsUpdate = true;
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

    float averageVoltage = getArrayMedian(tdsSensorBuffer, TDS_SENSOR_SAMPLE_COUNT) * TDS_SENSOR_VREF / 4096.0;

    float compensationCoefficient = 1.0 + 0.02 * (TDS_SENSOR_AMBIENT_TEMP - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    float tdsValue = (133.42 * powf(compensationVoltage, 3) - 255.86 * powf(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;

    displayDrawWaterQuality(tdsValue);
    displayNeedsUpdate = true;
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
      maxContainerVolumeMl = 250.0;
    }
    else if (content == "1d1081b8081080")
    {
      hasScanned = true;
      maxContainerVolumeMl = 500.0;
    }
    else if (content == "1d0f81b8081080")
    {
      hasScanned = true;
      maxContainerVolumeMl = 1000.0;
    }
    else if (content == "1d0e81b8081080")
    {
      hasScanned = true;
      maxContainerVolumeMl = 1500.0;
    }
    else if (content == "1d0d81b8081080")
    {
      hasScanned = true;
      maxContainerVolumeMl = 2000.0;
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
      toFillContainerVolumeMl = maxContainerVolumeMl;
      displayContainerFillingStatusNeedsUpdate = true;

      valveIsOpen = false;
      valveNeedsUpdate = true;
    }

    rfidReader.PICC_HaltA();
    rfidReader.PCD_StopCrypto1();
  }

  if ((valveNeedsUpdate || displayContainerFillingStatusNeedsUpdate) && hasContainer)
  {
    displayContainerFillingStatusNeedsUpdate = false;
    displayDrawContainerFillingStatus(currContainerVolumeMl, toFillContainerVolumeMl, maxContainerVolumeMl, valveIsOpen);
    displayNeedsUpdate = true;
  }

  if (valveNeedsUpdate)
  {
    valveNeedsUpdate = false;
    valveUpdate(valveIsOpen);

    if (!hasContainer)
    {
      displayDrawValve(valveIsOpen);
      displayNeedsUpdate = true;
    }
  }

  if (displayNeedsUpdate)
  {
    displayNeedsUpdate = false;
    display.display();
  }
}

// Returns the median of the array
int getArrayMedian(int array[], int filterLen)
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

void containerClearFilling()
{
  hasContainer = false;
  currContainerVolumeMl = 0.0;
  toFillContainerVolumeMl = 0.0;
  maxContainerVolumeMl = 0.0;
  displayContainerFillingStatusNeedsUpdate = true;
}

void containerStopFilling()
{
  containerClearFilling();

  valveIsOpen = false;
  valveNeedsUpdate = true;
}

void displayDrawTotalFlow(float totalFlowL)
{
  display.fillRect(0, 0, CONTAINER_TEXT_START_X, 40, BLACK);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TOTAL");

  double totalFlowCubicM = totalFlowL / 1000.0;
  double totalPrice = 36.24 * totalFlowCubicM;

  display.setTextSize(2);
  display.setCursor(0, 9);
  display.printf("P%.2f", totalPrice);

  display.setTextSize(1);
  display.setCursor(0, 26);
  display.printf("(%.1fL)", totalFlowL);
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
}

void displayDrawContainerFillingStatus(float currVolumeMl, float toFillVolumeMl, float maxVolumeMl, float isFilling)
{
  display.fillRect(CONTAINER_TEXT_START_X, 0, display.width() - CONTAINER_TEXT_START_X, 64, BLACK);

  display.drawRoundRect(CONTAINER_START_X, CONTAINER_START_Y, CONTAINER_WIDTH, CONTAINER_HEIGHT, CONTAINER_RECT_RADIUS, WHITE);

  int16_t filledRectHeight;
  if (isFilling)
  {
    const int16_t ARROW_OFFSET = 3;
    int16_t arrowY = CONTAINER_START_Y + CONTAINER_HEIGHT - map(toFillVolumeMl, 0, maxVolumeMl, 0, CONTAINER_HEIGHT);

    int16_t arrowLeftX = CONTAINER_START_X - ARROW_OFFSET;
    display.fillTriangle(arrowLeftX, arrowY - ARROW_OFFSET, arrowLeftX, arrowY + ARROW_OFFSET, arrowLeftX + ARROW_OFFSET, arrowY, WHITE);

    int16_t arrowRightX = CONTAINER_START_X + CONTAINER_WIDTH + ARROW_OFFSET;
    display.fillTriangle(arrowRightX, arrowY - ARROW_OFFSET, arrowRightX, arrowY + ARROW_OFFSET, arrowRightX - ARROW_OFFSET, arrowY, WHITE);

    filledRectHeight = map(currVolumeMl, 0, maxVolumeMl, 0, CONTAINER_HEIGHT);
  }
  else
  {
    filledRectHeight = map(toFillVolumeMl, 0, maxVolumeMl, 0, CONTAINER_HEIGHT);
  }
  display.fillRoundRect(CONTAINER_START_X, CONTAINER_START_Y + CONTAINER_HEIGHT - filledRectHeight, CONTAINER_WIDTH, filledRectHeight, CONTAINER_RECT_RADIUS, WHITE);

  display.setTextSize(1);

  String fillStatusText;
  if (!isFilling)
  {
    fillStatusText = "START";
  }
  else if (currVolumeMl < toFillVolumeMl)
  {
    fillStatusText = "FILLING";
  }
  else if (currVolumeMl >= toFillVolumeMl)
  {
    fillStatusText = "DONE";
  }

  int16_t fillStatusX, fillStatusY;
  uint16_t fillStatusW, fillStatusH;
  display.getTextBounds(fillStatusText, CONTAINER_TEXT_START_X, 0, &fillStatusX, &fillStatusY, &fillStatusW, &fillStatusH);

  display.setCursor(CONTAINER_TEXT_START_X, 0);
  display.println(fillStatusText);

  if (isFilling)
  {
    display.setCursor(CONTAINER_TEXT_START_X, CONTAINER_START_Y + CONTAINER_HEIGHT + 2);
    display.printf("%.1f/", currVolumeMl);

    display.setCursor(CONTAINER_TEXT_START_X, CONTAINER_START_Y + CONTAINER_HEIGHT + 12);
    display.printf("%.1fmL", toFillVolumeMl);
  }
  else
  {
    display.setCursor(CONTAINER_TEXT_START_X, CONTAINER_START_Y + CONTAINER_HEIGHT + 2);
    display.printf("%.1f/", toFillVolumeMl);

    display.setCursor(CONTAINER_TEXT_START_X, CONTAINER_START_Y + CONTAINER_HEIGHT + 12);
    display.printf("%.1fmL", maxVolumeMl);
  }
}

void valveUpdate(bool isOpen)
{
  if (isOpen)
  {
    digitalWrite(VALVE_PIN, HIGH);
  }
  else
  {
    digitalWrite(VALVE_PIN, LOW);
  }
}
