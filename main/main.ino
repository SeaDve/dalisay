#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <algorithm>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <math.h>
#include <MFRC522.h>
#include <OneButton.h>
#include <SPI.h>
#include <Wire.h>

const uint8_t FLOW_SENSOR_PIN = 33;
const unsigned long FLOW_SENSOR_UPDATE_INTERVAL_MS = 1000;
const float FLOW_SENSOR_CALIBRATION_FACTOR = 4.5; // TODO Do proper calibration

const uint8_t TDS_SENSOR_PIN = 32;
const float TDS_SENSOR_VREF = 3.3;
const float TDS_SENSOR_AMBIENT_TEMP = 25.0;
const int TDS_SENSOR_SAMPLE_COUNT = 30;
const unsigned long TDS_SENSOR_SAMPLE_INTERVAL_MS = 40;
const unsigned long TDS_SENSOR_UPDATE_INTERVAL_MS = 800;

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

const char *WIFI_SSID = "HUAWEI-2.4G-E75z";
const char *WIFI_PASSWORD = "JgY5wBGt";

const char *WS_DATA_FLOW_RATE = "flowRate";
const char *WS_DATA_TOTAL_FLOW_ML = "totalFlowMl";
const char *WS_DATA_TDS_VALUE = "tdsValue";
const char *WS_DATA_VALVE_IS_OPEN = "valveIsOpen";
const char *WS_DATA_FILLED_CONTAINER_VOLUME_ML = "filledContainerVolumeMl";
const char *WS_DATA_TO_FILL_CONTAINER_VOLUME_ML = "toFillContainerVolumeMl";
const char *WS_DATA_MAX_CONTAINER_VOLUME_ML = "maxContainerVolumeMl";
const char *WS_DATA_COST_PER_CUBIC_M = "costPerCubicM";

const float TO_FILL_CONTAINER_VOLUME_ADJUST_STEP = 0.1;

OneButton selectButton(27, true, true);
OneButton upButton(14, true, true);
OneButton downButton(12, true, true);

MFRC522 rfidReader(RFID_READER_SS_PIN, RFID_READER_RST_PIN);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile byte flowSensorCurrPulseCount = 0;

int tdsSensorBuffer[TDS_SENSOR_SAMPLE_COUNT];
int tdsSensorCurrBufferIndex = 0;

float flowRate = 0.0;
float totalFlowMl = 0.0;

float tdsValue = 0.0;

bool valveIsOpen = false;
bool valveNeedsUpdate = true;

bool displayTotalFlowNeedsUpdate = false;
bool displayNeedsUpdate = false;

float filledContainerVolumeMl = 0.0;
float toFillContainerVolumeMl = 0.0;
float maxContainerVolumeMl = 0.0;

bool filledContainerVolumeNeedsUpdate = false;
bool toFillContainerVolumeNeedsUpdate = false;
bool maxContainerVolumeNeedsUpdate = false;

float costPerCubicM = 36.24;

void IRAM_ATTR onFlowSensorInterrupt()
{
  flowSensorCurrPulseCount++;
}

void onSelectButtonClicked()
{
  if (maxContainerVolumeMl > 0.0)
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
  if (maxContainerVolumeMl > 0.0 && !valveIsOpen)
  {
    containerClearFilling();
  }
}

void onUpButtonClicked()
{
  if (maxContainerVolumeMl > 0.0 && !valveIsOpen)
  {
    float delta = maxContainerVolumeMl * TO_FILL_CONTAINER_VOLUME_ADJUST_STEP;

    if (delta + toFillContainerVolumeMl <= maxContainerVolumeMl)
    {
      toFillContainerVolumeMl += delta;
      toFillContainerVolumeNeedsUpdate = true;
    }
  }
}

void onDownButtonClicked()
{
  if (maxContainerVolumeMl > 0.0 && !valveIsOpen)
  {
    float delta = maxContainerVolumeMl * TO_FILL_CONTAINER_VOLUME_ADJUST_STEP;

    if (toFillContainerVolumeMl - delta >= 0.0)
    {
      toFillContainerVolumeMl -= delta;
      toFillContainerVolumeNeedsUpdate = true;
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

  setupServer();
}

void setupSerial()
{
  Serial.begin(115200);

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

  displayDrawTotalFlow(totalFlowMl, 0.0);
  displayDrawWaterQuality(tdsValue);
  displayDrawValve(valveIsOpen);
  displayDrawContainerFillingStatus(filledContainerVolumeMl, toFillContainerVolumeMl, maxContainerVolumeMl, valveIsOpen);
  displayNeedsUpdate = true;

  Serial.println(F("Display initialized"));
}

void onGetTotalFlowMl(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", String(totalFlowMl));
}

void onGetTdsValue(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", String(tdsValue));
}

void onGetValveIsOpen(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", String(valveIsOpen));
}

void onGetFlowRate(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", String(flowRate));
}

void onGetFilledContainerVolumeMl(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", String(filledContainerVolumeMl));
}

void onGetToFillContainerVolumeMl(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", String(toFillContainerVolumeMl));
}

void onGetMaxContainerVolumeMl(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", String(maxContainerVolumeMl));
}

void onServerGetCostPerCubicM(AsyncWebServerRequest *request)
{
  request->send(200, "text/plain", String(costPerCubicM));
}

void onServerSetCostPerCubicM(AsyncWebServerRequest *request)
{
  if (request->hasParam("value"))
  {
    float prevCostPerCubicM = costPerCubicM;

    String cost = request->getParam("value")->value();
    costPerCubicM = cost.toFloat();

    if (abs(costPerCubicM - prevCostPerCubicM) > __FLT_EPSILON__)
    {
      wsSend(WS_DATA_COST_PER_CUBIC_M, String(costPerCubicM));

      displayTotalFlowNeedsUpdate = true;
    }
  }
  request->send(200, "text/plain", "OK");
}

void onWsEvent(AsyncWebSocket *ws, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

void setupServer()
{
  WiFi.mode(WIFI_AP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi ..");

  unsigned long prevMs = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    unsigned long currMs = millis();
    if (currMs - prevMs >= 1000)
    {
      Serial.print('.');
      prevMs = currMs;
    }
  }

  Serial.println(WiFi.localIP());

  server.on("/getFlowRate", HTTP_GET, onGetFlowRate);
  server.on("/getTotalFlowMl", HTTP_GET, onGetTotalFlowMl);
  server.on("/getTdsValue", HTTP_GET, onGetTdsValue);
  server.on("/getValveIsOpen", HTTP_GET, onGetValveIsOpen);
  server.on("/getFilledContainerVolumeMl", HTTP_GET, onGetFilledContainerVolumeMl);
  server.on("/getToFillContainerVolumeMl", HTTP_GET, onGetToFillContainerVolumeMl);
  server.on("/getMaxContainerVolumeMl", HTTP_GET, onGetMaxContainerVolumeMl);
  server.on("/getCostPerCubicM", HTTP_GET, onServerGetCostPerCubicM);
  server.on("/setCostPerCubicM", HTTP_PUT, onServerSetCostPerCubicM);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.begin();
}

unsigned long flowSensorPrevTimestamp = millis();

unsigned long tdsSensorPrevSampleTimestamp = millis();
unsigned long tdsSensorPrevPrintTimestamp = millis();

float prevFlowRate = 0.0;
float prevTotalFlowMl = 0.0;
float prevTdsValue = 0.0;
float prevFilledContainerVolumeMl = 0.0;

void loop()
{
  ws.cleanupClients();

  downButton.tick();
  upButton.tick();
  selectButton.tick();

  unsigned long currMs = millis();

  if (currMs - flowSensorPrevTimestamp > FLOW_SENSOR_UPDATE_INTERVAL_MS)
  {

    byte pulseCount = flowSensorCurrPulseCount;
    flowSensorCurrPulseCount = 0;

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (currMs - flowSensorPrevTimestamp)) * pulseCount) / FLOW_SENSOR_CALIBRATION_FACTOR;
    flowSensorPrevTimestamp = currMs;

    // Divide the flow rate in L/min by 60 to get water flow per second, then multiply by 1000 to convert to mL.
    float flowMl = (flowRate / 60) * 1000;
    totalFlowMl += flowMl;

    if (maxContainerVolumeMl > 0.0)
    {
      filledContainerVolumeMl += flowMl;

      if (abs(filledContainerVolumeMl - prevFilledContainerVolumeMl) > __FLT_EPSILON__)
      {
        if (filledContainerVolumeMl > toFillContainerVolumeMl)
        {
          containerStopFilling();
        }

        filledContainerVolumeNeedsUpdate = true;
      }
    }

    if (abs(totalFlowMl - prevTotalFlowMl) > __FLT_EPSILON__)
    {
      displayTotalFlowNeedsUpdate = true;

      wsSend(WS_DATA_TOTAL_FLOW_ML, String(totalFlowMl));

      prevTotalFlowMl = totalFlowMl;
    }

    if (abs(flowRate - prevFlowRate) > __FLT_EPSILON__)
    {
      wsSend(WS_DATA_FLOW_RATE, String(flowRate));

      prevFlowRate = flowRate;
    }
  }

  if (currMs - tdsSensorPrevSampleTimestamp > TDS_SENSOR_SAMPLE_INTERVAL_MS)
  {
    tdsSensorPrevSampleTimestamp = currMs;

    tdsSensorBuffer[tdsSensorCurrBufferIndex] = analogRead(TDS_SENSOR_PIN);
    tdsSensorCurrBufferIndex++;
    if (tdsSensorCurrBufferIndex == TDS_SENSOR_SAMPLE_COUNT)
    {
      tdsSensorCurrBufferIndex = 0;
    }
  }

  if (currMs - tdsSensorPrevPrintTimestamp > TDS_SENSOR_UPDATE_INTERVAL_MS)
  {
    tdsSensorPrevPrintTimestamp = currMs;

    float averageVoltage = getArrayMedian(tdsSensorBuffer, TDS_SENSOR_SAMPLE_COUNT) * TDS_SENSOR_VREF / 4096.0;

    float compensationCoefficient = 1.0 + 0.02 * (TDS_SENSOR_AMBIENT_TEMP - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    tdsValue = (133.42 * powf(compensationVoltage, 3) - 255.86 * powf(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;

    if (abs(tdsValue - prevTdsValue) > __FLT_EPSILON__)
    {
      wsSend(WS_DATA_TDS_VALUE, String(tdsValue));

      displayDrawWaterQuality(tdsValue);
      displayNeedsUpdate = true;

      prevTdsValue = tdsValue;
    }
  }

  if (rfidReader.PICC_IsNewCardPresent() && rfidReader.PICC_ReadCardSerial())
  {
    maxContainerVolumeMl = getContainerVolume(rfidReader.uid.uidByte, rfidReader.uid.size);

    if (maxContainerVolumeMl > 0.0)
    {
      filledContainerVolumeMl = 0.0;
      toFillContainerVolumeMl = maxContainerVolumeMl;

      filledContainerVolumeNeedsUpdate = true;
      toFillContainerVolumeNeedsUpdate = true;
      maxContainerVolumeNeedsUpdate = true;

      valveIsOpen = false;
      valveNeedsUpdate = true;
    }
    else
    {
      maxContainerVolumeMl = 0.0;

      Serial.println(F("Unknown card"));
    }

    rfidReader.PICC_HaltA();
    rfidReader.PCD_StopCrypto1();
  }

  if (displayTotalFlowNeedsUpdate)
  {
    float totalFlowCubicM = totalFlowMl / 1000000.0;
    displayDrawTotalFlow(totalFlowMl, totalFlowCubicM * costPerCubicM);
    displayNeedsUpdate = true;

    displayTotalFlowNeedsUpdate = false;
  }

  if (filledContainerVolumeNeedsUpdate || toFillContainerVolumeNeedsUpdate || maxContainerVolumeNeedsUpdate || valveNeedsUpdate)
  {
    if (maxContainerVolumeMl > 0.0)
    {
      displayDrawContainerFillingStatus(filledContainerVolumeMl, toFillContainerVolumeMl, maxContainerVolumeMl, valveIsOpen);
      displayNeedsUpdate = true;
    }
    else
    {
      displayDrawValve(valveIsOpen);
      displayNeedsUpdate = true;
    }
  }

  if (filledContainerVolumeNeedsUpdate)
  {
    wsSend(WS_DATA_FILLED_CONTAINER_VOLUME_ML, String(filledContainerVolumeMl));
    filledContainerVolumeNeedsUpdate = false;
  }

  if (toFillContainerVolumeNeedsUpdate)
  {
    wsSend(WS_DATA_TO_FILL_CONTAINER_VOLUME_ML, String(toFillContainerVolumeMl));
    toFillContainerVolumeNeedsUpdate = false;
  }

  if (maxContainerVolumeNeedsUpdate)
  {
    wsSend(WS_DATA_MAX_CONTAINER_VOLUME_ML, String(maxContainerVolumeMl));
    maxContainerVolumeNeedsUpdate = false;
  }

  if (valveNeedsUpdate)
  {
    valveUpdate(valveIsOpen);
    wsSend(WS_DATA_VALVE_IS_OPEN, String(valveIsOpen));
    valveNeedsUpdate = false;
  }

  if (displayNeedsUpdate)
  {
    display.display();
    displayNeedsUpdate = false;
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

float getContainerVolume(byte *uid, size_t uidSize)
{
  const byte knownUIDs[][10] = {
      {0x1D, 0x11, 0x81, 0xB8, 0x08, 0x10, 0x80, 0x00, 0x00, 0x00},
      {0x1D, 0x10, 0x81, 0xB8, 0x08, 0x10, 0x80, 0x00, 0x00, 0x00},
      {0x1D, 0x0F, 0x81, 0xB8, 0x08, 0x10, 0x80, 0x00, 0x00, 0x00},
      {0x1D, 0x0E, 0x81, 0xB8, 0x08, 0x10, 0x80, 0x00, 0x00, 0x00},
      {0x1D, 0x0D, 0x81, 0xB8, 0x08, 0x10, 0x80, 0x00, 0x00, 0x00},
  };
  const float volumes[] = {250.0, 500.0, 1000.0, 1500.0, 2000.0};

  size_t uidCount = sizeof(knownUIDs) / sizeof(knownUIDs[0]);

  for (size_t i = 0; i < uidCount; i++)
  {
    if (memcmp(uid, knownUIDs[i], uidSize) == 0)
    {
      return volumes[i];
    }
  }

  return -1;
}

void wsSend(const char *dataType, String data)
{
  ws.printfAll("%s: %s", dataType, data.c_str());
}

void containerClearFilling()
{
  filledContainerVolumeMl = 0.0;
  toFillContainerVolumeMl = 0.0;
  maxContainerVolumeMl = 0.0;

  filledContainerVolumeNeedsUpdate = true;
  toFillContainerVolumeNeedsUpdate = true;
  maxContainerVolumeNeedsUpdate = true;
}

void containerStopFilling()
{
  containerClearFilling();

  valveIsOpen = false;
  valveNeedsUpdate = true;
}

void displayDrawTotalFlow(float totalMl, float totalCost)
{
  display.fillRect(0, 0, CONTAINER_TEXT_START_X, 40, BLACK);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("TOTAL");

  display.setTextSize(2);
  display.setCursor(0, 9);
  display.printf("P%.2f", totalCost);

  display.setTextSize(1);
  display.setCursor(0, 26);
  display.printf("(%.1fL)", totalMl / 1000.0);
}

void displayDrawWaterQuality(float tdsValue)
{
  display.fillRect(0, 40, CONTAINER_TEXT_START_X, 64, BLACK);

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.println("QUALITY");

  display.setTextSize(2);
  display.setCursor(0, 49);

  const char *waterQualityText;
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

void displayDrawContainerFillingStatus(float filledVolumeMl, float toFillVolumeMl, float maxVolumeMl, float isFilling)
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

    filledRectHeight = map(filledVolumeMl, 0, maxVolumeMl, 0, CONTAINER_HEIGHT);
  }
  else
  {
    filledRectHeight = map(toFillVolumeMl, 0, maxVolumeMl, 0, CONTAINER_HEIGHT);
  }
  display.fillRoundRect(CONTAINER_START_X, CONTAINER_START_Y + CONTAINER_HEIGHT - filledRectHeight, CONTAINER_WIDTH, filledRectHeight, CONTAINER_RECT_RADIUS, WHITE);

  display.setTextSize(1);

  const char *fillStatusText;
  if (!isFilling)
  {
    fillStatusText = "START";
  }
  else if (filledVolumeMl < toFillVolumeMl)
  {
    fillStatusText = "FILLING";
  }
  else if (filledVolumeMl >= toFillVolumeMl)
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
    display.printf("%.1f/", filledVolumeMl);

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
