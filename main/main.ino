#include <algorithm>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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

volatile byte flowSensorCurrPulseCount = 0;

Adafruit_SSD1306 display(128, 64, &Wire);

const int16_t CONTAINER_START_X = display.width() / 2 + 25;

void IRAM_ATTR flowSensorInterruptCallback()
{
  flowSensorCurrPulseCount++;
}

void setup()
{
  Serial.begin(9600);

  pinMode(TDS_SENSOR_PIN, INPUT);

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowSensorInterruptCallback, FALLING);

  setupDisplay();
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
}

void displayDrawTotalFlow(float flowValueL)
{
  display.fillRect(0, 0, CONTAINER_START_X, 40, BLACK);

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
  display.fillRect(0, 40, CONTAINER_START_X, 64, BLACK);

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

unsigned long flowSensorPrevTimestamp = millis();

float totalFlowMl = 0;

unsigned long tdsSensorPrevSampleTimestamp = millis();
unsigned long tdsSensorPrevPrintTimestamp = millis();

int tdsSensorBuffer[TDS_SENSOR_SAMPLE_COUNT];
int tdsSensorCurrBufferIndex = 0;

void loop()
{
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

    displayDrawTotalFlow(totalFlowMl / 1000);

    Serial.print("Flow rate: ");
    Serial.print(int(flowRate));
    Serial.print("L/min");
    Serial.print("\t");

    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalFlowMl);
    Serial.print("mL / ");
    Serial.print(totalFlowMl / 1000);
    Serial.println("L");
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

    Serial.print("voltage:");
    Serial.print(averageVoltage, 2);
    Serial.print("V   ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
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
