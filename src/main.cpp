#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PHSensorPin A0  // ph sensor pin
#define TdsSensorPin A1 // tds sensor pin
#define VREF 5.0        // voltage reference of ADC
#define SCOUNT 30       // sum of sample point
#define Offset 0.00     // deviation compensate
#define LED 13
#define samplingInterval 20
#define samplingInterval2 40U
#define printInterval 800
#define ArrayLenth 40 // times of collection

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
int pHArray[ArrayLenth]; // Store the average value of the sensor feedback
int pHArrayIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

LiquidCrystal_I2C lcd(0x27, 16, 2); // lcd addresses and spec

double avergearray(int *arr, int number)
{
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0)
  {
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if (number < 5)
  { // less than 5, calculated directly statistics
    for (i = 0; i < number; i++)
    {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  }
  else
  {
    if (arr[0] < arr[1])
    {
      min = arr[0];
      max = arr[1];
    }
    else
    {
      min = arr[1];
      max = arr[0];
    }
    for (i = 2; i < number; i++)
    {
      if (arr[i] < min)
      {
        amount += min; // arr<min
        min = arr[i];
      }
      else
      {
        if (arr[i] > max)
        {
          amount += max; // arr>max
          max = arr[i];
        }
        else
        {
          amount += arr[i]; // min<=arr<=max
        }
      } // if
    }   // for
    avg = (double)amount / (number - 2);
  } // if
  return avg;
}

int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

void setup(void)
{
  pinMode(LED, OUTPUT);
  pinMode(TdsSensorPin, INPUT);
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  Serial.println("pH meter experiment!"); // Test the serial monitor
}
void loop(void)
{
  static unsigned long samplingTime = millis();
  static unsigned long samplingTime2 = millis();
  static unsigned long printTime = millis();
  static float pHValue, voltage;
  if (millis() - samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++] = analogRead(PHSensorPin);
    if (pHArrayIndex == ArrayLenth)
      pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
  }
  if (millis() - samplingTime2 > samplingInterval2) // every 40 milliseconds,read the analog value from the ADC
  {
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
    samplingTime2 = millis();
  }

  if (millis() - printTime > printInterval) // Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                  // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                               // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                            // temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; // convert voltage value to tds value

    // clear LCD first
    lcd.clear();

    // display voltage PH to serial
    Serial.print("Voltage PH:");
    Serial.print(voltage, 2);

    // display voltage PH to lcd on row 1
    lcd.setCursor(0, 0);
    lcd.print("V:");
    lcd.setCursor(3, 0);
    lcd.print(voltage);

    // display PH to serial
    Serial.print("    pH value: ");
    Serial.println(pHValue, 2);

    // display PH to lcd on row 1
    lcd.setCursor(8, 0);
    lcd.print("PH:");
    lcd.setCursor(12, 0);
    lcd.print(pHValue);

    // display voltage TDS to serial
    Serial.print("Voltage TDS:");
    Serial.print(averageVoltage, 2);
    Serial.print("V   ");

    // display voltage TDS to lcd on row 2
    lcd.setCursor(0, 1);
    lcd.print("V:");
    lcd.setCursor(3, 1);
    lcd.print(averageVoltage);

    // display TDS value to serial
    Serial.print("    TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");

    // display TDS value to lcd on row 2
    lcd.setCursor(8, 1);
    lcd.print("TD:");
    lcd.setCursor(12, 1);
    lcd.print(tdsValue, 0);

    // turn on led as indicator
    digitalWrite(LED, digitalRead(LED) ^ 1);

    // reset millis
    printTime = millis();
  }
}
