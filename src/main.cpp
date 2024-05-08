#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <GravityTDS.h>
#include <EEPROM.h>

#define PHSensorPin A0  // ph sensor pin
#define TdsSensorPin A5 // tds sensor pin
#define VREF 5.0        // voltage reference of ADC
#define ADCRange 1024   // ADC Range of microcontroller
#define SCOUNT 30       // sum of sample point
#define Offset 0.00     // deviation compensate
#define LED 13
#define samplingInterval 20
#define samplingInterval2 1000
#define printInterval 800
#define ArrayLenth 40 // times of collection
#define ONE_WIRE_BUS 2 // Data wire of DS18B is plugged into port 2 on the Arduino

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
int pHArray[ArrayLenth]; // Store the average value of the sensor feedback
int pHArrayIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2); // lcd addresses and spec
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature tempSensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
GravityTDS tdsSensor;

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
    } // for
    avg = (double)amount / (number - 2);
  } // if
  return avg;
}

void setup(void)
{
  pinMode(LED, OUTPUT);
  pinMode(TdsSensorPin, INPUT);
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  tempSensors.begin();
  tdsSensor.setPin(TdsSensorPin);
  tdsSensor.setAref(VREF);
  tdsSensor.setAdcRange(ADCRange);
  tdsSensor.begin();
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
    voltage = avergearray(pHArray, ArrayLenth) * VREF / 1024;
    pHValue = 3.5 * voltage + Offset;
    samplingTime = millis();
  }

  if (millis() - samplingTime2 > samplingInterval2)
  {
    // request tds value from sensor
    tdsSensor.setTemperature(temperature);
    tdsSensor.update();
    tdsValue = tdsSensor.getTdsValue();
    // request temperature to DS18 Sensor
    tempSensors.requestTemperatures();
    temperature = tempSensors.getTempCByIndex(0);
    samplingTime2 = millis();
  }
  

  if (millis() - printTime > printInterval) // Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    
  
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

    // display temperature to lcd on row 2
    lcd.setCursor(0, 1);
    lcd.print("T:");
    lcd.setCursor(3, 1);
    lcd.print(temperature, 1);

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
