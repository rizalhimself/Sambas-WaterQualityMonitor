#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <GravityTDS.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <TaskScheduler.h>
#include <time.h>

#define PHSensorPin A0  // ph sensor pin
#define TdsSensorPin A1 // tds sensor pin
#define TBDSensorPin A2 // turbidity sensor pin
#define pinBuzzer 5     // buzzer pin
#define VREF 5.0        // voltage reference of ADC
#define ADCRange 1024   // ADC Range of microcontroller
#define SCOUNT 30       // sum of sample point
#define Offset 0.28     // deviation compensate
#define ArrayLenth 25   // times of collection
#define ONE_WIRE_BUS 3  // Data wire of DS18B is plugged into port 2 on the Arduino

LiquidCrystal_I2C lcd(0x27, 20, 4);      // lcd addresses and spec
OneWire oneWire(ONE_WIRE_BUS);           // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature tempSensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
GravityTDS tdsSensor;                    // tds sensor variable
Scheduler userScheduler;

// task interval
#define getPhDataInterval 20
#define getTempDataInterval 1000
#define getTDSDataInterval 1000
#define displayDataInterval 500
#define sendDataInterval 2000
#define recieveDataInterval 50
#define getTbdDataInterval 1000
#define checkWarnStatsInterval 1000

// variables used
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
int pHArray[ArrayLenth]; // Store the average value of the sensor feedback
int pHArrayIndex = 0;
int tbdSamples = 600;
String message;
float averageVoltage = 0, tdsValue = 0, temperature = 0, pHValue, vPH, vTBD, tbdValue;
int signalStrength = 0, inetStatus = 0, timeHour, timeMin, currentDays, currentMonths, currentYears, ntu;
JsonDocument docSend, docRec;

// custom character
byte signalIconChar[8] = {
    B11111,
    B10001,
    B01010,
    B00100,
    B00100,
    B00100,
    B00100,
    B00100};

byte signalStrength1[8] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B10000,
    B10000};

byte signalStrength2[8] = {
    B00000,
    B00000,
    B00000,
    B00100,
    B00100,
    B00100,
    B10100,
    B10100};

byte signalStrength3[8] = {
    B00001,
    B00001,
    B00001,
    B00101,
    B00101,
    B00101,
    B10101,
    B10101};

byte inetIconChar[8] = {
    B00010,
    B01011,
    B01010,
    B01010,
    B01010,
    B01010,
    B11010,
    B01000};

byte failedIconChar[8] = {
    B00000,
    B00000,
    B00000,
    B10001,
    B01010,
    B00100,
    B01010,
    B10001};

byte inetOkChar[8] = {
    B00000,
    B00000,
    B00000,
    B00000,
    B01100,
    B11010,
    B10110,
    B01100};

byte mgPerL[8] = {
    B11111,
    B11111,
    B10101,
    B00011,
    B00100,
    B01010,
    B10010,
    B00011};

// prototype function
void getPHData();
void getTempData();
void getTDSData();
void sendData();
void displayData();
void recieveData();
void getTbdData();
void checkWarnStats();

// define task for multitasking
Task taskGetTempData(getTempDataInterval, TASK_FOREVER, &getTempData);
Task taskGetPHData(getPhDataInterval, TASK_FOREVER, &getPHData);
Task taskGetTdsData(getTDSDataInterval, TASK_FOREVER, &getTDSData);
Task taskSendData(sendDataInterval, TASK_FOREVER, &sendData);
Task taskRecieveData(recieveDataInterval, TASK_FOREVER, &recieveData);
Task taskDisplayData(displayDataInterval, TASK_FOREVER, &displayData);
Task taskGetDateFromEpoch(getTbdDataInterval, TASK_FOREVER, &getTbdData);
Task taskCheckWarnStats(checkWarnStatsInterval, TASK_FOREVER, &checkWarnStats);

// create character function
void lcdCreateCharacter()
{
  lcd.createChar(0, signalIconChar);
  lcd.createChar(1, signalStrength1);
  lcd.createChar(2, signalStrength2);
  lcd.createChar(3, signalStrength3);
  lcd.createChar(4, inetIconChar);
  lcd.createChar(5, failedIconChar);
  lcd.createChar(6, inetOkChar);
  lcd.createChar(7, mgPerL);
}

// request Temp data from sensor
void getTempData()
{
  // request temperature to DS18 Sensor
  tempSensors.requestTemperatures();
  temperature = tempSensors.getTempCByIndex(0);
}

// function to count average data from ph sensor
double averageArray(int *arr, int number)
{
  int i;
  int max, min;
  double avg;
  long amount = 0;
  if (number <= 0)
  {
    // Serial.println("Error number for the array to avraging!/n");
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

// function to limit decimal place
float roundToDp(float in_value, int decimal_place)
{
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier) / multiplier;
  return in_value;
}

// function to check warning and status
void checkWarnStats()
{
  if (tdsValue > 500)
  {
    /* code */
  }
  else
  {
    /* code */
  }
}

// function to clear lcd line
void clearCursor(int cursorStart, int cursorEnd, int line)
{
  lcd.setCursor(cursorStart, line);
  int countCursor = cursorEnd - cursorStart + 1;
  for (int i = 0; i < countCursor; i++)
  {
    lcd.print(" ");
  }
}

// request PH data from sensor
void getPHData()
{
  pHArray[pHArrayIndex++] = analogRead(PHSensorPin);
  if (pHArrayIndex == ArrayLenth)
    pHArrayIndex = 0;
  vPH = averageArray(pHArray, ArrayLenth) * VREF / 1024;
  pHValue = 3.5 * vPH + Offset;
}

// request tds value from sensor
void getTDSData()
{
  tdsSensor.setTemperature(temperature);
  tdsSensor.update();
  tdsValue = tdsSensor.getTdsValue();
}

// send data value trough serial connection to wemos
void sendData()
{
  docSend["tds"] = tdsValue;
  docSend["temp"] = temperature;
  docSend["ph"] = pHValue;
  serializeJson(docSend, Serial3);
}

// recieve data/status value from wemos
void recieveData()
{
  if (Serial3.available())
  {
    message = Serial3.readString();
    // Serial.println("Pesan :" + message);
  }
  deserializeJson(docRec, message);
  signalStrength = docRec["sg"];
  inetStatus = docRec["tps"];
  timeMin = docRec["tm"];
  timeHour = docRec["th"];
  currentDays = docRec["cd"];
  currentMonths = docRec["cm"];
  currentYears = docRec["cy"];
  docRec.clear();
}

// request TBD data from sensor
void getTbdData()
{
  vTBD = 0;
  for (int i = 0; i < tbdSamples; i++)
  {
    vTBD += ((float)analogRead(TBDSensorPin) / 1023) * 5;
  }
  vTBD = vTBD / tbdSamples;
  vTBD = roundToDp(vTBD, 2);
  if (vTBD < 2.5)
  {
    tbdValue = 3000;
  }
  else if (vTBD > 4.2)
  {
    tbdValue = 0;
  }
  else
  {
    tbdValue = -1120.4 * square(vTBD) + 5742.3 * vTBD - 4352.9;
  }
  ntu = tbdValue;
}

// display data to LCD and/or serial monitor
void displayData()
{
  // clear LCD first
  // lcd.clear();

  // display voltage PH to serial
  // Serial.print("Voltage PH:");
  // Serial.print(voltage, 2);

  // display signal icon to lcd on row 1
  lcd.setCursor(0, 0);
  lcd.write(byte(0));
  if (signalStrength <= -30 && signalStrength >= -51)
  {
    lcd.setCursor(1, 0);
    lcd.print("");
    lcd.write(byte(3));
  }
  else if (signalStrength <= -51 && signalStrength >= -56)
  {
    lcd.setCursor(1, 0);
    lcd.print("");
    lcd.write(byte(2));
  }
  else if (signalStrength <= -56)
  {
    lcd.setCursor(1, 0);
    lcd.print("");
    lcd.write(byte(1));
  }
  else if (signalStrength == 0)
  {
    lcd.setCursor(1, 0);
    lcd.write(byte(5));
  }

  // display inet icon to lcd on row 1
  lcd.setCursor(18, 0);
  lcd.write(byte(4));
  if (inetStatus == 200)
  {
    lcd.setCursor(19, 0);
    lcd.print("");
    lcd.write(byte(6));
  }
  else
  {
    lcd.setCursor(19, 0);
    lcd.print("");
    lcd.write(byte(5));
  }

  // display date to lcd on row 1
  if (currentYears != 0)
  {
    if (currentDays < 10)
    {
      lcd.setCursor(3, 0);
      lcd.print("");
      lcd.print(0);
      lcd.setCursor(4, 0);
      lcd.print("");
      lcd.print(currentDays);
    }
    else
    {
      lcd.setCursor(3, 0);
      lcd.print("");
      lcd.print(currentDays);
    }
    lcd.setCursor(5, 0);
    lcd.print("/");
    if (currentMonths < 10)
    {
      lcd.setCursor(6, 0);
      lcd.print("");
      lcd.print(0);
      lcd.setCursor(7, 0);
      lcd.print("");
      lcd.print(currentMonths);
    }
    else
    {
      lcd.setCursor(6, 0);
      lcd.print("");
      lcd.print(currentMonths);
    }
    lcd.setCursor(8, 0);
    lcd.print("/");
    lcd.setCursor(9, 0);
    lcd.print("");
    lcd.print(currentYears);
  }

  // display time to lcd on row 1
  if (timeHour < 10)
  {
    lcd.setCursor(12, 0);
    lcd.print("");
    lcd.print(0);
    lcd.setCursor(13, 0);
    lcd.print("");
    lcd.print(timeHour);
  }
  else
  {
    lcd.setCursor(12, 0);
    lcd.print("");
    lcd.print(timeHour);
  }
  lcd.setCursor(14, 0);
  lcd.print(":");
  if (timeMin < 10)
  {
    lcd.setCursor(15, 0);
    lcd.print("");
    lcd.print(0);
    lcd.setCursor(16, 0);
    lcd.print("");
    lcd.print(timeMin);
  }
  else
  {
    lcd.setCursor(15, 0);
    lcd.print("");
    lcd.print(timeMin);
  }

  // display temperature to lcd on row 2
  lcd.setCursor(1, 1);
  lcd.print("TMP:");
  lcd.setCursor(5, 1);
  lcd.print("");
  lcd.print(temperature, 1);
  lcd.write(223);
  // display TDS value to serial
  // Serial.print("    TDS Value:");
  // Serial.print(tdsValue, 0);
  // Serial.println("ppm");

  // display TDS value to lcd on row 2
  clearCursor(15, 19, 1);
  lcd.setCursor(11, 1);
  lcd.print("TDS:");
  lcd.setCursor(15, 1);
  lcd.print(tdsValue, 0);
  lcd.write(byte(7));

  // display TBD to lcd on row 3
  clearCursor(4, 10, 2);
  lcd.setCursor(1, 2);
  lcd.print("TB:");
  lcd.setCursor(4, 2);
  lcd.print(ntu);
  lcd.print("ntu");

  // display PH to serial
  // Serial.print("    pH value: ");
  // Serial.println(pHValue, 2);

  // display PH to lcd on row 3
  lcd.setCursor(12, 2);
  lcd.print("PH:");
  lcd.setCursor(15, 2);
  lcd.print("");
  lcd.print(pHValue);

  // display voltage TDS to serial
  // Serial.print("Voltage TDS:");
  // Serial.print(averageVoltage, 2);
  // Serial.print("V   ");

  // display warning on row 4
  lcd.setCursor(0, 3);
  lcd.print("Warn:");

  // display status on row 4
  lcd.setCursor(11, 3);
  lcd.print("Stats:");
}

// fancy startup sound
void nadaStartup()
{
  tone(pinBuzzer, 500, 300);
  delay(300);
  tone(pinBuzzer, 1500, 350);
  delay(350);
  tone(pinBuzzer, 650, 200);
  delay(300);
  noTone(pinBuzzer);
}

// user define display text on startup
void displayStartup()
{
  lcd.setCursor(7, 1);
  lcd.print("Water QM");
  lcd.setCursor(6, 2);
  lcd.print("Ari Selan");
}

// setup root function
void setup(void)
{
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial3.setTimeout(300);
  pinMode(TdsSensorPin, INPUT);
  pinMode(pinBuzzer, OUTPUT);
  tempSensors.begin();
  tdsSensor.setPin(TdsSensorPin);
  tdsSensor.setAref(VREF);
  tdsSensor.setAdcRange(ADCRange);
  tdsSensor.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcdCreateCharacter();
  displayStartup();
  nadaStartup();
  delay(2000);
  userScheduler.addTask(taskGetTempData);
  userScheduler.addTask(taskGetPHData);
  userScheduler.addTask(taskGetTdsData);
  userScheduler.addTask(taskSendData);
  userScheduler.addTask(taskRecieveData);
  userScheduler.addTask(taskGetDateFromEpoch);
  userScheduler.addTask(taskDisplayData);
  taskGetTempData.enable();
  taskGetPHData.enable();
  taskGetTdsData.enable();
  taskSendData.enable();
  taskRecieveData.enable();
  taskGetDateFromEpoch.enable();
  taskDisplayData.enable();
  lcd.clear();
}

// loop root function
void loop(void)
{
  userScheduler.execute();
  // recieveData();
}
