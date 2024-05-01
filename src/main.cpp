#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;
LiquidCrystal_I2C lcd(0x27,16,2);

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

void setup(void)
{
  pinMode(LED,OUTPUT);
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);
  Serial.println("pH meter experiment!");    //Test the serial monitor
}
void loop(void)
{
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    //clear LCD first
    lcd.clear();

    //display voltage to serial
    Serial.print("Voltage:");
    Serial.print(voltage,2);

    //display voltage to lcd on row 1
    lcd.setCursor(0,0);
    lcd.print("V: ");
    lcd.setCursor(3,0);
    lcd.print(voltage);

    //display PH to serial
    Serial.print("    pH value: ");
    Serial.println(pHValue,2);

    //display PH to lcd on row 1
    lcd.setCursor(8,0);
    lcd.print("PH: ");
    lcd.setCursor(12,0);
    lcd.print(pHValue);

    //turn on led as indicator
    digitalWrite(LED,digitalRead(LED)^1);

    //reset millis
    printTime=millis();
  }
}
