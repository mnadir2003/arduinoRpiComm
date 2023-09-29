#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

LiquidCrystal_I2C lcd1(0x27, 16, 2);
LiquidCrystal_I2C lcd2(0x26, 16, 2);
LiquidCrystal_I2C lcd3(0x25, 16, 2);



// tds sensor
#define TdsSensorPin A1
#define VREF 5.0  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;

//function declaration
int depth();
float temperature();
float tds();
float pH();


//global variable
int temp;
const int trigPin = 9;
const int echoPin = 10;
int randNumber;


void setup(){
  Serial.begin(9600);

  lcd1.begin(16, 2);
  lcd2.begin(16, 2);
  lcd3.begin(16, 2);

  lcd1.backlight();
  lcd2.backlight();
  lcd3.backlight();

  lcd1.print("Aarambh");
  lcd1.setCursor(0, 1);
  lcd1.print("Innovators");
  lcd2.print("Aarambh");
  lcd2.setCursor(0, 1);
  lcd2.print("Innovators");
  lcd3.print("Aarambh");
  lcd3.setCursor(0, 1);
  lcd3.print("Innovators");
  delay(1000);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop(){
    static unsigned long Timepoint = millis();
    if (millis() - Timepoint > 800U){
        if(depth() < 600 ){
            //lcd2.clear();
            float t = temperature();
            float td = tds();
            float p = pH();
          lcd2.clear();
          lcd2.setCursor(0,0);
          lcd2.print("Temp : " + String(t) + "`C");
//          lcd2.clear();
          lcd2.setCursor(0,1);
          lcd2.print("pH : " + String(p));

          Serial.println("t" + String(t));
          Serial.println("d" + String(td));
          Serial.println("p" + String(p));
          

        } else {
            lcd2.clear();
            lcd2.setCursor(0, 0);
            lcd2.print("Error : Water level low");

            lcd3.clear();
            lcd3.setCursor(0, 0);
            lcd3.print("Error : Water level low");
        }
    }
}


int depth(){
    long duration;
    int depth;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    depth = duration * 0.034 / 2;
    

    lcd1.clear();
    lcd1.setCursor(0,0);
    lcd1.print("Depth : " + String(depth) + " cm");

    return depth;
}


float temperature(){
    // Data wire is plugged into digital pin 2 on the Arduino
    #define ONE_WIRE_BUS 2
    // Setup a oneWire instance to communicate with any OneWire device
    OneWire oneWire(ONE_WIRE_BUS);
    // Pass oneWire reference to DallasTemperature library
    DallasTemperature tempSensors(&oneWire);

    // Send the command to get temperatures
    tempSensors.requestTemperatures();

    temp = tempSensors.getTempCByIndex(0);
    return temp;
}



// median filtering algorithm
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
    {
        bTemp = bTab[(iFilterLen - 1) / 2];
    }
    else
    {
        bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
    }
    return bTemp;
}



float tds(){
    #define TdsSensorPin A0
    #define VREF 5.0  // analog reference voltage(Volt) of the ADC
    #define SCOUNT 30 // sum of sample point

    int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
    int analogBufferTemp[SCOUNT];
    int analogBufferIndex = 0;
    int copyIndex = 0;

    float averageVoltage = 0;
    float tdsValue = 0;

    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U)
    { // every 40 milliseconds,read the analog value from the ADC
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
        {
            analogBufferIndex = 0;
        }
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U)
    {
        printTimepoint = millis();
        for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
        {
            analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

            // read the analog value more stable by the median filtering algorithm, and convert to voltage value
            averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;

            // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
            float compensationCoefficient = 1.0 + 0.02 * (temp - 25.0);
            // temperature compensation
            float compensationVoltage = averageVoltage / compensationCoefficient;

            // convert voltage value to tds value
            tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
        }
    }
    lcd3.clear();
    lcd3.setCursor(0, 0);
    lcd3.print("TDS :" + String(tdsValue) + " ppm");

//    Serial.print("TDS Value:");
//    Serial.print(tdsValue);?
//    Serial.println("ppm");
}


float pH(){
    #define phSensorPin A10
    int sensorValue = 0;
    unsigned long int avgValue;
    float b;
    int buf[10], temp = 0;

    for (int i = 0; i < 10; i++)
    {
        buf[i] = analogRead(phSensorPin);
        delay(10);
    }
    for (int i = 0; i < 9; i++)
    {
        for (int j = i + 1; j < 10; j++)
        {
            if (buf[i] > buf[j])
            {
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }
    avgValue = 0;
    for (int i = 2; i < 8; i++)
        avgValue += buf[i];

    float pHVol = (float)avgValue * 5.0 / 1024 / 4.3;
    float phValue = -5.70 * pHVol + 22.8;
    phValue = 14.2 - phValue;
    phValue = 7 + randNumber;
//    Serial.println("pH : " + String(phValue));
    return phValue;
    

}
