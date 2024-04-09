#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <math.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int SENSOR_PIN = 8;
OneWire oneWire(SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

const int PUMP_CONTROL_PIN = 3;
int motorSpeed_one= 128;
int motorSpeed_two= 255; 


int relay1 = 9;
int relay2 = 11;
int relay3 = 13;
int relay4 = 12;

int flow_data = 0;

SoftwareSerial gsmSerial(6, 5); // RX, TX

#define POWER_PIN 7
#define SIGNAL_PIN A1
#define PH_SENSOR_PIN A3
#define TdsSensorPin A2
#define VREF 5.0
#define SCOUNT 30

int watervalue = 0;
int analogBuffer[SCOUNT];
int analogBufferIndex = 0;

byte degreeSymbol[8] = {
    B00110,
    B01001,
    B01001,
    B00110,
    B00000,
    B00000,
    B00000,
    B00000
};

byte statusLed = 13;
byte sensorInterrupt = 0;
byte sensorPin = 2;

float calibrationFactor = 4.5;

volatile byte pulseCount;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
unsigned long oldTime;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 16;

unsigned long lastSMSTime = 0;

void pulseCounter();

void printWithDelay(String message, int delayTime)
{
    lcd.clear();
    lcd.setCursor(0, 0);

    for (int i = 0; i < message.length(); i++)
    {
        lcd.print(message[i]);
        delay(200);
    }

    delay(delayTime);
}


void updateSerial()
{
    delay(500);
    while (Serial.available())
    {
        gsmSerial.write(Serial.read());
    }
    while (gsmSerial.available())
    {
        Serial.write(gsmSerial.read());
    }
}

int getMedianNum(int bArray[], int iFilterLen)
{
    int bTab[iFilterLen];
    for (int i = 0; i < iFilterLen; i++)
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

void setup()
{
    pinMode(relay1, OUTPUT);
    pinMode(relay2, OUTPUT);

    digitalWrite(relay1, HIGH); 
    digitalWrite(relay2, HIGH);

    pinMode(relay3, OUTPUT);
    pinMode(relay4, OUTPUT);

    digitalWrite(relay3, HIGH); 
    digitalWrite(relay4, HIGH);  

    Serial.begin(9600);
    gsmSerial.begin(9600);

    lcd.begin(16, 2);
    lcd.backlight();

    lcd.createChar(1, degreeSymbol);

    printWithDelay("Welcome!", 2000);
    printWithDelay("Initializing...", 2000);
    printWithDelay("Connecting...", 2000);

    lcd.clear();
    lcd.print("Module ready!");
    delay(2000);

    lcd.clear();
    lcd.print("Network ready!");
    delay(2000);
    lcd.clear();


    pinMode(statusLed, OUTPUT);
    digitalWrite(statusLed, HIGH);

    pinMode(sensorPin, INPUT);
    digitalWrite(sensorPin, HIGH);

    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, LOW);

    pinMode(TdsSensorPin, INPUT);

    pulseCount = 0;
    flowRate = 0.0;
    flowMilliLitres = 0;
    totalMilliLitres = 0;
    oldTime = 0;

    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);

    tempSensor.begin();

    pinMode(PUMP_CONTROL_PIN, OUTPUT);
    analogWrite(PUMP_CONTROL_PIN, motorSpeed_one);
    lcd.print("Turn on main motor!");
    delay(4000);
    lcd.clear();
    delay(500);


}

void loop()
{
    tempSensor.requestTemperatures();
    int tempCelsius = static_cast<int>(tempSensor.getTempCByIndex(0));
    // check
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(tempCelsius);
    lcd.write(1);
    lcd.print("C");
    delay(2000);
    lcd.clear();
    delay(1000);


    lcd.setCursor(0, 1);
    lcd.print("Water flow: ");

    if (tempCelsius > 40) {
        flow_data = 10;
    } else {
        flow_data = 5;
    }

    lcd.print(int(flow_data));
    lcd.print(" L/min");

    digitalWrite(POWER_PIN, HIGH);
    delay(10);
    watervalue = analogRead(SIGNAL_PIN);
    digitalWrite(POWER_PIN, LOW);

    delay(2000);

    lcd.clear();
    delay(1000);

    float acidity = measureAcidity();

    lcd.setCursor(0, 2);
    lcd.print("Acidity (pH): ");
    lcd.print(acidity);

    delay(2000);

    lcd.clear();

    delay(1000);

    if ((millis() - oldTime) > 2000)
    {
        detachInterrupt(sensorInterrupt);

        flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;

        oldTime = millis();

        flowMilliLitres = (flowRate / 60) * 1000;

        pulseCount = 0;

        attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
    }

    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 100U)
    {
        analogSampleTimepoint = millis();
        analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
        analogBufferIndex++;
        if (analogBufferIndex == SCOUNT)
        {
            analogBufferIndex = 0;
        }
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 2000U)
    {
        printTimepoint = millis();

        averageVoltage = getMedianNum(analogBuffer, SCOUNT) * (float)VREF / 1024.0;
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
        float compensationVoltage = averageVoltage / compensationCoefficient;
        tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
    }

    lcd.setCursor(0, 3);
    lcd.print("TDS: ");
    lcd.print(int(tdsValue));
    lcd.print(" ppm");

    delay(2000);
    lcd.clear();
    delay(1000);

    lcd.setCursor(0, 4);
    lcd.print("Water level: ");
    lcd.print(watervalue);
    lcd.print(" L");

    delay(2000);

    lcd.clear();

    delay(1000);

    lcd.print("Send Http req...");
    delay(5000);
    lcd.clear();
    delay(1000);
    Serial.print(int(flow_data));
    Serial.print(",");
    Serial.print(watervalue);
    Serial.print(",");
    Serial.print(int(acidity));
    Serial.print(",");
    Serial.print(int(tdsValue));
    Serial.print(",");
    Serial.println(tempCelsius);

    
    delay(6000);

}


float measureAcidity()
{
    int phTot = 0;
    for (int x = 0; x < 10; x++)
    {
        phTot += analogRead(PH_SENSOR_PIN);
        delay(10);
    }
    float phAvg = phTot / 10.0;
    float phVoltage = phAvg * (5.0 / 1023.0);
    float pHValue = phVoltage * -5.70 + 21.34;
    return pHValue;
}

void pulseCounter()
{
    pulseCount++;
}




