#include <Adafruit_BMP280.h> // library for work with BMP 280
#include <MPU6050.h> //library for work witch gyroskop MPU6050
#include <Wire.h> // library for work with I2C. I2C it's comminication protocol for data exchange betwen microcontroller and sensor.
#include <SPI.h> // comminication protocol for data exchange betwen microcontroller and sensor.
#include <SD.h> // library for work witch SD card adpater.


// all pin for SD card adapter
#define BMP_SCK (13);
#define BMP_MISO (12);
#define BMP_MOSI (11);
#define BMP_CS (4);

Adafruit_BMP280 bmp;
MPU6050 mpu;

const int PIN_CHIP_SELECT = 4;

double AltitudeError;
long Altitude = 0, Alt = 0, maxAlt = 0;
unsigned long time_MOSFET = 0;
char count = 0;

File dataFile;

const int buzzerPin = 8; 

const int RED_LED = 5;
const int GREEN_LED = 3;
const int YELLOW_LED = 2;

const int buttonPin = 6;
bool lastButton = LOW;
bool currentButton = LOW;

int flagDirection1 = 1;

const int mosfetPin = 7;

bool debounce(bool last){
  bool current = digitalRead(buttonPin);
  if(last != current){
    delay(5);
    current = digitalRead(buttonPin);
  }
  return current;
}

void setup(){
  Serial.begin(9600);

  pinMode(8, OUTPUT);

  tone(8, 500);
  delay(100);
  tone(8, 500);
  delay(100);
  noTone(8);

  Wire.begin();
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  Serial.println("MPU6050 is ready.");

  unsigned status;
  status = bmp.begin(0x76);

  double AltitudeReadout = bmp.readAltitude(1003.0);
  double AltitudeError = AltitudeReadout;

  pinMode(4, OUTPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  pinMode(7, OUTPUT);

  if (!SD.begin(PIN_CHIP_SELECT)){
    digitalWrite(RED_LED, HIGH);
    tone(8, 500);
    delay(10000);
    noTone(8);
    Serial.println(F("Card failed, or not present."));
  } else {
    Serial.println(F("Card initialized."));
    digitalWrite(YELLOW_LED, HIGH);
  }
}

void loop(){

  int16_t gyroX = mpu.getRotationX();
  int16_t gyroY = mpu.getRotationY();
  int16_t gyroZ = mpu.getRotationZ();

  currentButton = debounce(lastButton);
  if(lastButton == LOW && currentButton == HIGH){
    delay(50);
    if(flagDirection1 == 1){
      flagDirection1 =2;
      delay(200);
    } else {
      flagDirection1 =1;
      delay(200);
    }
  }

  lastButton = currentButton;
   
  if (flagDirection1 == 1){
    digitalWrite(GREEN_LED, LOW);
  }else {
    digitalWrite(GREEN_LED, HIGH);

    double AltitudeReadout = bmp.readAltitude(1003.0);
    double Altitude = AltitudeReadout - AltitudeError;

    double Pressure = bmp.readPressure();

    // code for inclusion electric ignitor
    Alt = Altitude;     // create variable for maximal altitude
    if (Alt > maxAlt){      // find out a apogeum
      maxAlt = Alt;
    }

    if (Alt < maxAlt-5 and Alt > -1000 and count == 0){     // if rocket dropped less 5 meters than apogeum
      digitalWrite(7, HIGH);      // include MOSFET, which blocks thr bridge beetwen voltage and electric ignitor
      count ++;   // amount activations MOSFET, would be onlu once per one flight
      time_MOSFET = millis();     // remember time
    }

    if (millis() - time_MOSFET > 3000){     // if after including MOSFET passed 3 sekund turn off MOSFET
      digitalWrite(7, LOW);
    }

    if (count == 1 and millis() - time_MOSFET > 300000){
      tone(8, 500);
    }

    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    if (dataFile) {
        dataFile.print(Altitude);
        dataFile.print(", ");
        dataFile.print(Pressure);
        dataFile.print(", ");
        dataFile.print(gyroX);
        dataFile.print(", ");
        dataFile.print(gyroY);
        dataFile.print(", ");
        dataFile.println(gyroZ);
        dataFile.close();

        Serial.print(Altitude);
        Serial.print(" m, ");
        Serial.print( Pressure);
        Serial.print(" Pa");
        Serial.print(", Axis X: ");
        Serial.print(gyroX);
        Serial.print(", Axis Y: ");
        Serial.print(gyroY);
        Serial.print(", Axis Z: ");
        Serial.println(gyroZ);
        delay(1); // Частота записи показаний датчика 1000 = 1 sek
      } else {
        digitalWrite(RED_LED, HIGH);
        delay(300);
        digitalWrite(RED_LED, LOW);
        delay(300);
        Serial.println("error opening datalog.txt");
      }
  }
}

