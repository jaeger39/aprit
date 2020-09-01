#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_BMP280.h>
#include "I2Cdev.h"
#include <TinyMPU6050.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>
//#define Controller
#define LCD_TRUE
#ifdef LCD_TRUE
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#endif

//---PID---
int maxError = 50;
float angleSetPoint = 0.5;
int maxErrorSum = 500;
int PIDsetPoint = 300;
float PidFactor = 1;
float Kp = 1.1; //1
float Ki = 0;//0.01
int Kd = 140;//140
float PIDnet;
float x_setPoint = 0;
float y_setPoint = 0;
float z_setPoint = 0;
float a_setPoint;
float throttle1;
float throttle2;
float throttle3;
float throttle4;
float xOutput;
float yOutput;
float zOutput;
float altOutput;
float throttleAll;

float x_PIDnet;
float x_error;
float x_errorSum;
float x_angle;
float x_previousError;
uint32_t x_currentTime;
uint32_t x_timeChange;
uint32_t x_previousTime;
float x_PIDp;
float x_PIDi;
float x_PIDd;

float y_PIDnet;
float y_error;
float y_errorSum;
float y_angle;
float y_previousError;
uint32_t y_currentTime;
uint32_t y_timeChange;
uint32_t y_previousTime;
float y_PIDp;
float y_PIDi;
float y_PIDd;

float z_PIDnet;
float z_error;
float z_errorSum;
float z_angle;
float z_previousError;
uint32_t z_currentTime;
uint32_t z_timeChange;
uint32_t z_previousTime;
float z_PIDp;
float z_PIDi;
float z_PIDd;

float altSetPoint;
float KpA = 5;
float KiA = 0.02;
int KdA = 1100;
float PIDnetA;
float a_PIDnet;
float a_error;
float a_errorSum;
float a_angle;
float a_previousError;
uint32_t a_currentTime;
uint32_t a_timeChange;
uint32_t a_previousTime;
float a_PIDp;
float a_PIDi;
float a_PIDd;




#ifdef LCD_TRUE
//---LCD---
#define I2C_ADDR          0x27 //0x27 Blue, 0x3F Green
#define BACKLIGHT_PIN      3
#define En_pin             2
#define Rw_pin             1
#define Rs_pin             0
#define D4_pin             4
#define D5_pin             5
#define D6_pin             6
#define D7_pin             7
LiquidCrystal_I2C      lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);
char errorString[7];
byte  errorCode;
byte  errorCount;
#endif
//---NRF24L01---
#define CE_PIN 7
#define CSN_PIN 8
const byte slaveAddress[5] = {'R', 'x', 'A', 'A', 'A'};
const byte masterAddress[5] = {'T', 'X', 'A', 'A', 'A'};
int dataReceived[7];
RF24 radio(CE_PIN, CSN_PIN);
bool newData = false;
uint32_t delayNRF;
uint32_t currentMillis;
uint32_t prevMillis;


//---BMP280---
Adafruit_BMP280 bmp;
float calibPress = 0;
int calibLoop = 300;
float temp;
float pressure;
float height;

//---VL53L0X---
VL53L0X sensor;
int groundHeight;


//---vSensor---
float vIn;
float vOut;
float R1 = 30000.0;
float R2 = 7500.0;

//---MPU6050---
MPU6050 mpu (Wire);

//---Motor---
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
int minThrottle = 1100;
int maxThrottle = 1900;
int nullThrottle = minThrottle;
int powerInc = 300;
int power1 = minThrottle;
int power2 = minThrottle;
int power3 = minThrottle;
int power4 = minThrottle;
uint32_t delayMotor;
uint32_t millisMotor;

//---PS2---
int Lx;
int Ly;
int Rx;
int Ry;
int LxNew;
int LyNew;
int RxNew;
int RyNew;





void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  Serial.println("Slave Starting");

  //Interrupt setup if no other components powered
  Wire.beginTransmission(0x68);
  Wire.endTransmission();
  

#ifdef LCD_TRUE
  //---LCD---
  Serial.println("LCD Starting");
  lcd.begin (16, 2);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("1/6");
  lcd.setCursor(0, 0);
  lcd.print("Master");
  lcd.setCursor(0, 1);
  lcd.print("Starting");
  //delay(1000);
#endif
  //---NRF24L01---
  Serial.println("NRF Starting");
#ifdef LCD_TRUE
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("2/6");
  lcd.setCursor(0, 0);
  lcd.print("NRF");
  lcd.setCursor(9, 0);
  lcd.print("ERROR#0");
  lcd.setCursor(15, 0);
  lcd.print(errorCount);
  lcd.setCursor(4, 1);
  lcd.print(errorString);
#endif
  radio.begin();
  radio.setDataRate( RF24_250KBPS );
  radio.setPALevel(RF24_PA_MIN);

  radio.openReadingPipe(1, slaveAddress);
  radio.setRetries(1, 5); // delay, count
  prevMillis = millis(); // set clock
  bool rslt;
  radio.startListening();




  //---Motor---
#ifdef LCD_TRUE
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("6/6");
  lcd.setCursor(0, 0);
  lcd.print("Motor");
  lcd.setCursor(9, 0);
  lcd.print("ERROR#0");
  lcd.setCursor(15, 0);
  lcd.print(errorCount);
  lcd.setCursor(4, 1);
  lcd.print(errorString);

#endif

  motor1.attach(5);//number 1(bl-3) Reversed c|BL
  motor2.attach(9);//number 2(bl-4) Normal cc|FR

  motor4.attach(6);// number 4(bl-7) Normal cc|BR
  motor3.attach(10);//number 3(bl-8) Reversed  c|FL
  delay(1000);



  //delay(1000);
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(1000);

  motor1.writeMicroseconds(1500);
  motor2.writeMicroseconds(1500);
  motor3.writeMicroseconds(1500);
  motor4.writeMicroseconds(1500);
  //delay(100);


  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(1000);


  //---BMP280---
  Serial.println("BMP Starting");
  Serial.println();
#ifdef LCD_TRUE
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("3/6");
  lcd.setCursor(0, 0);
  lcd.print("BMP");
  lcd.setCursor(9, 0);
  lcd.print("ERROR#0");
  lcd.setCursor(15, 0);
  lcd.print(errorCount);
  lcd.setCursor(4, 1);
  lcd.print(errorString);
#endif
  Serial.println("calib011 Starting");
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor");
#ifdef LCD_TRUE
    errorCount++;
    //errorString = errorString + " BMP";
    lcd.setCursor(15, 0);
    lcd.print(errorCount);
    lcd.setCursor(4, 1);
    lcd.print(errorString);
#endif
  }
  Serial.println("calib0 Starting");
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  Serial.println("calib Starting");
  calibration();
  Serial.println("calib1 Starting");
  //---MPU6050---
  Serial.println("MPU Starting");
  Serial.println();
#ifdef LCD_TRUE
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("4/6");
  lcd.setCursor(0, 0);
  lcd.print("MPU");
  lcd.setCursor(9, 0);
  lcd.print("ERROR#0");
  lcd.setCursor(15, 0);
  lcd.print(errorCount);
  lcd.setCursor(4, 1);
  lcd.print(errorString);
#endif
  mpu.Initialize();
  mpu.SetGyroOffsets (-69, -64.65, -30.72);//

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // the config address
  Wire.write(0x06);  // the config value
  Wire.endTransmission(true);
  //---VL53L0X---
  Serial.println("VL53 Starting");
#ifdef LCD_TRUE
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("5/6");
  lcd.setCursor(0, 0);
  lcd.print("VL53");
  lcd.setCursor(9, 0);
  lcd.print("ERROR#0");
  lcd.setCursor(15, 0);
  lcd.print(errorCount);
  lcd.setCursor(4, 1);
  lcd.print(errorString);
#endif
  sensor.setTimeout(500);
  if (!sensor.init())
  {
#ifdef LCD_TRUE
    errorCount++;
    //errorString = errorString + " VL";
    lcd.setCursor(15, 0);
    lcd.print(errorCount);
    lcd.setCursor(4, 1);
    lcd.print(errorString);
#endif
    Serial.println("Failed to detect and initialize VL53L0X sensor!");
  }



  //---Motor---
  Serial.println("Motor Starting");
#ifdef LCD_TRUE
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("6/6");
  lcd.setCursor(0, 0);
  lcd.print("Motor");
  lcd.setCursor(9, 0);
  lcd.print("ERROR#0");
  lcd.setCursor(15, 0);
  lcd.print(errorCount);
  lcd.setCursor(4, 1);
  lcd.print(errorString);

#endif



  Serial.println("Setup Finished");
#ifdef LCD_TRUE
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Running");
  lcd.setCursor(9, 0);
  lcd.print("ERROR#");
  lcd.setCursor(15, 0);
  lcd.print(errorCount);
  lcd.setCursor(4, 1);
  lcd.print(errorString);
  lcd.clear();
#endif

}


void loop() {

  Serial.print("Start loop");
  
  currentMillis = millis();
  Serial.print("timeC: ");
    Serial.println(currentMillis);
  delayNRF = currentMillis - prevMillis;
  Serial.print("getData");
  getData();
  //showData();
  prevMillis = millis();
  //Serial.print("DELAY: ");
  //Serial.println(delayNRF);
  Serial.print("Motor1");
  //---Motor---
  Lx = dataReceived[0] * 2; //-100 to 100
  Ly = dataReceived[1]; //1100 to 1900 //For position based throttle
  //Ly = dataReceived[1];//-100 to 100 //For acceleration based throttle
  x_setPoint = dataReceived[2];//-30 to 30,Rx
  y_setPoint = dataReceived[3];//-30 to 30,Ry
  Serial.print("Lx, Ly, Rx, Ry");
  Serial.print(Lx);
  Serial.print(", ");
  Serial.print(Ly);
  Serial.print(", ");
  Serial.print(Rx);
  Serial.print(", ");
  Serial.println(Ry);
  /*//For acceleration based throttle
    int LyLevel = minThrottle;

    if( LyLevel + Ly < maxThrottle){
    if( LyLevel + Ly > minThrottle){
    LyLevel += Ly;
    }
    else{
    LyLevel = minThrottle;
    }
    }
    else{
    LyLevel = maxThrottle;
    }
    if (dataReceived[1] == 0) { //For position based throttle
    Ly = minThrottle;
    }
  */

  Serial.print("getPID");
  xOutput = getPID(0);
  yOutput = getPID(1);
  //zOutput = getPID(2);
  //Serial.print("xOutput: ");
  //Serial.println(xOutput);
  //Serial.print(" yOutput: ");
  //Serial.println(yOutput);

  /*
    throttle1 = throttleAll - xOutput + yOutput + zOutput + 60;
    throttle2 = throttleAll - xOutput - yOutput - zOutput + 60;
    throttle3 = throttleAll + xOutput + yOutput - zOutput + 60;
    throttle4 = throttleAll + xOutput - yOutput + zOutput + 60;

    throttle1 = Ly;
    throttle2 = Ly;
    throttle3 = Ly;
    throttle4 = Ly;

  */

  /*
    throttle1 = Ly - xOutput + yOutput;// + zOutput;
    throttle2 = Ly - xOutput - yOutput;// - zOutput;
    throttle3 = Ly + xOutput + yOutput;// - zOutput;
    throttle4 = Ly + xOutput - yOutput;// + zOutput;
  */
  Serial.print("Motor2");
  throttle1 = Ly + xOutput + yOutput;// + zOutput;BL
  throttle2 = Ly - xOutput - yOutput;// - zOutput;FR
  throttle3 = Ly - xOutput + yOutput;// - zOutput;FL
  throttle4 = Ly + xOutput - yOutput;// + zOutput;BR
  /*
    if(Ly == 1100 ){
    throttle1 =1100;
    throttle2 =1100;
    throttle3 =1100;
    throttle4 =1100;
    }
  */
  /*
    delayMotor =  millis() - millisMotor + delayMotor;
    if(delayMotor > 80){//Around 110ms delay
    power1 = throttle1;//sensitivity(throttle1, power1);
    power2 = throttle2;//sensitivity(throttle2, power2);
    power3 = throttle3;//sensitivity(throttle3, power3);
    power4 = throttle4;//sensitivity(throttle4, power4);

  */
  power1 = sensitivity(throttle1, power1);
  power2 = sensitivity(throttle2, power2);
  power3 = sensitivity(throttle3, power3);
  power4 = sensitivity(throttle4, power4);
  /*
    delayMotor = 0;
    }
    millisMotor = millis();
  */
  if ( power1 > maxThrottle) {
    power1 = maxThrottle;
  }
  if ( power2 > maxThrottle) {
    power2 = maxThrottle;
  }
  if ( power3 > maxThrottle) {
    power3 = maxThrottle;
  }
  if ( power4 > maxThrottle) {
    power4 = maxThrottle;
  }
  /*
    if ( power1 < minThrottle) {
    power1 = minThrottle;
    }
    if ( power2 < minThrottle) {
    power2 = minThrottle;
    }
    if ( power3 < minThrottle) {
    power3 = minThrottle;
    }
    if ( power4 < minThrottle) {
    power4 = minThrottle;
    }
  */
  Serial.print("Motor3");
  if (dataReceived[4] == 0) {

    power1 = nullThrottle;
    power2 = nullThrottle;
    power3 = nullThrottle;
    power4 = nullThrottle;
  }


  Serial.print("Motor4");
  motor1.writeMicroseconds(power1);
  motor2.writeMicroseconds(power2);
  motor3.writeMicroseconds(power3);
  motor4.writeMicroseconds(power4);

  Serial.print("power1,2,3,4: ");
  Serial.print(power1);
  Serial.print(", ");
  Serial.print(power2);
  Serial.print(", ");
  Serial.println(power3);
  Serial.print(", ");
  Serial.println(power4);
  Serial.print("throttle1,2,3,4: ");
  Serial.print(throttle1);
  Serial.print(", ");
  Serial.print(throttle2);
  Serial.print(", ");
  Serial.println(throttle3);
  Serial.print(", ");
  Serial.println(throttle4);

  //Serial.println(power3);
  //Serial.println(power4);
  /*
    Serial.println(xOutput);
    Serial.println(yOutput);
    Serial.println(zOutput);
    Serial.println(throttle4);
  */
#ifdef LCD_TRUE
  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Run");
  lcd.setCursor(4, 0);
  lcd.print("DEL");
  lcd.print(delayNRF);
  lcd.setCursor(11, 0);
  lcd.print("ERR#");
  lcd.print(errorCount);
  lcd.setCursor(0, 1);
  lcd.print("T3");
  lcd.print(power3);
  lcd.print(" T4");
  lcd.print(power4);
#endif

  Serial.println("voidLoop Finished\n");

}



float getPID(int i) {
  mpu.Execute();
  uint32_t currentTime;
  uint32_t timeChange;
  uint32_t previousTime;
  float angle;
  float setPoint;
  float error;
  float errorSum;
  float PIDi;
  float PIDp;
  float PIDd;
  float PIDnet;
  float previousError;


  if (i == 0) {
    previousError = x_previousError;
    previousTime = x_previousTime;
    Serial.print("timeP: ");
    Serial.println(previousTime);
    angle = mpu.GetAngX();
    setPoint = x_setPoint;
    errorSum = x_errorSum;
    PIDi = x_PIDi;
    PIDp = x_PIDp;
    PIDd = x_PIDd;
    PIDnet = x_PIDnet;
    /*Serial.println("X: ");
    Serial.println(angle);
    Serial.print("x_prevT: ");
    Serial.println(x_previousTime);
    Serial.print("x_setPoint: ");
    Serial.println(x_setPoint);
    Serial.print("x_errorSum: ");
    Serial.println(x_errorSum);
    Serial.print("x_PIDp,i,d: ");
    Serial.print(x_PIDp);
    Serial.print(" , ");
    Serial.print(x_PIDi);
    Serial.print(" , ");
    Serial.println(x_PIDd);
*/
  }
  if (i == 1) {
    previousError = y_previousError;
    previousTime = y_previousTime;
    angle = mpu.GetAngY();
    setPoint = y_setPoint;
    errorSum = y_errorSum;
    PIDi = y_PIDi;
    PIDp = y_PIDp;
    PIDd = y_PIDd;
    PIDnet = y_PIDnet;
    /*Serial.println("Y: ");
    Serial.println(angle);
    Serial.print("y_prevT: ");
    Serial.println(y_previousTime);
    Serial.print("y_setPoint: ");
    Serial.println(y_setPoint);
    Serial.print("y_errorSum: ");
    Serial.println(y_errorSum);
    Serial.print("y_PIDp,i,d: ");
    Serial.print(y_PIDp);
    Serial.print(" , ");
    Serial.print(y_PIDi);
    Serial.print(" , ");
    Serial.println(y_PIDd);
    */
  }
  if (i == 2) {
    previousTime = z_previousTime;
    angle = mpu.GetAngZ();
    setPoint = z_setPoint;
    errorSum = z_errorSum;
    PIDi = z_PIDi;
    PIDp = z_PIDp;
    PIDd = z_PIDd;
    PIDnet = z_PIDnet;
  }
  if (i == 3) {
    previousError = a_previousError;
    previousTime = a_previousTime;
    angle = bmp.readAltitude(calibPress);
    setPoint = altSetPoint;
    errorSum = a_errorSum;
    PIDi = a_PIDi;
    PIDp = a_PIDp;
    PIDd = a_PIDd;
    PIDnet = a_PIDnet;
  }


  //PID
  currentTime = millis();
  timeChange = currentTime - previousTime;
  if (angle < setPoint + angleSetPoint && angle > setPoint - angleSetPoint) {
    angle = setPoint;
    //errorSum = 0;
  }

  error = setPoint - angle;
  /*s
    if (error > maxError) {
    error = maxError;
    }
    if (error < maxError * -1) {
    error = maxError * -1;
    }
  */
  errorSum = errorSum + error;
  //error limit
  if (errorSum > maxErrorSum) {
    errorSum = maxErrorSum;
  }
  if (errorSum < maxErrorSum * -1) {
    errorSum = maxErrorSum * -1;
  }



  //Proportional
  PIDp = error * Kp;
  //Integral
  PIDi = Ki * errorSum;
  //Differential
  PIDd = Kd * ((error - previousError) / timeChange);

  PIDnet = PIDp + PIDi + PIDd;
  //PIDnet = PIDnet * PidFactor;

  if (PIDnet > PIDsetPoint) {
    PIDnet = PIDsetPoint;
  }
  if (PIDnet < (PIDsetPoint * -1)) {
    PIDnet = PIDsetPoint * -1;
  }



  if (i == 0) {
    x_previousTime = millis();
    x_previousError = error;
    x_errorSum = errorSum;
    x_PIDi = PIDi;
    x_PIDp = PIDp;
    x_PIDd = PIDd;
    x_PIDnet = PIDnet;
  }
  if (i == 1) {
    y_previousTime = millis();
    y_previousError = error;
    y_errorSum = errorSum;
    y_PIDi = PIDi;
    y_PIDp = PIDp;
    y_PIDd = PIDd;
    y_PIDnet = PIDnet;
  }
  if (i == 2) {
    z_previousTime = millis();
    z_previousError = error;
    z_errorSum = errorSum;
    z_PIDi = PIDi;
    z_PIDp = PIDp;
    z_PIDd = PIDd;
    z_PIDnet = PIDnet;
  }
  if (i == 3) {
    a_previousTime = millis();
    a_previousError = error;
    a_errorSum = errorSum;
    a_PIDi = PIDi;
    a_PIDp = PIDp;
    a_PIDd = PIDd;
    a_PIDnet = PIDnet;
  }
  return PIDnet;
}

int averageR;
int runs;
int totalR;
void getData() {
  int retries = 0;
  bool goodSignal = radio.testRPD();
  /*
      if(goodSignal==0){
        //Serial.println("Bad signal");
        radio.setPALevel(RF24_PA_MAX);
      }
      else{
         radio.setPALevel(RF24_PA_MIN);
      }
  */
  if ( radio.available()) // Check for incoming data from transmitter
  {
    radio.read( &dataReceived, sizeof(dataReceived) );
    newData = true;
  }
  if ( !newData) {
    //Serial.println("Couldnt find reciever radio");
  }
  newData = false;
  //showData();

}




void showData() {

  Serial.print("Data received ");
  Serial.print(dataReceived[0]);
  Serial.print(", ");
  Serial.print(dataReceived[1]);
  Serial.print(", ");
  Serial.print(dataReceived[2]);
  Serial.print(", ");
  Serial.println(dataReceived[3]);
  Serial.print(", ");
  Serial.println(dataReceived[4]);


}


//---BMP---
void calibration() {
  Serial.println("Calibrating Pa");
  for (int i = 0; i < calibLoop; i++) {
    calibPress += bmp.readPressure() / 100;
    delay(10);
  }
  calibPress = calibPress / calibLoop;
  Serial.print(calibPress);
  Serial.println("Pa calibrated");
}
void bmp280() {
  temp = bmp.readTemperature();
  pressure = bmp.readPressure() / 100;
  height = bmp.readAltitude(calibPress);

  /*
    Serial.print(F("Temperature = "));
    Serial.print(temp);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print(F("Approx altitude = "));
    Serial.print(height);
    Serial.println(" m");
  */


}

//---MPU---
void MPU6050() {
  mpu.Execute();
  /*
    Serial.print("GyroAngX = ");
    Serial.println(mpu.GetAngGyroX());
    Serial.print("GyroAngY = ");
    Serial.println(mpu.GetAngGyroY());
    Serial.print("GyroAngZ = ");
    Serial.println(mpu.GetAngGyroZ());
  */
}

//---VL53L0X---
void VL53L0X() {

  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  /*Serial.print("TOF height = ");
    Serial.print(sensor.readRangeSingleMillimeters());
    Serial.println(" mm");*/
  groundHeight = sensor.readRangeSingleMillimeters();

}


//---vSensor---
void vSensor() {
  vIn = (analogRead(A0) * 5.0) / 1024.0;
  vOut = vIn / (R2 / (R1 + R2));
  //Serial.print("Voltage: ");
  //Serial.println(vOut);
}


int sensitivity (int throttleTarget, int throttleCurrent) {

  if (throttleCurrent < throttleTarget) {
    if (throttleCurrent + powerInc < throttleTarget) {
      throttleCurrent += powerInc;
    }
    else {
      throttleCurrent = throttleTarget;
    }
  }
  else if (throttleCurrent > throttleTarget) {
    if (throttleCurrent - powerInc > throttleTarget) {
      throttleCurrent -= powerInc;
    }
    else {
      throttleCurrent = throttleTarget;
    }
  }
  else {
    throttleCurrent = throttleTarget;
  }
  return throttleCurrent;
}
