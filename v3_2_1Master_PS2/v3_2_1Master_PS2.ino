#include <PS2X_lib.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <printf.h>


//---NRF24L01---
#define CE_PIN 7
#define CSN_PIN 8
const byte slaveAddress[5] = {'R','x','A','A','A'};
//const byte masterAddress[5] = {'T','X','A','A','A'};
int dataToSend[5];

RF24 radio(CE_PIN, CSN_PIN);
bool newData = false;
float currentMillis;
float prevMillis;
float txIntervalMillis;
int i = 1;


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
LiquidCrystal_I2C      lcd(I2C_ADDR, En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

//---PS2---
#define clk 5
#define cmd 2
#define att 6
#define dat 3
PS2X ps2x;
int error = 0; 
bool motorRun = false;
bool hoverRun = false;
bool boostRun = false;
int square1 = 0;
bool triangle1 = false;
float Lx = 0; 
float Ly = 0;
bool lButton = false; 
float Rx = 0;
float Ry = 0;
int RxNew = 0;
int RyNew = 0;
int LyNew = 0;
int LxNew = 0;
bool rButton = false;  
int centreRange = 15;
float rateOfChange = 1;
float rocLow = 2;
float rocHigh = 100;

int t_0=0;
int t_1=0;
int tSet=5;//120millis per cycle

void setup() {
    Serial.begin(115200);
    printf_begin();
    
  //---LCD--- 
    lcd.begin (16,2);
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);
    lcd.setCursor(0,0);
    lcd.print("Master");
    lcd.setCursor(0,1);
    lcd.print("Starting");
    Serial.println("Master Starting");
    //lcd.clear();

 //---NRF24L01---   
    radio.begin();
    radio.setDataRate( RF24_250KBPS );
    radio.setRetries(15,15); // delay, count
    radio.openWritingPipe(slaveAddress);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening();

Serial.println(radio.isChipConnected());
radio.printDetails();
    send(); 

  //---PS2---
    if(ps2x.config_gamepad(clk,cmd,att,dat, true, true) == 1||ps2x.config_gamepad(clk,cmd,att,dat, true, true) == 2||ps2x.config_gamepad(clk,cmd,att,dat, true, true) == 3)
    {
      Serial.println("Controller error");
    }

    Serial.println("Setup Finished");   
}

void loop() {
  Serial.println("Before update PS2");
  //---PS2---
  updatePS2();
  if (triangle1 == 0){
    rateOfChange = rocLow;
  }
  else{
    rateOfChange = rocHigh;
  }
Serial.println("Before send");



  send();
Serial.println("Before LCD");

  //---LCD---
  char buffer1[7] ="";
  lcd.setCursor(0,0);
  switch(square1){
    case 0:
      sprintf(buffer1, "Ly:%04d",(int)LyNew); break;
    case 1:
      sprintf(buffer1, "Lx:%04d",(int)LxNew); break;
    case 2:
      sprintf(buffer1, "Ry:%04d",(int)RyNew); break;
    case 3:
      sprintf(buffer1, "Rx:%04d",(int)RxNew); break;
   }
  lcd.print(buffer1);
  lcd.setCursor(7,0);
  char buffer3[9] = "";
  
  lcd.print(buffer3);
  char buffer2[16] ="";
  sprintf(buffer2,"M:%d S:%d H:%d B:%d", motorRun, triangle1, hoverRun, boostRun);
  lcd.setCursor(0,1);
  lcd.print(buffer2);


  Serial.println("voidLoop Finished\n");
  t_0 = millis();
}


void updatePS2(){
  ps2x.read_gamepad();
  //current uses Start, Buttons, Triggers, Square, Triangle
  if(ps2x.ButtonPressed(PSB_GREEN))
    if(triangle1 == false){
      triangle1 = true;
    }
    else{
      triangle1 = false;
    }         
  if(ps2x.ButtonPressed(PSB_PINK)) {
    square1++;
         if (square1 > 3){
          square1 = 0; 
         }
  }
  if(ps2x.ButtonPressed(PSB_START)) {
    if(motorRun == false){
      motorRun = true;
    }
    else{
      motorRun = false;
    }
    Serial.print("motor: ");
    Serial.println(motorRun);
  }
  if(ps2x.ButtonPressed(PSB_L1)||ps2x.ButtonPressed(PSB_R1)) {
    if(hoverRun == false){
      hoverRun = true;
    }
    else{
      hoverRun = false;
    }
    Serial.print("hover: ");
    Serial.println(hoverRun);
  }
  if(ps2x.ButtonPressed(PSB_L2)||ps2x.ButtonPressed(PSB_R2)) {
    if(boostRun == false){
      boostRun = true;
    }
    else{
      boostRun = false;
    }
    Serial.print("run: ");
    Serial.println(boostRun);
  }
  
  t_1 = millis()-t_0+t_1;
 
  //if (t_1>tSet){
  Ly = map(ps2x.Analog(PSS_LY),0,255,255,0)*0.01;
  Serial.println(Ly);
  Ly = pow(Ly,10);//max 11625.23, min 0, middle 10.92
  Serial.println(Ly);
  Ly = map(Ly,10.92,11625.23,1100,1350);//For position based throttle
  //Ly = map(Ly,0,255,-100,100); //For acceleration based throttle
  Serial.println(Ly);

  Lx = ps2x.Analog(PSS_LX);
  Lx = map(Lx,0,255,-100,100);

  Ry = ps2x.Analog(PSS_RY);
  Ry = map(Ry,0,255,20,-20);

  Rx = map(ps2x.Analog(PSS_RX),0,255,255,0);
  Rx = map(Rx,0,255,20,-20);

  if(Lx < centreRange && Lx > centreRange*-1){
    Lx = 0;
  }
  if(Ry < centreRange && Ry > centreRange*-1){
    Ry = 0;
  }
  if(Rx < centreRange && Rx > centreRange*-1){
    Rx = 0;
  }
  t_1=0;
  t_0 = millis();
  
  //}


  
/*
  LxNew = sensitivity(Lx,0);
  LyNew = sensitivity(Ly,1);
  RxNew = sensitivity(Rx,2);
  RyNew = sensitivity(Ry,3);
  */
  LxNew = Lx;
  LyNew = Ly;
  RxNew = Rx;
  RyNew = Ry;
  Serial.println(LyNew);
  
}

float sensitivity (int joy, int joyWhich){
  float joyNew;
  
  if (joyWhich == 0){
    joyNew = LxNew;
  }
  else if (joyWhich == 1){
    joyNew = LyNew;      
  }
  else if(joyWhich == 2){
    joyNew = RxNew;
  }
  else{
    joyNew = RyNew;
  }
  
  
  if(joy>0)
    if (joyNew + rateOfChange < joy){
      joyNew += rateOfChange;
    }
    else {
      joyNew = joy;
    }
  else if(joy<0){
    if (joyNew - rateOfChange > joy){
      joyNew -= rateOfChange;
    }
    else {
      joyNew = joy;
    }
  }
  else{
    joyNew=0;
  }
  return joyNew;
  }


void send() {
    
    bool rslt;
    Serial.println("Before rslt");
    rslt = radio.write( &dataToSend, sizeof(dataToSend) );
    Serial.print("Data Sent ");
    //Serial.print(dataToSend);
    if (rslt) {
        Serial.println("  Acknowledge received");
        updateMessage();
    }
    else {
        Serial.println("  Tx failed");
    }

    prevMillis = millis();
    
}



void updateMessage() {
    dataToSend[0] = LxNew;
    dataToSend[1] = LyNew;
    dataToSend[2] = RxNew;
    dataToSend[3] = RyNew;
    dataToSend[4] = motorRun;
    //dataToSend[5] = hoverRun;
    //dataToSend[6] = boostRun;

    
}
