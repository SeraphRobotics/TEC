
//**************************************************
// AUTO uv Controller Revision 3
// Sept 15, 2015
//
//******************************************************


#include <Wire.h>
#include <OneWire.h>
#include <EEPROM.h>
#include "TimerOne.h"
#include <PID_v1.h>

OneWire  ds(2);  // on digital pin 2 

#define ENC_A 5
#define ENC_B 6
#define ENC_PORT PIND
#define FAN 10
#define pDIR 4
#define pPWM 3
#define pRST 12
#define pFF1 8
#define pFF2 7
#define BLUE A0
#define GREEN A1
#define RED A2

#define LED_ON LOW
#define LED_OFF HIGH



//************************* VARIABLE DECLARATION ********************************
float cTemperature;
float celsius;
byte i;
byte data[12];
char textData[21];        //char array to display text on LCD
String tmp;
int setPoint1;
int setPointValue;
int setPointTemp;
byte negFlag;
long tempValue1;
long tempValue2;
long tempValue3;
int tempReadStatus=0;

int encoderPos=0;
int encoderPinALast=0;
unsigned long currentTime;
unsigned long loopTime;
unsigned long loopTime2;
byte stepValue=1;
int encoderA;
int encoderB;
int measFlag;
int measFlag1;
int encFlag;
unsigned long encoderCount;
unsigned long measureCount;
double Input;
double Output;
double Setpoint;

int minVal=0;
int maxVal=255;
float percent=0;
float percent10;
float stepsize=1.0;
float scale = 1.0;//1.275;


char bufferLCD[17];

PID tecPID(&Input, &Output, &Setpoint, 2, 5, 1, REVERSE);   //PID (INPUT, OUTPUT, SETPOINT, Kp, Ki, Kd, DIRECTION)

void setup()
{
 
  
 pinMode(ENC_A, INPUT);        //encoder input A
 pinMode(ENC_B, INPUT);        //encoder input B
 pinMode(FAN, OUTPUT);        //main fan control
 pinMode(pDIR,OUTPUT);        //direction of controller board
 pinMode(pPWM,OUTPUT);        //PWM output for controller board
 pinMode(pFF1,INPUT_PULLUP);    //fault pin 1 of controller
 pinMode(pFF2,INPUT_PULLUP);    //fault pin 2 of controller
 pinMode(pRST,OUTPUT);          //controller reset
 pinMode(BLUE, OUTPUT);          //LCD BLUE LED
 pinMode(GREEN, OUTPUT);          //LCD GREEN LED
 pinMode(RED, OUTPUT);          //LCD RED LED
 
 digitalWrite(pRST,HIGH);        //hold controller in reset until complete initilize
 digitalWrite(BLUE,LED_OFF);            //turn off display background
 digitalWrite(GREEN,LED_OFF);
 digitalWrite(RED,LED_OFF);
 
 Serial.begin(9600);      //start serial port
 Wire.begin();            //start I2C
 TWBR = 720;
 delay(200);              //startup delay to stabilize
 
 //**************** get stored set point value *********************************
 setPointValue = EEPROM.read(0x00);
 setPointValue = setPointValue * 100;
 setPoint1 = EEPROM.read(0x01);
 setPointValue = setPointValue + setPoint1;
 negFlag = EEPROM.read(0x02);
 if (negFlag)
 {
   setPointValue = setPointValue * (-1);
 }
  
 if (EEPROM.read(0x00) >80)            //if eeprom empty or value larger than 80
 {
   setPointValue = 8000;            //set max value for setpoint
 }
  
 
 digitalWrite(BLUE,LED_ON);            //turn on display background
 digitalWrite(GREEN,LED_ON);
 digitalWrite(RED,LED_ON);
 
 LCD_WelcomeScreen();             //lcd welcome screen 
 LCD_DataScreen();                //disply initial data
 
  Timer1.initialize(1000);         // initialize timer1, and set a 0.001 second period
 Timer1.attachInterrupt(systemTimer);
 
 digitalWrite(FAN,HIGH);            //turn on main fan
 digitalWrite(pRST,LOW);            //clear controller reset
 digitalWrite(pDIR,HIGH);           //set direction of motor driver (high == cool)          
 analogWrite(pPWM,0);                //initial PWM start up is off
 
 Input = -4.0;                      //set initial PID Input value low so PID doesnt start until gets temperature
 Setpoint = setPointValue;
 Setpoint = Setpoint/100;               //initialize current setpoint of PID
 tecPID.SetMode(AUTOMATIC);          //set PID to automatic mode
 
 
}
void loop()
{
  
  if (encFlag == 1)                      //update encoder value - display new setpoint if decoder change detected
  {
  
 //   Serial.println(setPoint);
    displaySetPoint();
    encFlag=0;
    //Setpoint = setPointValue;
    //Setpoint = Setpoint/100;
  }
  if(Serial.available() >= 1){
	setPointValue = Serial.read();

  }
  
  
  if (setPointValue > maxVal){setPointValue = maxVal;}
  if (setPointValue < minVal){setPointValue = minVal;}
  Setpoint = setPointValue*stepsize*scale;
  percent = setPointValue*0.39215;
  
  //Serial.print("Setpoint = ");Serial.println(Setpoint);
  Serial.print("\nSet Value = ");Serial.println(setPointValue);
  Serial.print("Percent = ");Serial.println(percent);
  displaySetPoint();
  //delay(1000);

  
  if (Setpoint >= 200)                //set heat or cool based on setpoint value
  {
    digitalWrite(pDIR,LOW);           //set direction of motor driver (low = heat)
    digitalWrite(GREEN,LED_OFF);
    digitalWrite(RED,LED_ON);          //turn screen RED to indicate heat mode
    digitalWrite(BLUE,LED_OFF);
  }
  else if(Setpoint >100)
  {
    digitalWrite(pDIR,LOW);           //set direction of motor driver (low = heat)
    digitalWrite(GREEN,LED_ON);
    digitalWrite(RED,LED_OFF);          //turn screen RED to indicate heat mode
    digitalWrite(BLUE,LED_OFF);
  }
  
  else
  {
    digitalWrite(pDIR,LOW);           //set direction of motor driver (high == cool)
    digitalWrite(GREEN,LED_OFF);
    digitalWrite(RED,LED_OFF);
    digitalWrite(BLUE,LED_ON);          //turn screen blue to indicate cooling mode
  }
  
  analogWrite(pPWM,Setpoint);        //set PWM value to OUTPUT vallue of PID
  
}

//************************************* FUNCTIONS **********************************

void systemTimer (void)
{
  encoderA = digitalRead(ENC_A);
  encoderB = digitalRead(ENC_B);
  
  if ((!encoderA) && (encoderPinALast) && (!encFlag))
  {
    if (encoderB)
        {
         if (setPointValue > 1800)
          {
            stepValue = 25;
          }
          else
          {
            stepValue = 10;
          }
         setPointValue -=stepValue;
         if (setPointValue < -400)
         {
           setPointValue = -400;
         }
        }
     else
        {
          if (setPointValue >= 1800)
          {
            stepValue = 25; 
          }
          else
          {
            stepValue = 10;
          }
          setPointValue +=stepValue;
          if (setPointValue > 8000)
          {
            setPointValue = 8000;
          }
        }
        encFlag=1;
  }
   encoderPinALast = encoderA;
 
  measureCount++;
  if (measureCount>=1000)
  {
    measFlag1 = 1;
    measureCount=0;
  }
  else if (measureCount>=100)
  {
    measFlag=1;
  }
  
 
}


//****************** READ TEMPERATURE SENSOR ********************************************
float getTemperature()
{
  if (tempReadStatus == 0)
  {
    ds.reset();
    ds.write(0xCC);
    ds.write(0x44, 1);        // start conversion
    measureCount = 0;
    tempReadStatus = 1;
  }
  else if (tempReadStatus == 1 && measFlag1 == 1)
  {  
    ds.reset();
    ds.write(0xCC);
    ds.write(0xBE);         // Read data
    for ( i = 0; i < 9; i++)
     {          
       data[i] = ds.read(); //read 9 bytes
     }
    int16_t raw = (data[1] << 8) | data[0];
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
 
    celsius = (float)raw / 16.0;
    measureCount=0;
    measFlag1 = 0;
    tempReadStatus = 0;
  }  
    return celsius;
  
}

void LCD_Clear()
{
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);        //hex code for clear screen (FE51)
    delay(15);
    Wire.write(0x51);
    Wire.endTransmission();
    delay(30);                    //200mS delay to complete sequence
}    

void LCD_BacklightOff()                 //turn off backlight
{
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);        //hex code for cursor home (FE42)
    Wire.write(0x42);
    Wire.endTransmission();
    delay(20);
}
void LCD_Contrast(int contrast)            //contrast 1 - 50
{
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);       //hex code for contrast (FE52)
    Wire.write(0x52);
    Wire.write(contrast);   //value  1 to 50
    Wire.endTransmission();
    delay(20);                    //100mS delay to complete sequence
}

void LCD_CursorSet(int row, int pos)
{
    if (row == 1)
    {
        pos = pos - 1;
    }
    else
    {
        pos = pos + 63;
    }
    Wire.beginTransmission(0x28);    
    Wire.write(0xFE);        //hex code for cursor position (FE45)
  //  delay(10);
    Wire.write(0x45);
   // delay(30);
    Wire.write(pos);         //value Row 1 or Row 2: Pos 1 to 20
    Wire.endTransmission();
    delay(5);                    //100mS delay to complete sequence
}
void LCD_MoveCursorRight()              //move cursor right one space
{
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);        //hex code for cursor home (FE4A)
    Wire.write(0x4A);
    Wire.endTransmission();
    delay(20);
}
void LCD_MoveCursorLeft()              //move cursor left one space
{
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);        //hex code for cursor home (FE49)
    Wire.write(0x49);
    Wire.endTransmission();
    delay(20);
}    
void LCD_UnderlineCursor(int lcdStatus)        //set to 1=on or 0=off
{
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);        //hex code for cursor home (FE47 or FE48)
    if (lcdStatus == 1)
    {
        Wire.write(0x47);    //turn cursor on
    }
    else
    {
        Wire.write(0x48);    //turn cursor off
    }
    Wire.endTransmission();    
    delay(20);                   //50mS delay to complete sequence
}    
void LCD_BlinkCursor(int lcdStatus)           //set to 1=on or 0=off
{
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);        //hex code for cursor blink (FE4B or FE4C)
    if (lcdStatus == 1)
    {
        Wire.write(0x4B);    //turn blink on
    }
    else
    {
        Wire.write(0x4C);    //turn blink off
    }
    Wire.endTransmission();    
    delay(20);                    //10mS delay to complete sequence
}

void displayText(String lcdText)
{
  for (int p=0; p<21;p++)
  {
   textData[p]=' ';
  }
  int len = lcdText.length();
  lcdText.toCharArray(textData,len+1);
// Wire.beginTransmission(0x28);
  for (int c=0;c<len;c++)
  {
    Wire.beginTransmission(0x28);
    Wire.write(textData[c]);
    Wire.endTransmission();
  }
 // Wire.endTransmission();
 
}

void displayLine(void)
{
  Wire.beginTransmission(0x28);
  Wire.write(bufferLCD);
  Wire.endTransmission();
  delay(10);
}

void LCD_WelcomeScreen()
{
  String line1 = "Seraph  Robotics";
  String line2 = "AutoUV 3.0";
  LCD_Contrast(45);                        //set full contrast
  LCD_Clear();                            //clear screen
  LCD_BlinkCursor(0);                      //turn off blinking cursor
  LCD_UnderlineCursor(0);                  //turn off underline
  line1.toCharArray(bufferLCD,16);
  LCD_CursorSet(1,1);
  displayLine();
  line2.toCharArray(bufferLCD,16);
  LCD_CursorSet(2,3);
  displayLine();
  delay(2000);
  LCD_Clear();
  delay(200);
}

void LCD_DataScreen()
{
  LCD_CursorSet(1, 1);
  displayText("Intensity: ");
  delay(50);
  LCD_CursorSet(1, 7);
  //displayText("--.-");
  delay(200);
  LCD_CursorSet(2, 1);
  displayText("--.-%");
  displaySetPoint();
}

void displayTemperature()
{
  tmp = "";
  tmp=String(int(cTemperature))+ "." + String(getDecimal(cTemperature));
  tmp += " C ";
 
  //LCD_CursorSet(1, 7);
  //displayText(tmp);
  
}

void displaySetPoint()
{
  tmp = "";
  negFlag=0;
  setPointTemp = setPointValue;
  if (setPointValue < 0)
  {
    setPointTemp = setPointValue * -1;
    negFlag = 1;
  }
//  Serial.print("Setpoint= ");Serial.println(setPointValue);
  int sp = (setPointTemp/100);    //acquire whole digits
  int dp = setPointTemp - (sp*100);        //acquire decimal value
  
 // Serial.println(sp);
 // Serial.println(dp);

  if (negFlag)
  {
    tmp = "-";
  }
  
    tmp +=String(sp)+ "." + String(dp);

  tmp += "% ";
  
  LCD_CursorSet(2,10);
  displayText(tmp);


  EEPROM.write(0x00,sp);          //store set point value whole number
  EEPROM.write(0x01,dp);          //store set point value decimal
  EEPROM.write(0x02,negFlag);          //store set point polarity
 
}

long getDecimal(float val)
{
 
  tempValue1 = int(val) * 10L;
  tempValue2 = val * 10L;
  tempValue3 = tempValue2 - tempValue1;
  return abs(tempValue3);
}




