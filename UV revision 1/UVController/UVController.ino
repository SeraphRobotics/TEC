
//**************************************************
// TEC Controller Revision 1
// February 24, 2015
//
//******************************************************


#include <Wire.h>
#include <EEPROM.h>
#include "TimerOne.h"


#define ENC_A 5
#define ENC_B 6
#define ENC_PORT PIND
#define FAN 10
#define pDIR 4
#define pPWM 3
#define pRST 12
#define pFF1 8
#define pFF2 7




//************************* VARIABLE DECLARATION ********************************
float cTemperature;
float celsius;
byte i;
byte data[12];
char textData[21];        //char array to display text on LCD
String tmp;
long tempValue1;
long tempValue2;
long tempValue3;


int encoderPos=0;
int encoderPinALast=0;


int setPointValue;
int minVal=0;
int maxVal=200;
float percent=0;
float percent10;
float stepsize=0.5;
float scale = 2.55;//1.275;

int encoderA;
int encoderB;
int encFlag;
unsigned long encoderCount;
unsigned long measureCount;

int Setpoint;

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
 
 digitalWrite(pRST,HIGH);        //hold controller in reset until complete initilize
 
 Serial.begin(9600);      //start serial port
 Wire.begin();            //start I2C
 delay(100);              //startup delay to stabilize
 
 //**************** get stored set point value *********************************
 setPointValue = EEPROM.read(0x00);

 if (setPointValue > maxVal)            //if eeprom empty or value larger than 18.4
 {
   setPointValue =  maxVal;            //set max value for setpoint
 }
 
 LCD_WelcomeScreen();             //lcd welcome screen 
 LCD_DataScreen();                //disply initial data

 Timer1.initialize(1000);         // initialize timer1, and set a 0.001 second period
 Timer1.attachInterrupt(systemTimer);
 
 digitalWrite(FAN,HIGH);            //turn on main fan
 digitalWrite(pRST,LOW);            //clear controller reset
 digitalWrite(pDIR,HIGH);           //set direction of motor driver           
 analogWrite(pPWM,0);                //initial PWM start up is off
 
}
void loop()
{
  
    if(Serial.available() >= 1){
        setPointValue = Serial.read();

    }
  
  if (encFlag == 1)                      //update encoder value - display new setpoint if decoder change detected
  {
  
    
    encFlag=0;

  }
  
  if (setPointValue > maxVal){setPointValue = maxVal;}
  if (setPointValue < minVal){setPointValue = minVal;}
  Setpoint = setPointValue*stepsize*scale;
  
  
  percent = setPointValue*stepsize;
  
  //Serial.print("Setpoint = ");Serial.println(Setpoint);
  Serial.print("\nSet Value = ");Serial.println(setPointValue);
  Serial.print("Percent = ");Serial.println(percent);
  displaySetPoint();
  //delay(1000);
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
         setPointValue --;
         if (setPointValue < minVal)
         {
           setPointValue = minVal;
         }
        }
        else
        {
         setPointValue ++;
          if (setPointValue > maxVal)
          {
            setPointValue = maxVal;
          }
        }
        encFlag=1;
  }
   encoderPinALast = encoderA;
}


//****************** READ TEMPERATURE SENSOR ********************************************

void LCD_Clear()
{
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);        //hex code for clear screen (FE51)
    delay(20);
    Wire.write(0x51);
    Wire.endTransmission();
    delay(20);                    //200mS delay to complete sequence
}    
void LCD_Backlight(int intensity)           //intensity 1 - 8
{
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);        //hex code for backlight on (FE41)
    Wire.write(0x41);
    Wire.endTransmission();
    delay(20);                    //20mS delay to complete sequence
    Wire.beginTransmission(0x28);
    Wire.write(0xFE);        //hex code for backlight level on (FE53)
    Wire.write(0x53);
    Wire.write(intensity);   //value  1 to 8
    Wire.endTransmission();
    delay(20);                   //100mS delay to complete sequence
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
    delay(100);                    //100mS delay to complete sequence
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
  Wire.beginTransmission(0x28);
  for (int c=0;c<len;c++)
  {
    Wire.beginTransmission(0x28);
    Wire.write(textData[c]);
    Wire.endTransmission();
  //  delay(5);
  }
 
}

void LCD_WelcomeScreen()
{
  LCD_Backlight(8);                        //set full brightnest
  LCD_Contrast(50);                        //set full contrast
  LCD_Clear();                            //clear screen
  LCD_BlinkCursor(0);                      //turn off blinking cursor
  LCD_UnderlineCursor(0);                  //turn off underline
  LCD_CursorSet(1, 3);
  displayText("Seraph  Robotics");
  LCD_CursorSet(2, 3);
  displayText("Automatic UV 1.1");
  delay(2000);
  LCD_Clear();
  delay(200);
}

void LCD_DataScreen()
{
  LCD_CursorSet(1, 1);
  displayText("Intensity: ");
  delay(50);
  LCD_CursorSet(2, 1);
  displayText("--.-%");
  delay(200);
  //LCD_CursorSet(2, 1);
  //displayText("Set Temp:");
  displaySetPoint();
}

void displaySetPoint()
{
  tmp = "";
 // Serial.print("Setpoint= ");Serial.println(setPointValue);
  percent10=percent*10;
  int sp = (percent10/10);    //acquire whole digits
  int dp = percent10 - (sp*10);        //acquire decimal value
  
 // Serial.println(sp);
 // Serial.println(dp);
  if (percent < 0 && percent >-10)
  {
    dp=dp*(-1);
    tmp="-"+String(sp)+ "." + String(dp);
  }
  else if (percent < -9)
  {
    sp=sp*(-1);
    dp=dp*(-1);
    tmp="-"+String(sp)+ "." + String(dp);
  }
  else
  {
    tmp=String(sp)+ "." + String(dp);
  }
  tmp += "% ";
  
  LCD_CursorSet(2,1);
  displayText(tmp);

    EEPROM.write(0x00,setPointValue);          //store set point value (x10 of actual setpoint)
  
 
}

long getDecimal(float val)
{
 
  tempValue1 = int(val) * 10L;
  tempValue2 = val * 10L;
  tempValue3 = tempValue2 - tempValue1;
  return abs(tempValue3);
}




