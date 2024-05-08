#include "ACS712.h"
#include "LiquidCrystal_I2C.h"
#include <Wire.h>
#include <FlashStorage_STM32.h>
#include <DHT22.h>
#include <GP2YDustSensor.h>
#include <stdio.h>
#include <AT24C512C.h>
#include <STM32RTC.h>
// #include <rtc.h>
#include <RTClib.h>

#define EEPROM_ADDRESS       0x50
#define MAX_RECORDS          458
#define BYTES_IN_EACH_RECORD 152

#define VCS            PA1
#define VCS1           PB0
#define VCS2           PA7
#define ACCS           PA0
#define ACCS1          PA4
#define ACCS2          PA6
#define RLY10          PB4
#define RLY11          PB2
#define RLY12          PA5
#define RLY20          PB5
#define RLY21          PB12
#define RLY22          PB15
#define LOAD_SW_1      PB10
#define LOAD_SW_2      PB13
#define LOAD_SW_3      PB14
#define DHT_22         PB1
#define SHARP_LED_PIN  PB3  // Sharp Dust/particle sensor Led Pin
#define SHARP_VO_PIN   PA8   // Sharp Dust/particle analog out pin used for reading 
#define BT             PA9

#define No_Of_Samples 1000

typedef struct{
  float masterOpenCircuitVoltage;
  float slave1OpenCircuitVoltage;
  float slave2OpenCircuitVoltage;
  float masterShortCircuitCurrent;
  float slave1ShortCircuitCurrent;
  float slave2ShortCircuitCurrent;
  // float masterLoadVoltage;
  // float masterLoadCurrent;
  // float slave1LoadVoltage;
  // float slave1LoadCurrent;
  // float slave2LoadVoltage;
  // float slave2LoadCurrent;
  // float masterMaxPower;
  // float slave1MaxPower;
  // float slave2MaxPower;
  // float masterMaxImpedence;
  // float slave1MaxImpedence;
  // float slave2MaxImpedence;
  float temperature;
  float humidity;
  float dustSensor;
  uint8_t day;
  uint8_t month;
  uint16_t year;
  uint8_t hour;
  uint8_t mins;
  uint8_t sec; 
  uint8_t padBytes;
}Parameters_t;
Parameters_t parameter;

typedef struct {
  float pow;
  float imp;
}mppt_res;


String st;
int add_counter;
RTC_DS3231 rtc;
char t[32];

LiquidCrystal_I2C   lcd(0x27,16,2);
HardwareSerial      BTSerial(PA3, PA2);  //RX,TX
HardwareSerial      PCSerial(PA12,PA11); //RX,TX
GP2YDustSensor      dustSensor(GP2YDustSensorType::GP2Y1010AU0F, SHARP_LED_PIN, SHARP_VO_PIN);
DHT22               dht22(DHT_22);
// STM32RTC&           rtc = STM32RTC::getInstance();


int      recNum = 0;
uint16_t LastRecNum= 0;
uint16_t recordNum= 0;
uint8_t  RecordsCounterLo = 0;
uint8_t  RecordsCounterHi = 0;
uint16_t RecordsCounterAddressLo = 0xFFFE; // 65534 : 1 byte
uint16_t RecordsCounterAddressHi = 0xFFFF; // 65535 : 1 byte

int count=0;
char data[4];

int i=0, j=0, k=0;

int counter;
// int   i=0;
int   adc_value = 0;
float Vo = 0.00;
float Ao = 0.00;
float ACS_Sensitivity = 0.066;
int address = 0;

 
// Floats for ADC voltage & Input voltage
float V_ADC = 0.0;
float V_IN = 0.0;
 
// Float for Reference Voltage
float ref_voltage = 3.3;

void EEPROMWrite(int recordNum);
uint16_t ReadRecordNumber(uint16_t loByte, uint16_t hiByte);
void EEPROMRead(void);
float sensor(int sensor, int seq);


 
void setup()
{
  // rtc.setClockSource(STM32RTC::Source_Clock::LSI_CLOCK);
  // rtc.begin();
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__),F(__TIME__)));
  // rtc.adjust(DateTime(2024, 4, 28, 7, 14, 0));
  //================  RTC initial time set (only once) ================//

      // rtc.setEpoch(1711371700);

  //===================================================================//      

  BTSerial.begin(9600);
  PCSerial.begin(9600);
  Wire.begin();

  PCSerial.println("This pc serial");
  PCSerial.println("DC Voltage Test");


  analogReadResolution(12);
  attachInterrupt(digitalPinToInterrupt(BT), BlueTothDownloadData, LOW);
  pinMode(RLY10,OUTPUT);
  pinMode(RLY11,OUTPUT);
  pinMode(RLY12,OUTPUT);
  pinMode(RLY20,OUTPUT);
  pinMode(RLY21,OUTPUT);
  pinMode(RLY22,OUTPUT);
  pinMode(LOAD_SW_1,OUTPUT);
  pinMode(LOAD_SW_2,OUTPUT);
  pinMode(LOAD_SW_3,OUTPUT);
  pinMode(PC13,OUTPUT);
  pinMode(BT,INPUT_PULLUP);

  for(i=0; i<20; i++){
    digitalWrite(PC13,HIGH);
    delay(100);
    digitalWrite(PC13,LOW);
    delay(100);  
  }

 
  // parameter.day = rtc.getDay();
  // parameter.month = rtc.getMonth();
  // parameter.year = rtc.getYear();
  // parameter.hour = rtc.getHours();
  // parameter.hour = parameter.hour + 5; // GMT Time + 5 = Local Pakistan Time
  // parameter.mins = rtc.getMinutes();
  // parameter.sec = rtc.getSeconds();
  // PCSerial.printf("Current Time: %d:%d:%d\n",parameter.hour,parameter.mins,parameter.sec);
  // PCSerial.printf("Current Date: %d:%d:%d\n",parameter.day,parameter.month,parameter.year);

  lcd.init();                      // initialize the lcd 
  lcd.backlight();

  digitalWrite(RLY10,LOW);
  digitalWrite(RLY11,LOW);
  digitalWrite(RLY12,LOW);
  digitalWrite(RLY20,HIGH);
  digitalWrite(RLY21,HIGH);
  digitalWrite(RLY22,HIGH);
  counter = 0;
  add_counter=1;
  dustSensor.begin();

  //====================================================================
  // TO RESET THE EEPROM
        
  // EEPROM512.write(RecordsCounterAddressLo, 0);
  // EEPROM512.write(RecordsCounterAddressHi, 0);
  // while(1);

  //====================================================================

  LastRecNum = ReadRecordNumber(RecordsCounterAddressLo,RecordsCounterAddressHi);
  PCSerial.printf("Rec Num: %d\n",LastRecNum);

  recordNum = LastRecNum;  
}

 
void loop()
{
  DateTime now = rtc.now();
  while(count++ < 10)
  {
    digitalWrite(PC13,HIGH);
    delay(100);
    digitalWrite(PC13,LOW);
    delay(100);
  }
  count=0;
  float var;
  float var1;
  
  // ==============OPEN CIRCUIT VOLTAGE CALCULATIONS==============

  lcd.setCursor(0,0);
  lcd.print("Open circut Volts");
  delay(5000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("OV1 =");
  lcd.setCursor(6,0);
  var=sensor(0,1);
  lcd.print(var);

  parameter.masterOpenCircuitVoltage = var ;
  
  delay(5000);
  lcd.clear();
  lcd.print("OV2 =");
  lcd.setCursor(6,0);
  var=sensor(0,2);
  lcd.print(var);

  parameter.slave1OpenCircuitVoltage = var ;
  
  delay(5000);
  lcd.clear();
  lcd.print("OV3 =");
  lcd.setCursor(6,0);
  var=sensor(0,3);
  lcd.print(var);

  parameter.slave2OpenCircuitVoltage = var ;

  delay(5000);
  lcd.clear();

  // ==========SHORT CIRUCUIT CURRENT CALCULATIONS============== 

  lcd.setCursor(0,0);
  lcd.print("Short circuit I");
  delay(5000);
  lcd.clear();
  digitalWrite(RLY10,HIGH);
   
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SC1 =");
  lcd.setCursor(6,0);
  var=sensor(1,1);
  lcd.print(var);

  parameter.masterShortCircuitCurrent = var ;

  delay(5000);
  digitalWrite(RLY10,LOW);
  digitalWrite(RLY11,HIGH);
  lcd.setCursor(0,0);
  lcd.print("SC2 =");
  lcd.setCursor(6,0);
  var=sensor(1,2);
  lcd.print(var);

  parameter.slave1ShortCircuitCurrent = var ;
  
  delay(5000);
  digitalWrite(RLY11,LOW);
  digitalWrite(RLY12,HIGH);
  lcd.setCursor(0,0);
  lcd.print("SC3 =");
  lcd.setCursor(6,0);
  var=sensor(1,3);
  lcd.print(var);
  
  parameter.slave2ShortCircuitCurrent = var ;

  digitalWrite(RLY12,LOW);
  delay(3000);
  lcd.clear();



  // NOW BELOW THE CODE IS ABOUT THE LOAD CIRCUIT VOLTAGES AND THE CURRENT //

  // digitalWrite(LOAD_SW_1,HIGH);
  // digitalWrite(LOAD_SW_2,HIGH);
  // digitalWrite(LOAD_SW_3,HIGH);

  // delay(5000);
  // digitalWrite(RLY20,LOW);
  // lcd.setCursor(0,0);
  // lcd.print("LV1 =");
  // lcd.setCursor(6,0);
  // var=sensor(0,1);
  // lcd.print(var);

  // parameter.masterLoadVoltage = var ;
  
  // digitalWrite(RLY20,HIGH);
  // delay(5000);
  // lcd.clear();
  // digitalWrite(RLY21,LOW);
  // lcd.print("LV2 =");
  // lcd.setCursor(6,0);
  // var=sensor(0,2);
  // lcd.print(var);

  // parameter.slave1LoadVoltage = var ;

  // digitalWrite(RLY21,HIGH);
  // delay(5000);
  // lcd.clear();
  // digitalWrite(RLY22,LOW);
  // lcd.print("LV3 =");
  // lcd.setCursor(6,0);
  // var=sensor(0,3);
  // lcd.print(var);

  // parameter.slave2LoadVoltage = var ;

  // digitalWrite(RLY22,HIGH);
  // delay(5000);
  // lcd.clear();

  // lcd.setCursor(0,0);
  // digitalWrite(RLY20,LOW);
  // lcd.print("LC1 =");
  // lcd.setCursor(6,0);
  // var=sensor(1,1);
  // lcd.print(var);

  // parameter.masterShortCircuitCurrent = var ;

  // digitalWrite(RLY20,HIGH);
  // delay(5000);
  // lcd.clear();
  // digitalWrite(RLY21,LOW);
  // lcd.print("LC2 =");
  // lcd.setCursor(6,0);
  // var=sensor(1,2);
  // lcd.print(var);

  // parameter.slave1ShortCircuitCurrent = var ;
  
  // digitalWrite(RLY21,HIGH);
  // delay(5000);
  // lcd.clear();
  // digitalWrite(RLY22,LOW);
  // lcd.print("LC3 =");
  // lcd.setCursor(6,0);
  // var=sensor(1,3);
  // lcd.print(var);

  // parameter.slave2LoadCurrent = var ;
  
  // digitalWrite(RLY22,HIGH);
  // delay(5000);
  // lcd.clear();

  // digitalWrite(LOAD_SW_1,LOW);
  // digitalWrite(LOAD_SW_2,LOW);
  // digitalWrite(LOAD_SW_3,LOW);

  lcd.print("temp =");
  lcd.setCursor(7,0);
  var = dht22.getTemperature();
  lcd.print(var);

  parameter.temperature = var;
  
  lcd.setCursor(0,1);
  lcd.print("humi =");
  lcd.setCursor(7,1);
  var1 = dht22.getHumidity();
  lcd.print(var1);

  parameter.humidity = var1 ;

  delay(5000);

  lcd.clear();
  lcd.print("dust =");
  lcd.setCursor(7,0);
  var = dustSensor.getDustDensity();
  lcd.print(var);
  
  lcd.setCursor(0,1);
  lcd.print("avg =");
  lcd.setCursor(7,1);
  var1=dustSensor.getRunningAverage();
  lcd.print(var1);

  parameter.dustSensor = var1 ;

  delay(5000);

  lcd.clear();
  // digitalWrite(RLY20,LOW);
  // lcd.print("power =");
  // lcd.setCursor(8,0);

  // mppt_res ret=mppt(LOAD_SW_1,ACCS,VCS);
  
  // var=ret.pow;
  // var1=ret.imp;

  // parameter.masterMaxPower = var;
  
  // parameter.masterMaxImpedence = var1 ;
  
  // lcd.print(var);
  // lcd.setCursor(0,1);
  // lcd.print("imped =");
  // lcd.setCursor(7,1);
  // lcd.print(var1);
  // delay(5000);
  // digitalWrite(RLY20,HIGH);
  // delay(5000);

  // lcd.clear();
  // digitalWrite(RLY21,LOW);
  // lcd.print("power1 =");
  // lcd.setCursor(9,0);
  
  // ret=mppt(LOAD_SW_2,ACCS1,VCS1);
  
  // var=ret.pow;
  // var1=ret.imp;
  
  // parameter.slave1MaxPower = var ;

  // parameter.slave1MaxImpedence = var1;

  // lcd.print(var);
  // lcd.setCursor(0,1);
  // lcd.print("imped1 =");
  // lcd.setCursor(8,1);
  // lcd.print(var1);
  // delay(5000);
  // digitalWrite(RLY21,HIGH);
  // delay(5000);

  // lcd.clear();
  // digitalWrite(RLY22,LOW);
  // lcd.print("power2 =");
  // lcd.setCursor(9,0);
  
  // ret=mppt(LOAD_SW_3,ACCS2,VCS2);
  // var=ret.pow;
  // var1=ret.imp;
  
  // parameter.slave2MaxPower = var ;
  
  // parameter.slave2MaxImpedence = var1;
  
  // lcd.print(var);
  // lcd.setCursor(0,1);
  // lcd.print("imped2 =");
  // lcd.setCursor(8,1);
  // lcd.print(var1);
  // delay(5000);
  // digitalWrite(RLY22,HIGH);
  // delay(5000);
 
  parameter.day = now.day();
  parameter.month = now.month();
  parameter.year = now.year();
  parameter.hour = now.hour();
  // parameter.hour = parameter.hour + 5; // GMT Time + 5 = Local Pakistan Time
  parameter.mins = now.minute();
  parameter.sec = now.second();
  PCSerial.printf("Current Time: %02d:%02d:%02d\n",parameter.hour,parameter.mins,parameter.sec);
  PCSerial.printf("Current Date: %02d:%02d:%02d\n",parameter.day,parameter.month,parameter.year);

  recordNum += 1;

  PCSerial.printf("============= Write EEPROM ==========\n");
  EEPROMWrite(recordNum);
  
  //delay(10000);

  //PCSerial.printf("============= Read EEPROM ==========\n");
  //EEPROMRead();  

  //while(1);
  
  // adjustable delayh
  delay(10000 *6 *60 *1); //* 60 * 60 * 4);
  //delay(1000);
}


float vol_sensor(int adc_pin)
{
          adc_value = analogRead(adc_pin);
          // PCSerial.println(adc_value);
          V_ADC = (adc_value*ref_voltage) / 4096.0;
          // Serial.println(V_ADC*10);
          return V_ADC*10; 
}


/////////mppt
//  //clock()

mppt_res mppt(int ls,int cp,int  vp)
{
float power=0.0;
float temp_p=0.0;
mppt_res results;
float data_v [10]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float data_c [10]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float power1 [10]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
int i=0;
float adc_1,adc_2;
  for (int x =0; x<10; x++)
{
  digitalWrite(ls, HIGH);
    for (int n=0; n<10; n++)
  {
    adc_1+=(float)analogRead(cp);
    delayMicroseconds(10);
  }
  data_c[i] = ((((float)(adc_1/100) - 3038.0) *(11.9)))/1000;
  // data_c[i]=adc_1/100;
      for (int n=0; n<10; n++)
  {
    adc_2+=(float)analogRead(vp);
    delayMicroseconds(10);
  }
  data_v[i] = ((adc_2/10)*ref_voltage) / 4096.0;
  delay(10-x);
  digitalWrite(ls, LOW);
  delay(x);
  power1[i]=data_c[i]*data_v[i];
  i++;
}

i=0;
// power = max_val(data_c)*max_val(data_v);
// return max_pow(power1);
results.pow=max_pow(power1);
results.imp= max_imp(power1,data_c,data_v);
return results;
// return data_1[0];
// return max_val(data_c);
// return ((((float)(adc_1) - 3069.0) *(11.9)))/1000;
}
/////////mppt
float max_pow(float arr[])
{
  float ret;
  for (int i=0; i<10; i++)
  {
    if (arr[i]>arr[i-1])  ret=arr[i];
  }
  return ret; 
} 



float max_imp(float arr_p[],float arr_c[],float arr_v[])
{
  float ret;
  int n;
  for (int i=0; i<10; i++)
  {
    if (arr_p[i]>arr_p[i-1])  n=i;
  }
  ret=arr_v[n]/arr_c[n];
  return ret; 
} 


float cur_sensor(int adc_pin)
{
for(i=0; i<No_Of_Samples; i++)
  {
    adc_value+= analogRead(adc_pin);
    delay(1);
  }

  adc_value = adc_value/No_Of_Samples;
  // Serial.print("The smapled adc value is = ");
  // Serial.println(adc_value);
  // Vo=(2.505 - (adc_value * (3.3/ 4096.0)));
  // if(Vo<0.00) Vo = 0.00;
  // Ao = (Vo/ACS_Sensitivity);
  // Ao = ((float)(adc_value) - 3030.0) /(0.066*1000) ;
    Ao = ((((float)(adc_value) - 3038.0) *(11.9)))/1000 ;
  // Serial.print("The value form the current sensor is = ");
  // Serial.println(Ao);
  return Ao;
  // return adc_value;
}

float sensor(int sensor, int seq)
{ 
    float val=0.0;
    if (sensor  ==  0)
    {
      switch (seq)
      {
        case 1:
         val = vol_sensor(VCS);
         PCSerial.println(val);
          break;

        case 2:
         val = vol_sensor(VCS1);
          break;

        case 3:
          val = vol_sensor(VCS2);
          break;
      }
    }
    else if (sensor  ==  1)
    { 
      switch (seq)
      {
        case 1:
         val = cur_sensor(ACCS);
          break;
          case 2:
         val = cur_sensor(ACCS1);
          break;
          case 3:
         val = cur_sensor(ACCS2);
          break;
      }
    }

    return val;
}

void BlueTothDownloadData()
{  

//   lcd.clear();
  lcd.print("   //BT Mode//  ");
  BTSerial.println("<" +

                   String("Record No ")           + "," +
                   String("Date ")                + "," +
                   String("Time ")                + "," +
                   String("OC Voltage 1 ")        + "," +
                   String("OC Voltage 2 ")        + "," +
                   String("OC Voltage 3 ")        + "," +
                   String("SC Current 1 ")        + "," +
                   String("SC Current 2 ")        + "," +
                   String("SC Current 3 ")        + "," +
                  //  String("Load Voltage 1 ")      + "," +
                  //  String("Load Voltage 2 ")      + "," +
                  //  String("Load Voltage 3 ")      + "," +
                  //  String("Load Current 1 ")      + "," +
                  //  String("Load Current 2 ")      + "," +
                  //  String("Load Current 3 ")      + "," +
                   String("Temp ")                + "," +
                   String("Humidity ")            + "," +
                   String("Dust ")                + "," +
                   String("Dust Avg ")            + "," +
                  //  String("Max Power 1 ")         + "," +
                  //  String("Max Impedence 1 ")     + "," +
                  //  String("Max Power 2 ")         + "," +
                  //  String("Max Impedence 2 ")     + "," +
                  //  String("Max Power 3 ")         + "," +
                  //  String("Max Impedence 3 ")     + "," +

                   ">");

  EEPROMRead();

}



// #========== CODE FOR DATA WRITE IN EEPROM ============#
void EEPROMWrite(int recNum)
{
  String   rcNum;
  uint32_t addr;

  addr = (recNum-1)*BYTES_IN_EACH_RECORD;
  PCSerial.printf("Addr: %d\n",addr);

  if(recNum < 10)
  {
    rcNum = "00" + String(recNum);
  }

  if( (recNum > 9) && (recNum < 100) )
  {
    rcNum = "0" + String(recNum);
  }

  if( (recNum > 99) && (recNum < 1000) )
  {
    rcNum = String(recNum);
  }

  String str = String("<") +
               String(rcNum) + String(",") +
               String(parameter.day) + String("/") + String(parameter.month) + String("/") + String(parameter.year) + String(",") +
               String(parameter.hour) + String(":") + String(parameter.mins) + String(":") + String(parameter.sec) +  String(",") +
               dtostrf(parameter.masterOpenCircuitVoltage, 5, 2,data) + String(",") +
               dtostrf(parameter.slave1OpenCircuitVoltage, 5, 2,data) + String(",") +
               dtostrf(parameter.slave2OpenCircuitVoltage, 5, 2,data) + String(",") +
               dtostrf(parameter.masterShortCircuitCurrent, 5, 2,data) + String(",") +
               dtostrf(parameter.slave1ShortCircuitCurrent, 5, 2,data) + String(",") +
               dtostrf(parameter.slave2ShortCircuitCurrent, 5, 2,data) + String(",") +
              //  dtostrf(parameter.masterLoadVoltage, 5, 2,data) + String(",") +
              //  dtostrf(parameter.slave1LoadVoltage, 5, 2,data) + String(",") +
              //  dtostrf(parameter.slave2LoadVoltage, 5, 2,data) + String(",") +
              //  dtostrf(parameter.masterLoadCurrent, 5, 2,data) + String(",") +
              //  dtostrf(parameter.slave1LoadCurrent, 5, 2,data) + String(",") +
              //  dtostrf(parameter.slave2LoadCurrent, 5, 2,data) + String(",") +
               dtostrf(parameter.temperature, 5, 2,data) + String(",") +
               dtostrf(parameter.humidity, 5, 2,data) + String(",") +
               dtostrf(parameter.dustSensor, 5, 2,data) + String(",") +
              //  dtostrf(parameter.masterMaxPower, 5, 2,data) + String(",") +
              //  dtostrf(parameter.slave1MaxPower, 5, 2,data) + String(",") +
              //  dtostrf(parameter.slave2MaxPower, 5, 2,data) + String(",") +
              //  dtostrf(parameter.masterMaxImpedence, 5, 2,data) + String(",") +
              //  dtostrf(parameter.slave1MaxImpedence, 5, 2,data) + String(",") +
              //  dtostrf(parameter.slave2MaxImpedence, 5, 2,data) + String(",") +
               String(parameter.padBytes) +
               String(">");

  PCSerial.println(str);

  for(long i=0; i<BYTES_IN_EACH_RECORD; i++)
  {
    EEPROM512.write(i+addr, str[i]);
    PCSerial.print(str[i]);
    delay(2);
  }

  if(recNum < MAX_RECORDS)
  {
    RecordsCounterLo = (recNum & 0x000000FF);
    RecordsCounterHi = (recNum & 0x0000FF00) >> 8;

    if(RecordsCounterLo >= 0xFF)
    {
      RecordsCounterLo = 0;
      RecordsCounterHi++;
    }
  }

  EEPROM512.write(RecordsCounterAddressLo, RecordsCounterLo);
  EEPROM512.write(RecordsCounterAddressHi, RecordsCounterHi);

  PCSerial.printf("\nAddressLo: %d   RecordCounterLo: %d\n",RecordsCounterAddressLo,RecordsCounterLo);
  PCSerial.printf("\nAddressHi: %d   RecordCounterHi: %d\n",RecordsCounterAddressHi,RecordsCounterHi);

  str = " ";
}



uint16_t ReadRecordNumber(uint16_t loByte, uint16_t hiByte)
{
  uint16_t recNuml;
  uint8_t  dataReadByte[2];

  dataReadByte[0] = EEPROM512.read(RecordsCounterAddressLo);
  dataReadByte[1] = EEPROM512.read(RecordsCounterAddressHi);

  recNum = dataReadByte[1]*256 + dataReadByte[0];

  return (recNum);
}


// #========== CODE FOR DATA READ FROM EEPROM ============#

void EEPROMRead(void)
{
  LastRecNum = ReadRecordNumber(RecordsCounterAddressLo, RecordsCounterAddressHi);

  delay(1000);

  for(j=0; j<LastRecNum; j++)
  {
    for(i=0; i<BYTES_IN_EACH_RECORD; i++)
    {
      uint8_t data = EEPROM512.read(i+j*BYTES_IN_EACH_RECORD);
      char ch =  char(data);
      PCSerial.print(ch);
      BTSerial.print(ch);
      delay(2);
    }
    PCSerial.println(" ");
    BTSerial.println(" ");
    delay(2);
  }
  PCSerial.println("$");
  BTSerial.println("$");
}
