//Include required libraries
#include "WiFi.h"
#include <HTTPClient.h>
#include "time.h"
#include "ACS712.h"                //CALLING LIBRARY FOR CURRENT SENSOR
#include <DHT.h>


#define ACS            35           //CURRENT SENSOR OUTPUT PIN CONNECTED TO GPIO 35
#define LED_BUILTIN    2            //CALLING BUILTIN LED OF CONTROLLER FOR PERFORMING ON/OFF OPERATIONS
#define ANALOG_IN_PIN  26           //INITALLIZING THE THE OUTPUT PIN FOR THE VOLTAGE SENSOR
#define NUM_OF_SAMPLES 100          //TAKING VARIABLE = 100 FOR TAKING AVG ADC VALUE
#define DHTPIN         27           // Define the type of sensor and the pin it's connected to.
#define DHTTYPE       DHT22



// WiFi credentials
const char* ssid     = "HUAWEI-ev4F";         // change SSID  Linurotech,HUAWEI-ev4F
const char* password =    "E7m7BwTW";        // change password JUTT960954,E7m7BwTW


// Google script ID and required credentials
String GOOGLE_SCRIPT_ID = "AKfycbxNT_EvgbMA7f_YYaiGvtUG1ST-2M2uGpMl592nPfY2R3bM8QPFPKLuYERTu4f9c1gg";    // change Gscript ID


int     i=0;                         //SET I = 0 FOR LOOP BELOW
int     adc_value = 0.00;               //INITIALIZE THE PARAMETER FOR SAVING ADC_VALUE BELOW
// float   Vo = 0.00;                //INITIAL VALUE FOR Vo as a float value
// float   Ao = 0.00;                //INITIAL VALUE FOR Ao

float ACS_SENSITIVITY = 0.066;         //BASICALLY IT'S THE CURRENT SENSOR SENSITIVTY FOR 30AMPS SENSOR (ITS CONSTANT)
float B1_Volts        =  0.00;         //INITIAL VALUE FOR b1_voltage as a float value
float B1_Current      =  0.00;         //INITIAL VALUE FOR Current
float Temperature     =  0.00;
float Humidity        =  5.00;
float ref_voltage     =   3.3;
float adc_voltage     =  0.00;
float in_voltage      =  0.00;
float Power           =  0.00;
float volts           =  0.00;
float amps            =  0.00;


DHT dht(DHTPIN, DHTTYPE);              // Initialize DHT sensor.

void setup() 

{
  delay(1000);
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_BUILTIN,OUTPUT);

  // connect to WiFi
  Serial.println();
  Serial.print("Connecting to wifi: ");
  Serial.println(ssid);
  Serial.flush();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

     if (WiFi.status() == WL_CONNECTED) 
   
   {
    
    for (i=0; i<20; i++)    
    {
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
    }

   }
  Serial.println("Your desired wifi is connected");
  // Init and get the time
  // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.begin(9600);
  dht.begin();

  analogReadResolution(12);

}





void loop() 
    
  {
   
    for (i=0; i<2; i++)    
    {
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
    }


    String asString(voltage_sensor());
    String asString1(current_sensor());
    String asString2(Temperature_sensor());
    String asString3(Humidity_in_air());
    String asString4(Power_max());
    asString.replace(" ", "-");
    Serial.print("voltage & current & Temperature & Humidity & Power:");
    Serial.print(asString); Serial.print(","); +  Serial.print(asString1); Serial.print(","); Serial.print(asString2);  Serial.print(","); Serial.print(asString3); Serial.print(","); Serial.print(asString4); Serial.println();
    String urlFinal = "https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?"+ "B1_Volts=" + asString + "&B1_Current=" + asString1 + "&Temperature=" + asString2 + "&Humidity=" + asString3 + "&Power=" + asString4;
    Serial.print("POST data to spreadsheet:");
    Serial.println(urlFinal);
    HTTPClient http;
    http.begin(urlFinal.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET(); 
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);
    //---------------------------------------------------------------------
    //getting response from google sheet
    String payload;

    if (httpCode > 0) 
    
    {
        payload = http.getString();
        Serial.println("Payload: "+payload);    
    }
    //---------------------------------------------------------------------
    http.end();
  
  delay(5000);
  } 


  // -----------------------------CURRENT SENSOR FUNCITON---------------------------------//

  
float current_sensor(void)
{
  for(i=0; i<NUM_OF_SAMPLES; i++)
  {
    adc_value += analogRead(ACS);
    delay(2);
  }
  adc_value = adc_value/NUM_OF_SAMPLES;
  // Serial.println(adc_value);

  B1_Volts = ((adc_value* (5 / 4096)) - 2.25);
  Serial.println(B1_Volts);

  B1_Current = ((B1_Volts / ACS_SENSITIVITY));

  if(B1_Current < 0.00) B1_Current = 0.00;
  B1_Current +=0.10;


  // Serial.print(adc_value);  Serial.print(",");  Serial.print(Vo); Serial.print(",");  Serial.println(Ao); Serial.println("Amps");

  return B1_Current;
}

//--------------X--------------------X--------------------X---------------X--------------//


//--------------------------FUNCTION FOR VOLTAGE SENSOR----------------------------------//

float voltage_sensor(void)

{ 
  
   // Read the Analog Input
   adc_value = analogRead(ANALOG_IN_PIN);
   Serial.print("The adc value is = ");
   Serial.println(adc_value);
   
   // Determine voltage at ADC input
   adc_voltage  = (adc_value * ref_voltage) / 4096.0; 

   Serial.print("The adc_voltage is = ");
   Serial.println(adc_voltage);
   // Calculate voltage at divider input
   B1_Volts = adc_voltage; 
   
   // Print results to Serial Monitor to 2 decimal places
  Serial.print("");
  Serial.print("Input Voltage = ");
  Serial.println(in_voltage, 2);
  Serial.print("The voltage is = ");
  Serial.println(B1_Volts);
  return B1_Volts;
}

//-----------------X------------------X-----------------X-----------------------//



float Temperature_sensor(void)

{

  // Temperature = 0.00;
  Temperature = dht.readTemperature();

  return Temperature;

}


float Humidity_in_air(void)

{
  Humidity = dht.readHumidity();
  // Humidity = random(5);

  return Humidity;
}


float Power_max()

{

  volts = voltage_sensor();
  amps  = current_sensor();

  Power = (volts * amps);
  if (Power == 0) Power = 1.00;

  return Power;

}
