#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "ACS712.h"
#include <Adafruit_INA219.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include "FaBoPWM_PCA9685.h"
#include <Filters.h>
#include <ArduinoOTA.h>

#include "_authentification.h"  /* credentials for WIFI and mqtt. Located in libraries folder */


/*****************************************************************************************/
/*                                    GENERAL DEFINE                                     */
/*****************************************************************************************/
#define TRUE  1
#define FALSE 0

#define STATE_OFF           0
#define STATE_ON            1

/*****************************************************************************************/
/*                                    PROJECT DEFINE                                     */
/*****************************************************************************************/
/* FABO_PWM */
#define PWM_ANZAHL_PCA9685_CH   16    /* PWM Channel Number of Module */
#define PWM_ANZAHL_PCA9685      2     /* PWM Modules used */
#define PWM_FREQUENZ            1000  /* caution ISR Frequence depends on this !!! */
#define PWM_ADDRESS_DEFAULT     0x40  /* Adress of PWM Module*/
#define PWM_100_PROZENT_FACTOR  40   /* 100% * 40 [digit] = 4000 [Raw Digit]   ~4095 */

#define LED_0_PROZENT           0     /* [Digit] Lowest Value */
#define LED_100_PROZENT         4095  /* [Digit] Highest Value (4095) */

#define FILTERS_FREQUENCY       0.02  /* Hz */

/* Pin definition for External Interrup1 Pin (comes from Pwm Module) */
#define INTERRUPT_EXT1_PIN      34

#define DHTPIN 27     // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.
// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
DHT dht(DHTPIN, DHTTYPE);

#define ACS712_REFERENCE_ADC_VAL    1792  /* experimental quotet value !!!*/

#define MQTT_PAYLOAD_MAX 250

/* Receive topics */
#define TOPIC_LIVINGROOM_LIGHT01_PERCENT   "livingroom/lights/1/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT02_PERCENT   "livingroom/lights/2/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT03_PERCENT   "livingroom/lights/3/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT04_PERCENT   "livingroom/lights/4/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT05_PERCENT   "livingroom/lights/5/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT06_PERCENT   "livingroom/lights/6/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT07_PERCENT   "livingroom/lights/7/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT08_PERCENT   "livingroom/lights/8/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT09_PERCENT   "livingroom/lights/9/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT10_PERCENT   "livingroom/lights/10/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT11_PERCENT   "livingroom/lights/11/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT12_PERCENT   "livingroom/lights/12/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT13_PERCENT   "livingroom/lights/13/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT14_PERCENT   "livingroom/lights/14/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT15_PERCENT   "livingroom/lights/15/set/brightness"
#define TOPIC_LIVINGROOM_LIGHT16_PERCENT   "livingroom/lights/16/set/brightness"

#define TOPIC_LIVINGROOM_GET_INFO_LIGHT1   "livingroom/lights/1/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT2   "livingroom/lights/2/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT3   "livingroom/lights/3/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT4   "livingroom/lights/4/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT5   "livingroom/lights/5/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT6   "livingroom/lights/6/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT7   "livingroom/lights/7/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT8   "livingroom/lights/8/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT9   "livingroom/lights/9/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT10   "livingroom/lights/10/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT11   "livingroom/lights/11/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT12   "livingroom/lights/12/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT13   "livingroom/lights/13/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT14   "livingroom/lights/14/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT15   "livingroom/lights/15/get/on"
#define TOPIC_LIVINGROOM_GET_INFO_LIGHT16   "livingroom/lights/16/get/on"

/* Send topics */
#define TOPIC_LIVINGROOM_WEATHER          "livingroom/weather"
#define TOPIC_LIVINGROOM_VOLTAGE          "livingroom/ina219/voltage"
#define TOPIC_LIVINGROOM_CURRENT          "livingroom/ina219/current"
#define TOPIC_LIVINGROOM_POWER            "livingroom/ina219/power"
#define TOPIC_LIVINGROOM_LED_CURRENT      "livingroom/acs712/current"
#define TOPIC_LIVINGROOM_LED_POWER        "livingroom/acs712/power"
#define TOPIC_LIVINGROOM_WDT_COUNTER      "livingroom/wdt/counter"

/*****************************************************************************************/
/*                                     TYPEDEF ENUM                                      */
/*****************************************************************************************/
typedef enum
{
  LED_0 = 0,
  LED_1,
  LED_2,
  LED_3,
  LED_4,
  LED_5,
  LED_6,
  LED_7,
  LED_8,
  LED_9,
  LED_10,
  LED_11,
  LED_12,
  LED_13,
  LED_14,
  LED_15,
  LED_16,
  LED_17,
  LED_MAX
}LED_INDEX;

/*****************************************************************************************/
/*                                   TYPEDEF STRUCT                                      */
/*****************************************************************************************/
typedef struct T_DHT_TAG
{
  float humidity;
  float temperature;
  float heat_index;
}T_DHT;

typedef struct LED_STRUCT_TAG
{
  uint16_t Led_AktValue;
  uint16_t Led_LastValue;
}LED_STRUCT;

/*****************************************************************************************/
/*                                         VARIABLES                                     */
/*****************************************************************************************/
/* create an instance of WiFiClientSecure */
WiFiClient espClient;
PubSubClient client(espClient);


int mqttRetryAttempt = 0;
int wifiRetryAttempt = 0;
boolean resetCondition = false;
                 


long lastMsg = 0;
char msg[20];
char touchmsg[20];
int counter = 0;

uint16_t WatchdogAliveCounter = 0;

/* DHT */
static T_DHT Dht;
/* PWM */
static FaBoPWM faboPWM_0;                              /* I2C driver class */
static FaBoPWM faboPWM_1;                              /* I2C driver class */
static LED_STRUCT Led_Struct[LED_MAX]; 
static uint16_t Led_AllOff;
/*                                     0  5 10 15 20  25  30  35  40  45  50  55  60  65  70  75  80  85   90   95      100% */
uint16_t Led_5Percent2Raw_array[21] = {0,20,40,60,80,120,160,200,250,300,350,400,450,500,550,600,650,710,830,980,LED_100_PROZENT};

/* ISR */
static uint32_t Isr_Ext1_counter = 0;
static uint32_t Isr_200ms_counter = 0;
static uint32_t Isr_500ms_counter = 0;
/* INA219 current measurement */
Adafruit_INA219 ina219(0x44);
/* ACS712 current measurement */
// Arduino UNO has 5.0 volt with a max ADC value of 1023 steps
// ACS712 5A  uses 185 mV per A
// ACS712 20A uses 100 mV per A
// ACS712 30A uses  66 mV per A
ACS712  ACS(A0, 5.0, 4095, 100);

static float Acs712_current_mA;
static float Acs712_current_mA_filtered;
static int Acs712_Offset;
static float Acs712_power_mW;

static uint16_t Led_BrightnessLevel;

  /* INA219 current meassurement */
static float Ina219_shuntvoltage = 0;
static float Ina219_busvoltage = 0;
static float Ina219_current_mA = 0;
static float Ina219_current_mA_filtered = 0;
static float Ina219_loadvoltage = 0;
static float Ina219_power_mW = 0;


FilterTwoPole filterTwoLowpass_ina219_current;                          // create a two pole Lowpass filter
FilterTwoPole filterTwoLowpass_acs712_current;                          // create a two pole Lowpass filter


#define DEBUG_MQTT_RECEIVER   Serial.print("Message received: ");  \
                              Serial.print(topic); \
                              Serial.print("\t"); \
                              Serial.print("payload: "); \
                              Serial.println(PayloadString);

#define DEBUG_RELAIS_HANDLING   Serial.print("Relais Number: "); \
                                Serial.print(Arg_Relais_Number); \
                                Serial.print("\t"); \
                                Serial.print("Pin: "); \
                                Serial.print(Loc_PinNumber[Arg_Relais_Number]); \
                                Serial.print("\t"); \
                                Serial.print("State: "); \
                                Serial.print(Arg_Relais_State); \
                                Serial.print("\t"); \
                                Serial.print("ReturnVal: "); \
                                Serial.print(RetVal); \
                                Serial.println(""); 
                                                         
#define DEBUG_TEMPERATURE   Serial.print(F("Humidity: ")); \
                            Serial.print(Dht.humidity); \
                            Serial.print(F("%  Temperature: ")); \
                            Serial.print(Dht.temperature); \
                            Serial.print(F("°C ")); \
                            Serial.print(F("   Heat index: ")); \
                            Serial.print(Dht.heat_index); \
                            Serial.print(F("°C ")); \
                            Serial.print(F("   Status: ")); \
                            Serial.println(RetVal); 


static void ISR_Ext1(void);
static void receivedCallback(char* topic, byte* payload, unsigned int length);
static void mqttconnect(void);

/**************************************************************************************************
Function: ArduinoOta_Init()
Argument: void
return: void
**************************************************************************************************/
void ArduinoOta_Init(void)
{
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("esp32-home-automation-mqtt");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
} 

/**************************************************************************************************
Function: Dht_Init()
Argument: void
return: void
**************************************************************************************************/
void Dht_Init(void)
{
  Serial.println(F("DHT22 Init..."));
  dht.begin();
}

/**************************************************************************************************
Function: Led_Init()
Argument: Arg_LedIndex [0...1024][1] ; Index of the LED
return: void
**************************************************************************************************/
void Led_Init()
{
  uint16_t Loc_Address = PWM_ADDRESS_DEFAULT;
  uint16_t i;

  /* memset of struct */
  memset(&Led_Struct[0],0x00, LED_MAX * sizeof(&Led_Struct[0]));

  Led_BrightnessLevel = LED_100_PROZENT;
  
  for (i = 0;i < LED_MAX; i++)
  {
    Led_Struct[i].Led_LastValue = LED_0_PROZENT;

    Led_SetValue((LED_INDEX)i, LED_0_PROZENT);
    /* update driver */
    Led_UpdateDriver();
    
    delay(10);
  }
}
/**************************************************************************************************
Function: Pwm_Extern_Init()
Argument: void
return: void
**************************************************************************************************/
void Pwm_Extern_Init(void)
{
  uint16_t Loc_Begin0 = 0;
  uint16_t Loc_Begin1 = 0;

  Loc_Begin0 |= faboPWM_0.begin(PWM_ADDRESS_DEFAULT);
  Loc_Begin1 |= faboPWM_1.begin(PWM_ADDRESS_DEFAULT + 1);

  /* join I2C bus (I2Cdev library doesn't do this automatically */
  if(Loc_Begin0 != 0)
  {
    Serial.println("Find PCA9685 PCA9685 Device 0");
  }
  if(Loc_Begin1 != 0)
  {
    Serial.println("Find PCA9685 PCA9685 Device 1");
  }
  faboPWM_0.init(PWM_ADDRESS_DEFAULT, 0);
  faboPWM_0.set_hz(PWM_ADDRESS_DEFAULT, PWM_FREQUENZ);
  
  faboPWM_1.init(PWM_ADDRESS_DEFAULT + 1, 0);
  faboPWM_1.set_hz(PWM_ADDRESS_DEFAULT + 1, PWM_FREQUENZ);
}

/**************************************************************************************************
Function: Led_UpdateDriver()
Argument: void
Argument: void 
return: void
**************************************************************************************************/
void Led_UpdateDriver(void)
{
  uint16_t Loc_Address = PWM_ADDRESS_DEFAULT;
  uint16_t i;
  
  for (i = 0;i < LED_MAX; i++)
  {
    if(Led_Struct[i].Led_AktValue != Led_Struct[i].Led_LastValue)
    {
      if(i <= LED_15)
      {
        /* update driver PWM Driver 1 */
        faboPWM_0.set_channel_value(Loc_Address,i, Led_Struct[i].Led_AktValue);
        //Led_Struct[i].Led_AktValue = faboPWM.get_channel_value(Loc_Address, i);
      }
      else
      {
        /* update driver PWM Driver 2 */
        faboPWM_1.set_channel_value(Loc_Address + 1,i-LED_16, Led_Struct[i].Led_AktValue);
        //Led_Struct[i].Led_AktValue = faboPWM.get_channel_value(Loc_Address, i);
      }
    }
    /* store akt value */
    Led_Struct[i].Led_LastValue = Led_Struct[i].Led_AktValue;
  }
}

/**************************************************************************************************
Function: Led_SetValue()
Argument: Arg_LedIndex [0...15][1] ; Index of the LED
Argument: Arg_Value [0...4095][1] ; value to set 
return: void
**************************************************************************************************/
void Led_SetValue(LED_INDEX Arg_LedIndex, uint16_t Arg_Value)
{
  if(Arg_LedIndex < LED_MAX)
  {
    Led_Struct[Arg_LedIndex].Led_AktValue = Arg_Value;
  }
}


/**************************************************************************************************
Function: Led_GetValue()
Argument: Arg_LedIndex [0...15][1] ; Index of the LED
return: uint16_t ; value
**************************************************************************************************/
uint16_t  Led_GetValue(LED_INDEX Arg_LedIndex)
{
  uint16_t Ret_Value;

  return (Led_Struct[Arg_LedIndex].Led_AktValue);
}

/**************************************************************************************************
Function: Led_GetValue()
Argument: Arg_LedIndex [0...15][1] ; Index of the LED
return: uint16_t ; value  TRUE = All off else FALSE
**************************************************************************************************/
uint16_t Led_GetAllOff(void)
{
  uint16_t Ret_Value = TRUE;
  uint16_t Loc_Val = 0;
  uint8_t i;

  for(i=0;i<LED_MAX;i++)
  {
    if(Led_Struct[i].Led_AktValue > 0)
    {
      Loc_Val |= TRUE;
    }
  }

  if(Loc_Val != FALSE)
  {
    Ret_Value = FALSE;
    Led_AllOff = FALSE;
  }
  else
  {
    Led_AllOff = TRUE;
    Ret_Value = TRUE;
  }

  return (Ret_Value);
}


/**************************************************************************************************
Function: Led_FadeIn()
Argument: Arg_LedIndex [0...1024][1] ; Index of the LED
Argument: Arg_Step [0...4095][1] ; Step with to fade 
return: void
**************************************************************************************************/
uint16_t Led_FadeIn(LED_INDEX Arg_LedIndex, uint16_t Arg_Step)
{
  if(Arg_LedIndex < LED_MAX)
  {
    /* Fade In LED */
    if((Led_Struct[Arg_LedIndex].Led_AktValue + Arg_Step) < Led_Get_BrightnessLevel())
    {
      Led_Struct[Arg_LedIndex].Led_AktValue += Arg_Step;
    }
    else
    {
      Led_Struct[Arg_LedIndex].Led_AktValue = Led_Get_BrightnessLevel();
    }
  }
 return (Led_Struct[Arg_LedIndex].Led_AktValue);
}

/**************************************************************************************************
Function: Led_FadeOut()
Argument: Arg_LedIndex [0...1024][1] ; Index of the LED
Argument: Arg_Step [0...4095][1] ; Step with to fade 
return: void
**************************************************************************************************/
uint16_t Led_FadeOut(LED_INDEX Arg_LedIndex, uint16_t Arg_Step)
{
  if(Arg_LedIndex < LED_MAX)
  {
    if(     ((Led_Struct[Arg_LedIndex].Led_AktValue - Arg_Step) < Led_Struct[Arg_LedIndex].Led_AktValue)
         && (Led_Struct[Arg_LedIndex].Led_AktValue > LED_0_PROZENT))
    {
      /* Fade Out LED */
      Led_Struct[Arg_LedIndex].Led_AktValue -= Arg_Step;
    }
    else
    {
      Led_Struct[Arg_LedIndex].Led_AktValue = LED_0_PROZENT;
    }
  }
  return Led_Struct[Arg_LedIndex].Led_AktValue;
}


/**************************************************************************************************
Function: Led_calc_percent2rawval()
Argument: Arg_LedIndex [0...15][1] ; Index of the LED
return: uint16_t ; value
**************************************************************************************************/
uint16_t  Led_calc_percent2rawval(uint8_t Arg_percent)
{
  uint8_t Loc_Index;
  uint16_t Ret_Value;

 if(Arg_percent == 0)
 {
  Loc_Index = 0;
 }
 else
 {
   Loc_Index = (Arg_percent / 5); 
 } 
 Ret_Value = Led_5Percent2Raw_array[Loc_Index]; 
 
 return (Ret_Value);
}

/**************************************************************************************************
Function: Led_Get_BrightnessLevel()
Argument: -
Argument: - 
return: uint16_t [0...1023][1]; Level of potentiometer for brightness
**************************************************************************************************/
uint16_t Led_Get_BrightnessLevel(void)
{
  /* 10 Bit value */
  return (Led_BrightnessLevel);
}
 
/**************************************************************************************************
Function: Dht_Loop()
Argument: void
return: void
**************************************************************************************************/
uint8_t Dht_Loop(void)
{
  uint8_t RetVal = TRUE;

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  Dht.humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  Dht.temperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(Dht.humidity) || isnan(Dht.temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    RetVal = FALSE;
  }
  // Compute heat index in Celsius (isFahreheit = false)
  Dht.heat_index = dht.computeHeatIndex(Dht.temperature, Dht.humidity, false);

  /* Debug */
  DEBUG_TEMPERATURE

   return (RetVal);
}

/**************************************************************************************************
Function: Ina219_Init()
Argument: void
return: void
**************************************************************************************************/
void Ina219_Init(void)
{
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  if (! ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();

  Serial.println("Measuring voltage and current with INA219 ...");
}

/**************************************************************************************************
Function: Acs712_Init()
Argument: void
return: void
**************************************************************************************************/
void Acs712_Init(void)
{
  /* set estimated value */
  ACS.setMidPoint(ACS712_REFERENCE_ADC_VAL);
}

/**************************************************************************************************
Function: Ina219_Loop()
Argument: void
return: uint_8 RetVal
**************************************************************************************************/
uint8_t Ina219_Loop(void)
{
  uint8_t RetVal = TRUE;
  
  //Ina219_shuntvoltage = ina219.getShuntVoltage_mV();
  Ina219_busvoltage = ina219.getBusVoltage_V();
  Ina219_current_mA = ina219.getCurrent_mA();
  Ina219_power_mW = ina219.getPower_mW();
  //Ina219_loadvoltage = Ina219_busvoltage + (Ina219_shuntvoltage / 1000);

    /* update the two pole Lowpass filter */
  filterTwoLowpass_ina219_current.input(Ina219_current_mA);
  /* read the filtered value */
  Ina219_current_mA_filtered = filterTwoLowpass_ina219_current.output();

  Serial.print("PCB: ");
  Serial.print("voltage: ");
  Serial.print(Ina219_busvoltage); 
  Serial.print(" V\t");
  Serial.print("current: "); 
  Serial.print(Ina219_current_mA_filtered); 
  Serial.print(" mA\t");
  Serial.print("power: "); 
  Serial.print(Ina219_power_mW); 
  Serial.println(" mW");
  
  //Serial.print("Shunt Voltage: "); Serial.print(Ina219_shuntvoltage); Serial.println(" mV");
  //Serial.print("Load Voltage: "); Serial.print(Ina219_loadvoltage); Serial.println(" V");
  
  //Serial.println("");
   
  return (RetVal);
}

/**************************************************************************************************
Function: Acs_Loop()
Argument: void
return: uint_8 RetVal
**************************************************************************************************/
uint8_t Acs712_Loop(void)
{
  uint8_t RetVal = TRUE;
  
  Acs712_current_mA = ACS.mA_DC();
  Acs712_Offset = ACS.getMidPoint();
  
  /*is the calibration allowed? */
  if(Led_GetAllOff() == TRUE)
  {
    /* Set current to zero, looks much better :-) */
    Acs712_current_mA = 0;
  }


  /* update the two pole Lowpass filter */
  filterTwoLowpass_acs712_current.input(Acs712_current_mA);
  /* read the filtered value */
  Acs712_current_mA_filtered = filterTwoLowpass_acs712_current.output();
  /* calculate the filtered used power */
  Acs712_power_mW = Acs712_current_mA_filtered * Ina219_busvoltage;

  Serial.print("LED: ");
  Serial.print("current raw: ");
  Serial.print(Acs712_current_mA);
  Serial.print(" mA\t");
  Serial.print("current filtered: ");
  Serial.print(Acs712_current_mA_filtered);
  Serial.print(" mA\t");
  Serial.print("power: ");
  Serial.print(Acs712_power_mW);
  Serial.println(" mW");
       
  return (RetVal);
}

/**************************************************************************************************
Function: Interrupt_Init()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Interrupt_Init(void)
{
  Serial.println("Interrupt Init ... ");
  /* Set the last channel from PWM modul as Interrupt source and 50% duty cycle */
  faboPWM_0.set_channel_value(PWM_ADDRESS_DEFAULT + 1,15, 2048);
  /* configure Pin as Interrupt Pin */
  pinMode(INTERRUPT_EXT1_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_EXT1_PIN), ISR_Ext1, RISING);
}

/*************************************************************************************************/
/**************************************************************************************************
Function: setup()
return: void
**************************************************************************************************/
/*************************************************************************************************/
void setup()
{
  Serial.begin(115200);

  Serial.println(" ");
  Serial.println("################################");
  Serial.println("# Program Home-Automation v0.3 #");
  Serial.println("################################");
  Serial.println(__FILE__);
  Serial.println(" ");
  Serial.println("Starting ...");
  Serial.println(" ");

  // standard Lowpass, set to the corner frequency
  filterTwoLowpass_ina219_current.setAsFilter( LOWPASS_BESSEL, FILTERS_FREQUENCY);

  Pwm_Extern_Init();
  Led_Init();

  Dht_Init();

  Ina219_Init();

  Acs712_Init();

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print(".");
    wifiRetryAttempt++;
    if (wifiRetryAttempt > 5) 
    {
      Serial.println("Restarting!");
      ESP.restart();
    }
  }

  ArduinoOta_Init();
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("IP address of server: ");
  Serial.println(serverHostname);
  /* set SSL/TLS certificate */
  /* configure the MQTT server with IPaddress and port */
  client.setServer(serverHostname, 1883);
  /* this receivedCallback function will be invoked
    when client received subscribed topic */
  client.setCallback(receivedCallback);

  Interrupt_Init();
    
  Serial.println("Setup finished ... ");
}

/*************************************************************************************************/
/**************************************************************************************************
Function: loop()
return: void
**************************************************************************************************/
/*************************************************************************************************/
void loop() 
{
  ArduinoOTA.handle();
  
  /* if client was disconnected then try to reconnect again */
  if (!client.connected()) {
    mqttconnect();
  }
  /* this function will listen for incomming
  subscribed topic-process-invoke receivedCallback */
  client.loop();
  /* Update PWM LED Driver */
  Led_UpdateDriver();

  /* we increase counter every 5 secs we count until 5 secs reached to avoid blocking program if using delay()*/
  long now = millis();
  
  /* calling every 5 sec. */
  if (now - lastMsg > 5000)
  {
    /* store timer value */
    lastMsg = now;
    
    /* update humidity and temperature */
    Dht_Loop();
    /* Update current meassurement pcb */
    Ina219_Loop();
    /* Update current meassurement of the LED Spots */
    Acs712_Loop();


    /********************************************************************************************/
    /************************      HANDLING OF Send MQTT TOPICS     *****************************/ 
    /********************************************************************************************/
    char data[MQTT_PAYLOAD_MAX];
    String json; 
       
    char counter[5];
    dtostrf(WatchdogAliveCounter++,  5, 0, counter);
    json = "{\"counter\":" + String(counter) + ",\"counter2\":" + String(counter) + "}";
    json.toCharArray(data, (json.length() + 1));
    client.publish(TOPIC_LIVINGROOM_WDT_COUNTER, data, false);
     
    char temp[8];
    char humidity[8];
    dtostrf(Dht.heat_index,  6, 2, temp);
    dtostrf(Dht.humidity, 6, 2, humidity);
    json = "{\"temperature\":" + String(temp) + ",\"humidity\":" + String(humidity) + "}";
    json.toCharArray(data, (json.length() + 1));
    client.publish(TOPIC_LIVINGROOM_WEATHER, data, false);

    char voltage[8];
    dtostrf(Ina219_busvoltage,  2, 1, voltage);
    json = "{\"voltage\":" + String(voltage) + "}";
    json.toCharArray(data, (json.length() + 1));
    client.publish(TOPIC_LIVINGROOM_VOLTAGE, data, false); 

    char current[8];
    dtostrf(Ina219_current_mA_filtered,  4, 0, current);
    json = "{\"current\":" + String(current) + "}";
    json.toCharArray(data, (json.length() + 1));
    client.publish(TOPIC_LIVINGROOM_CURRENT, data, false); 

    char power[8];
    dtostrf(Ina219_power_mW,  4, 0, power);
    json = "{\"power\":" + String(power) + "}";
    json.toCharArray(data, (json.length() + 1));
    client.publish(TOPIC_LIVINGROOM_POWER, data, false); 
    
    char current1[8];
    dtostrf(Acs712_current_mA_filtered,  6, 0, current1);
    json = "{\"current\":" + String(current1) + "}";
    json.toCharArray(data, (json.length() + 1));
    client.publish(TOPIC_LIVINGROOM_LED_CURRENT, data, false); 

    char power1[8];
    dtostrf(Acs712_power_mW,  7, 0, power1);
    json = "{\"power\":" + String(power1) + "}";
    json.toCharArray(data, (json.length() + 1));
    client.publish(TOPIC_LIVINGROOM_LED_POWER, data, false);
    
  }

}

/**************************************************************************************************
Function: receivedCallback()
Argument: char* topic ; received topic
          byte* payload ; received payload
          unsigned int length ; received length
return: void
**************************************************************************************************/
void receivedCallback(char* topic, byte* payload, unsigned int length) 
{
  byte i;
  uint8_t Loc_Brightness;
  String val_status;
  uint16_t Loc_Led_Value;
  String json_s;
  char json_data[MQTT_PAYLOAD_MAX];

  
  char PayloadString[length + 1 ];
  /* convert payload in string */
  for(i=0;i<length;i++)
  {
    PayloadString[i] = payload[i];
  }
  PayloadString[length] = '\0';

  /* Debug */
  DEBUG_MQTT_RECEIVER

  StaticJsonBuffer<250> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(PayloadString);

  if (!root.success()) 
  {
    Serial.println("JSon parseObject() failed");
  } 
  else 
  {
    Serial.println("JSON message parsed succesfully");
  }

  /********************************************************************************************/
  /********************      HANDLING OF Received MQTT TOPICS WITH JASON     ******************/ 
  /********************************************************************************************/
  
  /*+++++++++++++++++++++++++++++ Set Light in percent +++++++++++++++++++++++++++++++++++++++*/ 
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT01_PERCENT)==0)
  {
	
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 1 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_0, Led_calc_percent2rawval(Loc_Brightness));
    }
	  /* create status */
    Loc_Led_Value = Led_GetValue(LED_0);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT1, json_data, false); 
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT02_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 2 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_1, Led_calc_percent2rawval(Loc_Brightness));
    }
	  /* create status */
    Loc_Led_Value = Led_GetValue(LED_1);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT2, json_data, false); 
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT03_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 3 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_2, Led_calc_percent2rawval(Loc_Brightness));
    }
	  /* create status */
    Loc_Led_Value = Led_GetValue(LED_2);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT3, json_data, false);
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT04_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 4 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_3, Led_calc_percent2rawval(Loc_Brightness));
    }
	  /* create status */
    Loc_Led_Value = Led_GetValue(LED_3);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT4, json_data, false);
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT05_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 5 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_4, Led_calc_percent2rawval(Loc_Brightness));
    }
	  /* create status */
    Loc_Led_Value = Led_GetValue(LED_4);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT5, json_data, false); 
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT06_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 6 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_5, Led_calc_percent2rawval(Loc_Brightness));
    }
	  /* create status */
    Loc_Led_Value = Led_GetValue(LED_5);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT6, json_data, false); 
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT07_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 7 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_6, Led_calc_percent2rawval(Loc_Brightness));
    }
	  /* create status */
    Loc_Led_Value = Led_GetValue(LED_6);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT7, json_data, false); 
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT08_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 8 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_7, Led_calc_percent2rawval(Loc_Brightness));
    }
    /* create status */
    Loc_Led_Value = Led_GetValue(LED_7);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT8, json_data, false); 
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT09_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 9 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_8, Led_calc_percent2rawval(Loc_Brightness));
    }
	  /* create status */
    Loc_Led_Value = Led_GetValue(LED_8);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT9, json_data, false);
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT10_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 10 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_9, Led_calc_percent2rawval(Loc_Brightness));
    }
	  /* create status */
    Loc_Led_Value = Led_GetValue(LED_9);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT10, json_data, false); 
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT11_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 11 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_10, Led_calc_percent2rawval(Loc_Brightness));
    }
  	/* create status */
    Loc_Led_Value = Led_GetValue(LED_10);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT11, json_data, false);
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT12_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 12 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_11, Led_calc_percent2rawval(Loc_Brightness));
    }
  	/* create status */
    Loc_Led_Value = Led_GetValue(LED_11);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT12, json_data, false); 
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT13_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 13 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_12, Led_calc_percent2rawval(Loc_Brightness));
    }
  	/* create status */
    Loc_Led_Value = Led_GetValue(LED_12);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT13, json_data, false);
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT14_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 14 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_13, Led_calc_percent2rawval(Loc_Brightness));
    }
  	/* create status */
    Loc_Led_Value = Led_GetValue(LED_13);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT14, json_data, false);
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT15_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 15 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_14, Led_calc_percent2rawval(Loc_Brightness));
    }
  	/* create status */
    Loc_Led_Value = Led_GetValue(LED_14);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT15, json_data, false); 
  }
  if(strcmp(topic, TOPIC_LIVINGROOM_LIGHT16_PERCENT)==0)
  {
    if(root.containsKey("brightness")) 
    {
      Loc_Brightness = root["brightness"];
      Serial.print("Light 16 brightness Set:");
      Serial.println(Loc_Brightness, DEC);
      Led_SetValue(LED_15, Led_calc_percent2rawval(Loc_Brightness));
    }
  	/* create status */
    Loc_Led_Value = Led_GetValue(LED_15);
    if(Loc_Led_Value == 0){
      val_status = "false";}
    else{
      val_status = "true";}
    json_s = String(val_status);
    json_s.toCharArray(json_data, (json_s.length() + 1));
    client.publish(TOPIC_LIVINGROOM_GET_INFO_LIGHT16, json_data, false);
  }

}


/**************************************************************************************************
Function: mqttconnect()
Argument: void
return: void
**************************************************************************************************/
void mqttconnect(void)
{
  /* Loop until reconnected */
  while (!client.connected()) 
  {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "esp32-home-automation-mqtt";
    /* connect now */
    if (client.connect(clientId.c_str(), serverUsername.c_str(), serverPassword.c_str()))
    {
      Serial.println("connected");
      /* subscribe topic's */
      client.subscribe(TOPIC_LIVINGROOM_LIGHT01_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT02_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT03_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT04_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT05_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT06_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT07_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT08_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT09_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT10_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT11_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT12_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT13_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT14_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT15_PERCENT);
      client.subscribe(TOPIC_LIVINGROOM_LIGHT16_PERCENT);
    } 
    else 
    {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
      mqttRetryAttempt++;
      if (mqttRetryAttempt > 5) 
      {
        Serial.println("Restarting!");
        ESP.restart();
      }
    }
  }
}


/**************************************************************************************************
Function: Isr_10ms_Task0()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Isr_10ms_Task0(void)
{
      

}

/**************************************************************************************************
Function: Isr_200ms_Task0()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Isr_200ms_Task0(void)
{
  
  Isr_200ms_counter++;
  
}

/**************************************************************************************************
Function: Isr_500ms_Task0()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void Isr_500ms_Task0(void)
{
  Isr_500ms_counter++;
 
}

/**************************************************************************************************
Function: ISR_Ext1()
Argument: -
Argument: - 
return: void
**************************************************************************************************/
void ISR_Ext1(void)
{
  Isr_Ext1_counter++;
  if(Isr_Ext1_counter % 10 == 0)
  {
    //Serial.println("10ms ");
    Isr_10ms_Task0(); 
  }
  if(Isr_Ext1_counter % 200 == 0)
  {
    //Serial.println("200ms ");
    Isr_200ms_Task0(); 
  }
  if(Isr_Ext1_counter %500 == 0)
  {
    //Serial.println("500ms ");
    Isr_500ms_Task0();
  }
}
