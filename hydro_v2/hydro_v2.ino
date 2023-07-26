#include <dht.h>
#include <avr/wdt.h>
#include "functions.h"
#include <Wire.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>

char datestring[20];
unsigned long p_time_now, pump_time_now;

int previousDay;
int currentDay;
int start_day;
int percent;
//========================================
//For Serial Send
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing

// variables to hold the parsed data
char modeFromOrangePi[numChars] = { 0 };
char healthFromOrangePi[numChars] = { 0 };
int ppmlowerthresholdFromOrangePi = 0;
int ppmupperthresholdFromOrangePi = 0;
float pHlowerthresholdFromOrangePi = 0.0;
float pHupperthresholdFromOrangePi = 0.0;

bool newData = false;
//=================================


//Define Water Level Sensor (System Checking)
#define WaterLevelSensorPin A0
//**********************************

//Define Water Flow Sensor (System Checking)
//Pin to be defined
#define WaterFlow A0
float TIME = 0;
float FREQUENCY = 0;
float WATER = 0;
float TOTAL = 0;
float LS = 0;
//**********************************

//Define TDS Sensor (Growth factor Sensor)
#define TdsSensorPin A2
#define VREF 5.0   //analog reference voltage(Volt) of the ADC
#define SCOUNT 30  // analog reference voltage(Volt) of the ADC

int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;  // current temperature for compensation
float humidity = 0;
float ppm = 0;
//**********************************

//Define pH Sensor (Growth factor Sensor)
#define pHSensorPin A1
int samples = 10;
float adc_resolution = 1024.0;
float pH = 0.0;
//**********************************

//Define DHT Sensor (temperature and humidity) (Growth factor Sensor)
#define dhtPin 16
dht DHT;
//**********************************

//Define BH1750 Sensor (Light intensity) (Growth factor Sensor)
//BH1750 GY302;
//uint16_t lux;
//float light = 0.0;
//**********************************

//Define DS1302 RTC module (Growth factor Sensor)
ThreeWire myWire(18, 17, 19);  //IO,SCLK,CE
RtcDS1302<ThreeWire> Rtc(myWire);
//**********************************

//Define Actuators

//LED Light
#define lights 10

//Motor Driver
//U28
#define in1_28 34  //P1
#define in2_28 36  //P1
#define in3_28 38  //P2
#define in4_28 40  //P2
#define enA_28 2   //P1
#define enB_28 3   //P2
//U27
#define in1_27 42  //F1 & F2
#define in2_27 44  //F1 & F2
#define in3_27 46  //RS755 Fan Motor
#define in4_27 48  //RS755 Fan Motor
#define enA_27 4   //F1 & F2
#define enB_27 5   //RS755 Fan Motor
//U30
#define in1_30 A4  //PP1
#define in2_30 A5  //PP1
#define in3_30 A6  //PP2
#define in4_30 A7  //PP2
#define enA_30 6   //PP1
#define enB_30 7   //PP2
//U31
#define in1_31 A8   //PP3
#define in2_31 A9   //PP3
#define in3_31 A10  //PP4
#define in4_31 A11  //PP4
#define enA_31 8    //PP3
#define enB_31 9    //PP4
//**********************************
//------------Define Actuators------------
//Solenoid Valves
//U24 Close for misting, Open for Refilling
//U35  Close for Refilling, Open for Misting
#define solenoid24 13
//U36
#define solenoid35 12

//LED Light
#define lights 10

//Motor Driver
//U28
#define in1_28 34  //P1
#define in2_28 36  //P1
#define in3_28 38  //P2
#define in4_28 40  //P2
#define enA_28 2   //P1
#define enB_28 3   //P2
//U27
#define in1_27 42  //F1 & F2
#define in2_27 44  //F1 & F2
#define in3_27 46  //RS755 Fan Motor
#define in4_27 48  //RS755 Fan Motor
#define enA_27 4   //F1 & F2
#define enB_27 5   //RS755 Fan Motor
//U30
#define in1_30 A4  //PP1
#define in2_30 A5  //PP1
#define in3_30 A6  //PP2
#define in4_30 A7  //PP2
#define enA_30 6   //PP1
#define enB_30 7   //PP2
//U31
#define in1_31 A8   //PP3
#define in2_31 A9   //PP3
#define in3_31 A10  //PP4
#define in4_31 A11  //PP4
#define enA_31 8    //PP3
#define enB_31 9    //PP4
//------------END Define Actuators------------
//For Saver Mode
long previousMillis = 0;
//Define pins for Ultrasonic Sensors (System Checking)
//U6 - pH Up
const int trigPin6 = 23;
const int echoPin6 = 25;
//U9 - pH Down
const int trigPin9 = 27;
const int echoPin9 = 29;
//U8 - Nutrient A
const int trigPin8 = 31;
const int echoPin8 = 33;
//U11 - Nutrient B
const int trigPin11 = 39;
const int echoPin11 = 41;
//U12  - Water Reservoir
const int trigPin12 = 43;
const int echoPin12 = 45;
//U10 Nut Sol Reservoir
const int trigPin10 = 47;
const int echoPin10 = 49;

//**********************************

//******* THRESHOLD VALUES **********
//delay takes value in ms ==== 1000ms = 1 sec
//NutSolReservoir Level
int NutSol_lower_threshold = 0;
int NutSol_upper_threshold = 73;
int NutSolLevel = 0;

//TDS Thresholds
int ppm_lower_threshold = 600;
int ppm_upper_threshold = 850;
int NutrientA_delay_6a_first = 100;  //Delay in ms for first time 1.125 mL/L
int NutrientA_delay_6a_other = 10;   //Delay in ms for 0.05 ml/L
int NutrientB_delay_6a_first = 100;  ////Delay in ms for first time 1.125 mL/L
int NutrientB_delay_6a_other = 10;   //delay in ms for 0.05 mL
int five_c_one_delay = 10;           //delay in ms for 0.5 L

int nutsol_delay = 800; // 800ms per 1.5ml - 1 liter of water
int nutsol_delay_titration = 266; // 266ms per 0.5ml
//pH Thresholds
float pH_upper_threshold = 6.2;
float pH_lower_threshold = 5.8;

int pH_down_delay_first = 1000;  //Delay in ms to add X mL of pH down solution
int pH_down_delay_other = 10;    //Delay in ms to add 0.05 mL of pH down solution...1000ms = 1sec

int pH_up_delay_first = 1000;  //Delay in ms to add X mL of pH down solution
int pH_up_delay_other = 10;    //Delay in ms to add 0.05 mL of pH up solution...1000ms = 1sec

int pH_up_delay = 1000;    //Delay in ms to add X mL of pH Up for Tip burn disease
int pH_down_delay = 1000;  //Delay in ms to add X mL of pH Down for Tip burn disease

char mode = 'O';  //O - Optimal Mode(default) S - Saver Mode
char health = 'H';
long interval = 1000 * 60 * 15;  //15 minute interval

//Light Intensity Thresholds
int lux_lower_threshold = 5100;
int lux_upper_threshold = 7600;
int light_pwm = 255;  // 0 - 255 : to increase light intensity

//Fan F1 & F2 Speed Control
int F_Speed = 150;  //0-255

int notify_threshold = 20;             //if any container less than this value, send to orange pi
int Nutrient_A_container_height = 73;  //Max distance / ultrasonic sensor max value
int Nutrient_B_container_height = 73;  //Max distance / ultrasonic sensor max value
int pH_down_container_height = 73;
int pH_up_container_height = 73;
int water_reservoir_container_height = 73;
int solution_lower_threshold = 200; // 200 ml lowest threshold for solutions

int Nut_Sol_Level = 90;  //NutSol will fill upto this %

//humidity
int humidity_lower_threshold = 50;
int humidity_upper_threshold = 70;

//temperature
int temperature_threshold = 26;

//Brown disease add 1L water
long brown_spot_delay_add_one_L_water = 1;
//**********************************
int water_reservoir_Level;
int Nutrient_A_Notify_Level;
int Nutrient_B_Notify_Level;
float pH_up_Notify_Level;
float pH_down_Notify_Level;
float pH_up_Level;

#define countof(a) (sizeof(a) / sizeof(a[0]))

unsigned long printDateTime(const RtcDateTime& dt) {
  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  Serial3.println(datestring);
}

volatile int flow_frequency; // Measures flow sensor pulses
// Calculated litres/hour
 float vol = 0.0,l_minute;
unsigned long currentTime;
unsigned long cloopTime;

void setup() {
  //Start Serial
  Serial.begin(4800);
  Serial3.begin(9600);  //Connection to Orange Pi

  Wire.begin();
  //Initialise BH1750
  //GY302.begin();
  //Setup Ultrasonic sensors
  //U6
  pinMode(trigPin6, OUTPUT);
  pinMode(echoPin6, INPUT);
  //U9
  pinMode(trigPin9, OUTPUT);
  pinMode(echoPin9, INPUT);
  //U8
  pinMode(trigPin8, OUTPUT);
  pinMode(echoPin8, INPUT);
  //U11
  pinMode(trigPin11, OUTPUT);
  pinMode(echoPin11, INPUT);
  //U12
  pinMode(trigPin12, OUTPUT);
  pinMode(echoPin12, INPUT);
  //U10
  pinMode(trigPin10, OUTPUT);
  pinMode(echoPin10, INPUT);

  //TDS
  pinMode(TdsSensorPin, INPUT);

  //Actuators
  //Light
  pinMode(lights, OUTPUT);
  //Solenoids
  pinMode(solenoid24, OUTPUT);
  pinMode(solenoid35, OUTPUT);
  //pinMode(solenoid36, OUTPUT);

  //Motor Driver
  //U28
  pinMode(in1_28, OUTPUT);
  pinMode(in2_28, OUTPUT);
  pinMode(in3_28, OUTPUT);
  pinMode(in4_28, OUTPUT);
  pinMode(enA_28, OUTPUT);
  pinMode(enB_28, OUTPUT);

  //U27
  pinMode(in1_27, OUTPUT);
  pinMode(in2_27, OUTPUT);
  pinMode(in3_27, OUTPUT);
  pinMode(enA_27, OUTPUT);
  pinMode(enB_27, OUTPUT);

  //U30
  pinMode(in1_30, OUTPUT);
  pinMode(in2_30, OUTPUT);
  pinMode(in3_30, OUTPUT);
  pinMode(enA_30, OUTPUT);
  pinMode(enB_30, OUTPUT);

  //U31
  pinMode(in1_31, OUTPUT);
  pinMode(in2_31, OUTPUT);
  pinMode(in3_31, OUTPUT);
  pinMode(enA_31, OUTPUT);
  pinMode(enB_31, OUTPUT);

  //Water Flow Sensor
  pinMode(WaterFlow, INPUT);

  //RTC DS1302
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  Serial.println();

  if (!Rtc.IsDateTimeValid()) {
    // Common Causes:
    //    1) first time you ran and the device wasn't running yet
    //    2) the battery on the device is low or even missing

    Serial.println("RTC lost confidence in the DateTime!");
    Rtc.SetDateTime(compiled);
  }

  if (Rtc.GetIsWriteProtected()) {
    Serial.println("RTC was write protected, enabling writing now");
    Rtc.SetIsWriteProtected(false);
  }

  if (!Rtc.GetIsRunning()) {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) {
    Serial.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.SetDateTime(compiled);
  } else if (now > compiled) {
    Serial.println("RTC is newer than compile time. (this is expected)");
  } else if (now == compiled) {
    Serial.println("RTC is the same as compile time! (not expected but all is fine)");
  }

  previousDay = now.Day();
  start_day = now.Day();
}

bool start = false;
void loop() {
  // Program Logic
  RtcDateTime now = Rtc.GetDateTime();

  if(start){
    calculateAllSensors();
    if (now.Minute() % 10 == 0) sendSensorValues(); //send Sensor every 10 minutes
    if (now.Hour() % 4 == 0) sendStatusToOrangePi(); //send status/notification every 4 hours
  }
  if(Serial3.available() > 0){
    char receivedChar = Serial3.read();
    if(receivedChar == 'S' && !start){
      start = true; // system has been started
      hydro_start();
    }else if(receivedChar == 'R' && start){
      hydro_start();
    }else{
      //check for health
      if(start){
        health = receivedChar;
        checkHealth();
      }
    }
  }

  //Read from Orange Pi
  //Expected Input
  //<mode, health, ppm lower threshold (int), ppm upper threshold (int), pH lower threshold (float), pH upper threshold (float)>
  //<O, H, 600, 750, 5.5, 6.5>
  recvWithStartEndMarkers();
  if (newData == true) {
    // this temporary copy is necessary to protect the original data
    // because strtok() used in parseData() replaces the commas with \0
    strcpy(tempChars, receivedChars);
    parseData();
    processParsedData();
    newData = false;
  } else {
    //default values
    mode = 'O';
    health = 'H';
    ppm_lower_threshold = 600;
    ppm_upper_threshold = 850;
    pH_lower_threshold = 5.5;
    pH_upper_threshold = 6.5;
  }

  if (start_day - now.Day() == 21) {
    //Send to OrangePi
    //Hey Grower! Time to Harvest
    Serial3.println("h");
  }
  
} //end loop


/** HEALTH CHECKER **/
void checkHealth(){
  Serial.println("Checking health status.");
  switch (health) {
    case 'T':
      // Tipburn (Calcium) is detected
      // Lower the lights by 10%
      percent = (10 * light_pwm) / 100;
      light_pwm = light_pwm - percent;
      if (light_pwm <= 0) {
        light_pwm = 255;
      }
      if (light_pwm >= 255) {
        light_pwm = 255;
      }
      //Turn Lights ON
      analogWrite(lights, light_pwm);

      //stabilize temp
      stabilizeTemp();
      stabilizePH();
      break;
    case 'B':
      // Brown Spots (Boron Toxicity)  is detected
      // Turn on water pump P1 to add 1L water
      refillWater(brown_spot_delay_add_one_L_water);
      //add solution A  - 0.5ml
      addSol_A(nutsol_delay_titration);
      controlPropeller(1);
      //Adjust PH
      stabilizePH();
      break;
    case 'Y':
      //Yellowing & Wilting (Nitrogen) is detected
      //Add 0.5 ml of Nutrient B
      addSol_B(nutsol_delay_titration);
      controlPropeller(1);
      //stabilize PPM
      stabilizePPM();
      //Adjust PH
      stabilizePH();
      break;
    case 'N':
      //Gray White Spots is detected
      //Increase Fans Speed
      //Increase fans speed
      stabilizeTemp();
      break;
    case 'H': //healthy
      break;
    case 'R': //reset
      mode = 'O';
      health = 'H';
      ppm_lower_threshold = 600;
      ppm_upper_threshold = 850;
      pH_lower_threshold = 5.5;
      pH_upper_threshold = 6.5;
      break;
    default:
      mode = 'O';
      health = 'H';
      ppm_lower_threshold = 600;
      ppm_upper_threshold = 850;
      pH_lower_threshold = 5.5;
      pH_upper_threshold = 6.5;
      break;
  }
}
/** SEND AND RECEIVE TO ORANGE PI **/
void calculateAllSensors(){
  Serial.println("Calculating all sensors.");
  temperature = getTemperature();
  humidity = getHumidity();
  ppm = GetTDS();
  pH = GetpH();
  NutSolLevel = main_res_calculate();
  water_reservoir_Level = water_res_calculate();
  Nutrient_A_Notify_Level = solution_a_calculate();
  Nutrient_B_Notify_Level = solution_b_calculate();
  pH_up_Notify_Level = ph_up_calculate();
  pH_down_Notify_Level = ph_down_calculate();
}
void sendStatusToOrangePi(){
  Serial.println("Sending status/notification to orange pi.");

  //Your Nutrient A Solution is Running Out! Kindly refill its container
  if(solution_a_calculate() < solution_lower_threshold) Serial3.println("a");
  //Your Nutrient B Solution is Running Out! Kindly refill its container
  if(solution_b_calculate() < solution_lower_threshold) Serial3.println("b");
  //Your PH Up Solution is Running Out! Kindly refill its container
  if(ph_up_calculate() < solution_lower_threshold) Serial3.println("u");
  //Your PH Down Solution is Running Out! Kindly refill its container
  if(ph_down_calculate() < solution_lower_threshold) Serial3.println("d");
  //Your Water Reservoir is Running Out! Kindly refill its reservoir.
  if(water_res_calculate() < 3) Serial3.println("w");

  //a Check Water FLow
  if (getWaterFlow() <= 0) {
    //Send to Orange Pi
    //Water Flow Not Detected
    Serial3.println("c");
  }
  //b Check PPM
  ppm = GetTDS();
  if (!(ppm > ppm_lower_threshold)) {
    if (!(ppm < ppm_upper_threshold)) {
      //Send to Orange Pi
      Serial3.println("PPM not in desired range");
    }
  }
  //c Check pH
  pH = GetpH();
  if (!(pH > pH_lower_threshold)) {
    if (!(pH < pH_upper_threshold)) {
      //Send to Orange Pi
      Serial3.println("pH not in desired range");
    }
  }
  //e Check Humidity
  humidity = getHumidity();
  if (!(humidity > humidity_lower_threshold)) {
    if (!(humidity < humidity_upper_threshold)) {
      //Send to Orange Pi
      Serial3.println("Humidity not in desired range");
    }
  }
  //f Check temperature
  temperature = getTemperature();
  if (temperature > temperature_threshold) {
    //Send to Orange Pi
    Serial3.println("Temperature not in desired range");
  }
}
void sendSensorValues(){
  Serial.println("Sending sensor values.");
  //Send Sensor Values to Orange Pin
  //Serial3.print("Temp ");
  Serial3.print(temperature);
  Serial3.print(",");
  //Serial3.print("Humidity ");
  Serial3.print(humidity);
  Serial3.print(",");
  //Serial3.print("PPM ");
  Serial3.print(ppm);
  Serial3.print(",");
  //Serial3.print("pH ");
  Serial3.print(pH);
  Serial3.print(",");
  //Serial3.print("Flow ");
  Serial3.print(getWaterFlow());
  Serial3.print(",");
  //Serial3.print("lux ");
  Serial3.print(0);
  Serial3.print(",");
  //Serial3.print("NutSol Level ");
  Serial3.print(NutSolLevel);
  Serial3.print(",");
  //Serial3.print("Water Reservoir Level ");
  Serial3.print(water_reservoir_Level);
  Serial3.print(",");
  //Serial3.print("Nut A Level ");
  Serial3.print(Nutrient_A_Notify_Level);
  Serial3.print(",");
  //Serial3.print("Nut B Level ");
  Serial3.print(Nutrient_B_Notify_Level);
  Serial3.print(",");
  //Serial3.print("pH up Level ");
  Serial3.print(pH_up_Notify_Level);
  Serial3.print(",");
  //Serial3.print("pH down Level ");
  Serial3.print(pH_down_Notify_Level);
  Serial3.println(); 
  //All parameters setup
  Serial3.println("p");
}
/*------------------------------- **/


/** ULTRASONIC MEASUREMENTS **/
//these area the measurements using ultrasonic
int water_level = 0;
String water_status_level = "low";
float water_res_measurements[] = {21, 20.4, 19.6, 18.5, 17.7, 17.40, 16.64, 15.82, 14.66, 13.90, 12.28, 11.83, 11.17, 10.08, 9.04, 8.54, 8, 7.26, 6.05, 5.84, 5.01, 4.31};
float main_res_measurements[] = {29.76, 28.96, 28.80, 28.56, 28.08, 27.48, 26.64, 26.16, 25.66, 25.24, 24.83, 24.27, 24.03, 23.58, 22.80, 22.51, 22.28, 21.56, 20.91, 20.45, 20.07, 19.54, 19.19, 18.79, 18.12, 17.57, 17.34, 16.77, 16.29, 15.83, 15.44, 14.96, 14.50, 14.04, 13.63, 12.77, 12.36, 11.93, 11.49, 11.03, 10.61, 10.24, 9.73, 9.45, 9.24, 8.45, 8.03, 7.64, 7.24, 6.85, 6.71};
float solutions_res_measurements[] = {15, 14.28, 12.12, 10.64, 8.57, 6.14, 5.27, 3.94};

int measureLevelSearch(float reservoir[], float captureVal, int arrSize){
  for(int i=0; i < arrSize; i++){
    if(i == (arrSize-1)){
      //maximum level
      water_level = i+1;
      return water_level;
    }else{
      if(captureVal < reservoir[i] && captureVal >= reservoir[i+1]){
        water_level = i+1;
        return water_level;
      }
    }
  }
  return 0;
}
int checkMeasureLevel(float measurements[], int measurements_size, int container, float ultrasonic_captured_val){
  //1-water | 2-main | 3-solutions
  int arrSize = 0; int low = 0; int mid = 0; int high = 0;
  switch(container){
    case 1:
      //water
      low = 4;
      mid = 16;
      high = 21;
      break;
    case 2:
      //main
      low = 7;
      mid = 20;
      high = 50;
      break;
    case 3:
      //solutions
      low = 1; //1-200ml
      mid = 4; //201-500ml
      high = 7; //501-800ml
      break;
    default:
      break;
  };
  if(ultrasonic_captured_val >= measurements[0]){
    //no water
    water_status_level = "No Water";
    water_level = 0;
    return water_level;
  }else{
    float mLS = measureLevelSearch(measurements, ultrasonic_captured_val, measurements_size);
    if(mLS >= 0 && mLS <= low)
      water_status_level = "LOW";
    else if(mLS > low && mLS <= mid)
      water_status_level = "MID";
    else if(mLS > mid && mLS < high)
      water_status_level = "HIGH";

    return mLS;
  }
  return 0;
}
int water_res_calculate(){
  int arrSize = sizeof(water_res_measurements) / sizeof(water_res_measurements[0]);
  int liters = checkMeasureLevel(water_res_measurements, arrSize, 1, ultrasonicFilter(trigPin12,echoPin12));

  Serial.print("WATER LEVEL STATUS: ");
  Serial.println(water_status_level);
  Serial.print("MEASURE WATER RESERVOIR: ");
  Serial.print(liters);
  Serial.println("liters");
  return liters;
}
int main_res_calculate(){
  int arrSize = sizeof(main_res_measurements) / sizeof(main_res_measurements[0]); //get array size
  int liters = checkMeasureLevel(main_res_measurements, arrSize, 2, ultrasonicFilter(trigPin10,echoPin10));

  Serial.print("WATER LEVEL STATUS: ");
  Serial.println(water_status_level);
  Serial.print("MEASURE MAIN RESERVOIR: ");
  Serial.print(liters);
  Serial.println("liters");
  return liters;
}
int solution_a_calculate(){
  int arrSize = sizeof(solutions_res_measurements) / sizeof(solutions_res_measurements[0]);
  int ml = checkMeasureLevel(solutions_res_measurements, arrSize, 3, ultrasonicFilter(trigPin8,echoPin8));

  Serial.print("SOLUTION A LEVEL STATUS: ");
  Serial.println(water_status_level);
  Serial.print("MEASURE SOLUTION A RESERVOIR: ");
  Serial.print(ml);
  Serial.println("ml");
  return ml;
}
int solution_b_calculate(){
  int arrSize = sizeof(solutions_res_measurements) / sizeof(solutions_res_measurements[0]);
  int ml = checkMeasureLevel(solutions_res_measurements, arrSize, 3, ultrasonicFilter(trigPin11,echoPin11));

  Serial.print("SOLUTION B LEVEL STATUS: ");
  Serial.println(water_status_level);
  Serial.print("MEASURE SOLUTION B RESERVOIR: ");
  Serial.print(ml);
  Serial.println("ml");
  return ml;
}
int ph_up_calculate(){
  int arrSize = sizeof(solutions_res_measurements) / sizeof(solutions_res_measurements[0]);
  int ml = checkMeasureLevel(solutions_res_measurements, arrSize, 3, ultrasonicFilter(trigPin6,echoPin6));

  Serial.print("PH UP LEVEL STATUS: ");
  Serial.println(water_status_level);
  Serial.print("MEASURE PH UP RESERVOIR: ");
  Serial.print(ml);
  Serial.println("ml");
  return ml;
}
int ph_down_calculate(){
  int arrSize = sizeof(solutions_res_measurements) / sizeof(solutions_res_measurements[0]);
  int ml = checkMeasureLevel(solutions_res_measurements, arrSize, 3, ultrasonicFilter(trigPin9,echoPin9));

  Serial.print("PH DOWN LEVEL STATUS: ");
  Serial.println(water_status_level);
  Serial.print("MEASURE PH DOWN RESERVOIR: ");
  Serial.print(ml);
  Serial.println("ml");
  return ml;
}

//**********************************


void hydro_start(){
  Serial.println("System Starting in 3.");
  delay(1000);
  Serial.println("System Starting in 2.");
  delay(1000);
  Serial.println("System Starting in 1.");
  delay(1000);
  Serial.println("System has started.");
  //calculate to add water maximum of 17liters
  int maxLitersToAdd = 17;
  int mainLevel = main_res_calculate(); //current water level in main res
  int remainingLitersToAdd = maxLitersToAdd - mainLevel;
  if(remainingLitersToAdd > 0){
    refillWater(remainingLitersToAdd);
  }
  controlMainPump(1); //turn on and check water flow
  controlMainPump(0); //turn off after checking waterflow
  
  stabilizePPM();
  addNutSols(maxLitersToAdd);
  stabilizePPM();
  stabilizePH();
  controlLight(1);
  stabilizeHum();
  stabilizeTemp();
  Serial.println("System is already working.");
}

void controlLight(int control){
  Serial.println("Light On");
  if(control){
    analogWrite(lights, light_pwm);
  }else{
    analogWrite(lights, 50);
  }
}
void stabilizePH(){
  pH = GetpH();
  Serial.println("Stabilizing PH");
  while(pH <= pH_lower_threshold){
    controlMainPump(0);
    addPH_up(nutsol_delay_titration);
    controlPropeller(1);
    controlMainPump(1);
    pH = GetpH();
  }
  while(pH >= pH_upper_threshold){
    controlMainPump(0);
    addPH_down(nutsol_delay_titration);
    controlPropeller(1);
    controlMainPump(1);
    pH = GetpH();
  }
}
void addNutSols(int liters){
  //if liters = 0, then it will perform titration
  ppm = GetTDS();
  Serial.println("Adding NutSols.");
  if(ppm < ppm_lower_threshold){
    if(liters == 0){
      while(ppm < ppm_lower_threshold){
        controlMainPump(0);
        addSol_A(nutsol_delay_titration);
        controlPropeller(1);
        addSol_B(nutsol_delay_titration);
        controlPropeller(1);
        controlMainPump(1);
        ppm = GetTDS();
      }
    }else if(liters > 0){
      controlMainPump(0);
      addSol_A(nutsol_delay * liters);
      controlPropeller(1);
      addSol_B(nutsol_delay * liters);
      controlPropeller(1);
      controlMainPump(1);
      ppm = GetTDS();
    }
  }else{
    Serial.print("The system encountered a critical issue while attempting to add more nutrients to the plants. Adding additional nutrients at this stage would result in a detrimental effect, posing a higher risk of harm to the plants rather than providing any benefits.");
  }
}
void stabilizePPM(){
  ppm = GetTDS();
  Serial.println("Stabilizing PPM.");
  while (ppm > ppm_upper_threshold){
    //ppm is more than the threshold - add water
    refillWater(0.5);
    controlPropeller(1);
    controlMainPump(1);
    ppm = GetTDS();
  }
  if(ppm < ppm_lower_threshold) addNutSols(0);
}
float GetpH() {
  int measurings = 0;
  Serial.println("Getting PH.");
  for (int i = 0; i < samples; i++) {
    measurings += analogRead(pHSensorPin);
    delay(1000);
  }
  float volt = 5 / adc_resolution * measurings / samples;
  float pH1 = 7 + ((2.5 - volt) / 0.18);
  return pH1;
}
void addPH_up(int time_to_delay){
  Serial.println("Adding PH Up.");
  pump(in1_30,in2_30,enA_30,1); //turn ON ph-UP pump
  delay(time_to_delay); //Delay in ms to add X mL of pH up / 0.05mL for other
  pump(in1_30,in2_30,enA_30,0); //turn OFF ph-UP pump
}
void addPH_down(int time_to_delay){
  Serial.println("Adding PH Down.");
  pump(in3_30,in4_30,enB_30,1); //turn ON ph-DOWN pump
  delay(time_to_delay); //Delay in ms to add X mL of pH Down / 0.05mL for other
  pump(in3_30,in4_30,enB_30,0); //turn OFF ph-DOWN pump
}
void addSol_A(int time_to_delay){
  Serial.println("Adding Solution A.");
  pump(in1_31,in2_31,enA_31,1); 
  delay(time_to_delay);
  pump(in1_31,in2_31,enA_31,0); 
}
void addSol_B(int time_to_delay){
  Serial.println("Adding Solution B.");
  pump(in3_31,in4_31,enB_31,1); 
  delay(time_to_delay);
  pump(in3_31,in4_31,enB_31,0); 
}
void controlMainPump(int control){
  Serial.println("Controlling Main Pump.");
  if(control){
    //on
    pump(in3_28,in4_28,enB_28,1);
    delay(30000); // wait for 30 seconds to allow the water to flow to the waterflow sensor
    if(getWaterFlow() <= 0){
      //send to orange pi
      // no water flowing
      //pump(in3_28,in4_28,enB_28,0);
      Serial3.println("c"); // send to orange pi
      Serial.println("Warning no waterflow. Check the system especially the pump!");
    }else{
      //water is circulating = good
      Serial.println("Water pump is working");
    }
  }else{
    //off
    pump(in3_28,in4_28,enB_28,0);
  }
}
void controlPropeller(int control){
  Serial.println("Controlling Main Pump.");
  if(control){
    //on
    // !IMPORTANT - ADD FUNCTION - CHECK WATER LEVEL FIRST BEFORE TURNING ON THE PROPELLER! //
    pump(in3_27,in4_27,enB_27,1); //turn ON propeller
    analogWrite(enB_27, 110);
    delay(10000); //10 seconds to mix
    pump(in3_27,in4_27,enB_27,0); //turn OFF propeller
  }else{
    //off
    pump(in3_27,in4_27,enB_27,0);
  }
}
void refillWater(float liters_to_add){
  Serial.println("Refilling water to main reservoir.");
  int secondsPerLiter = 31;
  if(water_res_calculate() > 3 && main_res_calculate() < 30){
    Serial.print("Refilling now.");
    refillAndMistingSol(1); //refill mode
    pump(in3_28,in4_28,enB_28,1); 
    delay(liters_to_add * (secondsPerLiter * 1000)); 
    pump(in3_28,in4_28,enB_28,0); 
    refillAndMistingSol(0); //turn off solenoids
    Serial.print("Done refilling.");
  }else{
    Serial.print("Warning cannot refill it could be water reservoir is lacking water or main reservoir is at maximum volume.");
  }
}

void checkTemp(){
  Serial.println("Checking temperature.");
  float temp = getTemperature();
  while(temp > temperature_threshold){
    //Increase Speed of fans F1 and F2
    //F1 & F2
    fans(1);
    temp = getTemperature();
  }
  if (temperature < temperature_threshold) {
    //Turn On fan F1 & F2 at lower speed
    fans(2);//Reduce F speed
  }
}
void fans(int control){
  Serial.println("Controlling fans.");
  switch(control){
    case 0:
      //turn off
      pump(in1_27,in2_27,enA_27,0);
      break;
    case 1:
      //turn on at full speed
      pump(in1_27,in2_27,enA_27,1);
      break;
    case 2:
      pump(in1_27,in2_27,enA_27,1);
      digitalWrite(enA_27, F_Speed);
      break;
    default:
      //turn off
      pump(in1_27,in2_27,enA_27,0);
      break;
  }
}
float GetTDS() {
  Serial.println("Getting PPM.");
  delay(5000);
  bool b = true;
  while (b) {
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U) {  //every 40 milliseconds,read the analog value from the ADC
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  //read the analog value and store into the buffer
      analogBufferIndex++;
      if (analogBufferIndex == SCOUNT) {
        analogBufferIndex = 0;
      }
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U) {  //every 800 milliseconds,read the analog value from the ADC
      printTimepoint = millis();
      for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

        // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;

        //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
        //temperature compensation
        float compensationVoltage = averageVoltage / compensationCoefficient;

        //convert voltage value to tds value
        tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
        b = false;
      }
    }
  }
  return tdsValue;
}

bool mist_control = true;
void turnOnMist(){
  /**Serial.println("Turning on Mist.");
  if(mist_control){
    refillAndMistingSol(2);
    pump(in1_28,in2_28,enA_28,1); 
    delay(30000); 
    pump(in1_28,in2_28,enA_28,0); 
    refillAndMistingSol(0);
    mist_control = false;
  }else{
    pump(in1_28,in2_28,enA_28,0); 
    refillAndMistingSol(0);
    Serial.print("Warning: Cannot turn on mist since it has been turned on.");
  }**/
}
void stabilizeHum(){
  Serial.println("Stabilizing humidity.");
  int maxIteration = 5;
  bool hum = false;
  humidity = getHumidity();
  fans(2);
  Serial.print("Humidity: ");
  Serial.println(getHumidity());
  while(humidity > humidity_upper_threshold && maxIteration > 0){
    Serial.print("Stabilizing humidity: ");
    Serial.println(humidity);
    fans(1);
    delay(5000);
    humidity = getHumidity();
    maxIteration--;
  }
  if(humidity > humidity_lower_threshold && mist_control){
    Serial.print("Stabilizing humidity: ");
    Serial.println(humidity);
    turnOnMist();
    delay(5000);
    humidity = getHumidity();
  }else{
    Serial.print("Warning: Cannot turn on mist since it has been turned on.");
  }
  
  Serial.print("Humidity: ");
  Serial.println(getHumidity());
}
void stabilizeTemp(){
  Serial.println("Stabilizing temperature.");
  int maxIteration = 5;
  temperature = getTemperature();
  while(temperature > temperature_threshold && maxIteration > 0){
    Serial.print("Stabilizing Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C ");
    fans(1);
    delay(5000);
    temperature = getTemperature();
    maxIteration--;
  }
}
float getHumidity() {
  Serial.println("Getting humidity.");
  float chk = DHT.read11(dhtPin);
  float humidity = DHT.humidity;
  return humidity;
}
float getTemperature() {
  Serial.println("Getting temperature.");
  float chk = DHT.read11(dhtPin);
  float temperature = DHT.temperature;
  return temperature;
}
void shutdownArduino() {
  Serial.println("Arduino shutdown.");
  wdt_enable(WDTO_15MS);  // Enable the Watchdog Timer with a short timeout
  while (1);  // Wait for the Watchdog Timer to reset the Arduino
}
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial3.available() > 0 && newData == false) {
    rc = Serial3.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

//============

//<mode, health, ppm lower threshold (int), ppm upper threshold (int), pH lower threshold (float), pH upper threshold (float)>
//<O, H, 400, 450, 5.5, 6.5>
void parseData() {  // split the data into its parts

  char* strtokIndx;  // this is used by strtok() as an index

  //mode
  strtokIndx = strtok(tempChars, ",");   // get the first part - the string
  strcpy(modeFromOrangePi, strtokIndx);  // copy it to modeFromOrangePi

  //health
  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  strcpy(healthFromOrangePi, strtokIndx);  // copy it to healthFromOrangePi

  //ppm lower threshold
  strtokIndx = strtok(NULL, ",");                    // this continues where the previous call left off
  ppmlowerthresholdFromOrangePi = atoi(strtokIndx);  // convert this part to an integer

  //ppm upper threshold
  strtokIndx = strtok(NULL, ",");                    // this continues where the previous call left off
  ppmupperthresholdFromOrangePi = atoi(strtokIndx);  // convert this part to an integer

  //pH lower threshold
  strtokIndx = strtok(NULL, ",");
  pHlowerthresholdFromOrangePi = atof(strtokIndx);  // convert this part to a float

  //pH upper threshold
  strtokIndx = strtok(NULL, ",");
  pHupperthresholdFromOrangePi = atof(strtokIndx);  // convert this part to a float
}

//============

void processParsedData() {
  mode = modeFromOrangePi;
  health = healthFromOrangePi;
  ppm_lower_threshold = ppmlowerthresholdFromOrangePi;
  ppm_upper_threshold = ppmupperthresholdFromOrangePi;
  pH_lower_threshold = pHlowerthresholdFromOrangePi;
  pH_upper_threshold = pHupperthresholdFromOrangePi;
}

void res_calibration(){
  //***************************************************
  //  analogWrite(lights, light_pwm); 
  //analogWrite(lights, 255);
  //pump(in1_27,in2_27,en _27,1); //fans
  //controlMainPump(1);
  
  /**MAIN RESERVOIR CALIBRATION**/
  for(int i=50; i<=50; i++){
    Serial.print("Adding water for ");
    Serial.print(i);
    Serial.println("liters");

    refillAndMistingSol(1);
    pump(in1_28,in2_28,enA_28,1); //water pump`
    delay(31000); // 31,000 ms = 31 = seconds = 1 liter 
    pump(in1_28,in2_28,enA_28,0);
    refillAndMistingSol(0);


    //Reading Distance
    Serial.print("Reading distance for ");
    Serial.print(i);
    Serial.println("liters");

    Serial.println("The distance for ");
    Serial.print(i);
    ultrasonicFilter(trigPin10,echoPin10);  
  }
  controlMainPump(1);
  delay(60000*5);
  analogWrite(lights, 0);
}

void refillAndMistingSol(int control){
  //1-refill 2-misting 3-off
  switch(control){
    case 0:
      //turn OFF ALL
      digitalWrite(solenoid24, LOW);
      digitalWrite(solenoid35, LOW);
      break;
    case 1:
      //turn on REFILL
      digitalWrite(solenoid24, HIGH);
      digitalWrite(solenoid35, LOW);
      break;
    case 2:
      //turn on MISTING
      digitalWrite(solenoid24, LOW);
      digitalWrite(solenoid35, HIGH);
      break;
    case 3:
      //turn on MISTING
      digitalWrite(solenoid24, HIGH);
      digitalWrite(solenoid35, HIGH);
      break;
    default:
      //turn OFF ALL
      digitalWrite(solenoid24, LOW);
      digitalWrite(solenoid35, LOW);
      Serial.println("Error: Wrong parameters for solenoid refill and misting sol.");
      break;
  }
}