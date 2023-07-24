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

//Solenoid Valves

//U35  Close for Refilling, Open for Misting
#define solenoid35 34
//U36
#define solenoid36 30

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
#define solenoid35 34
//U36
#define solenoid36 30

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

int Nut_Sol_Level = 90;  //NutSol will fill upto this %

//humidity
int humidity_lower_threshold = 50;
int humidity_upper_threshold = 60;

//temperature
int temperature_threshold = 22;

//Brown disease add 1L water
long brown_spot_delay_add_one_L_water = 1000;
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

void setup() {
  //Start Serial
  Serial.begin(9600);
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
  //pinMode(solenoid24, OUTPUT);
  //pinMode(solenoid35, OUTPUT);
  pinMode(solenoid36, OUTPUT);

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
  Serial3.println();

  if (!Rtc.IsDateTimeValid()) {
    // Common Causes:
    //    1) first time you ran and the device wasn't running yet
    //    2) the battery on the device is low or even missing

    Serial3.println("RTC lost confidence in the DateTime!");
    Rtc.SetDateTime(compiled);
  }

  if (Rtc.GetIsWriteProtected()) {
    Serial3.println("RTC was write protected, enabling writing now");
    Rtc.SetIsWriteProtected(false);
  }

  if (!Rtc.GetIsRunning()) {
    Serial3.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) {
    Serial3.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.SetDateTime(compiled);
  } else if (now > compiled) {
    Serial3.println("RTC is newer than compile time. (this is expected)");
  } else if (now == compiled) {
    Serial3.println("RTC is the same as compile time! (not expected but all is fine)");
  }

  previousDay = now.Day();
  start_day = now.Day();




  //***************************************************

  //  analogWrite(lights, light_pwm); 
  //analogWrite(lights, 255);
  //pump(in1_27,in2_27,en _27,1); //fans
  //controlMainPump(1);
  
  /**MAIN RESERVOIR CALIBRATION
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
  **/
  //controlMainPump(1);
  //delay(60000*5);
  //analogWrite(lights, 0);
  
  Serial.println("Starting in 1");
  delay(1000);
}
void loop() {
}

void hydro_start(){
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
}

void stabilizePH(){
  pH = GetpH();
  while(ph <= pH_lower_threshold){
    controlMainPump(0);
    addPH_up(nutsol_delay_titration);
    controlPropeller(1);
    controlMainPump(1);
    pH = GetpH();
  }
  while(ph >= pH_upper_threshold){
    controlMainPump(0);
    addPH_down(nutsol_delay_titration);
    controlPropeller(1);
    controlMainPump(1);
    pH = GetpH();
  }
}

void addNutSols(int liters){
  //if liters = 0, then it will stabilize
  ppm = GetTDS();
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
  while (ppm > ppm_upper_threshold){
    //ppm is more than the threshold - add water
    refillWater(0.5);
    controlPropeller(1);
    controlMainPump(1);
    ppm = GetTDS();
  }
}

float GetpH() {
  int measurings = 0;
  for (int i = 0; i < samples; i++) {
    measurings += analogRead(pHSensorPin);
    delay(10);
  }
  float volt = 5 / adc_resolution * measurings / samples;
  float pH1 = 7 + ((2.5 - volt) / 0.18);
  return pH1;
}

void addPH_up(int time_to_delay){
  pump(in1_30,in2_30,enA_30,1); //turn ON ph-UP pump
  delay(time_to_delay); //Delay in ms to add X mL of pH up / 0.05mL for other
  pump(in1_30,in2_30,enA_30,0); //turn OFF ph-UP pump
}

void addPH_down(int time_to_delay){
  pump(in3_30,in4_30,enB_30,1); //turn ON ph-DOWN pump
  delay(time_to_delay); //Delay in ms to add X mL of pH Down / 0.05mL for other
  pump(in3_30,in4_30,enB_30,0); //turn OFF ph-DOWN pump
}

void addSol_A(int time_to_delay){
  pump(in1_31,in2_31,enA_31,1); 
  delay(time_to_delay);
  pump(in1_31,in2_31,enA_31,0); 
}

void addSol_B(int time_to_delay){
  pump(in3_31,in4_31,enB_31,1); 
  delay(time_to_delay);
  pump(in3_31,in4_31,enB_31,0); 
}

void controlMainPump(int control){
  if(control){
    //on
    pump(in3_28,in4_28,enB_28,1);
    delay(10000); // wait for 10 seconds to allow the water to flow to the waterflow sensor
    if(getWaterFlow() <= 0){
      //send to orange pi
      // no water flowing
      pump(in3_28,in4_28,enB_28,0);
      Serial.println("Warning no waterflow. Check the system especially the pump!");
      shutdownArduino(); //shutdown arduino
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
  // !IMPORTANT - ADD FUNCTION - CHECK WATER LEVEL FIRST BEFORE TURNING ON THE WATER PUMP! //
  int secondsPerLiter = 31;
  refillAndMistingSol(1); //refill mode
  pump(in3_28,in4_28,enB_28,1); 
  delay(liters_to_add * (secondsPerLiter * 1000)); 
  pump(in3_28,in4_28,enB_28,0); 
  refillAndMistingSol(0); //turn off solenoids
}

void turnOnMist(){
  refillAndMistingSol(2);
  pump(in1_28,in2_28,enA_28,1); 
  delay(3000); 
  pump(in1_28,in2_28,enA_28,0); 
  refillAndMistingSol(0);
}

void checkTemp(){
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
float getHumidity() {
  float chk = DHT.read11(dhtPin);
  float humidity = DHT.humidity;
  return humidity;
}
float getTemperature() {
  float chk = DHT.read11(dhtPin);
  float temperature = DHT.temperature;
  return temperature;
}
void shutdownArduino() {
  wdt_enable(WDTO_15MS);  // Enable the Watchdog Timer with a short timeout
  while (1);  // Wait for the Watchdog Timer to reset the Arduino
}
