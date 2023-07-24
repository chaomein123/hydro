#include "functions.h"

/** DEFINITIONS **/
  #define solenoid24 36         //REFILL
  #define solenoid35 34         //MISTING
  #define in1_28 34             //P1 - HIGH PRESSURE PUMP
  #define in2_28 36             //P1 - HIGH PRESSURE PUMP
  #define enA_28 2              //P1 - HIGH PRESSURE PUMP
  #define in3_28 38             //P2 - WATER PUMP
  #define in4_28 40             //P2 - WATER PUMP
  #define enB_28 3              //P2 - WATER PUMP
  #define in1_27 42             //FAN1 & FAN2
  #define in2_27 44             //FAN1 & FAN2
  #define enA_27 4              //FAN1 & FAN2
  #define in3_27 46             //RS755 Fan Motor
  #define in4_27 48             //RS755 Fan Motor
  #define enB_27 5              //RS755 Fan Motor
  #define in1_30 A4             //PP 1 UP 
  #define in2_30 A5             //PP 1 UP 
  #define enA_30 6              //PP 1 UP 
  #define in3_30 A6             //PP 2 DOWN 
  #define in4_30 A7             //PP 2 DOWN 
  #define enB_30 7              //PP 2 DOWN 
  #define in1_31 A8             //PP 3 NUTSOL A
  #define in2_31 A9             //PP 3 NUTSOL A
  #define enA_31 8              //PP 3 NUTSOL A
  #define in3_31 A10            //PP 4 NUTSOL B
  #define in4_31 A11            //PP 4 NUTSOL B
  #define enB_31 9              //PP 4 NUTSOL B
  #define WaterFlow A0          //Water Flow Sensor
//**********************************

int UltraMeasureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration / 58;
  return distance;
}
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
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
    default:
      //turn OFF ALL
      digitalWrite(solenoid24, LOW);
      digitalWrite(solenoid35, LOW);
      Serial.println("Error: Wrong parameters for solenoid refill and misting sol.");
      break;

  }
}

void pump(int in1, int in2, int en, bool control){
  // Turn on water pump P1
  // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in1_28 and in2_28
  if(control){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(en, HIGH);
  }else{
    //Turn Pump off.
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(en, LOW);
  }
}

int X;
int Y;
float TIME2 = 0;
float FREQUENCY2 = 0;
float WATER2 = 0;
float TOTAL2 = 0;
float LS2 = 0;

float getWaterFlow() {
  X = pulseIn(WaterFlow, HIGH);
  Y = pulseIn(WaterFlow, LOW);
  TIME2 = X + Y;
  FREQUENCY2 = 1000000/TIME2;
  WATER2 = FREQUENCY2/7.5;
  LS2 = WATER2/60;
  if(FREQUENCY2 >= 0)
  {
    if(isinf(FREQUENCY2))
    {
      Serial.println("VOL. :0.00");
      Serial.print("TOTAL: ");
      Serial.print(TOTAL2);
      Serial.println("L");
      return 0;
    }
    else
    {
      TOTAL2 = TOTAL2 + LS2;
      Serial.println(FREQUENCY2);
      Serial.print("VOL.: ");
      Serial.print(WATER2);
      Serial.println("L/M");
      Serial.print("TOTAL: ");
      Serial.print(TOTAL2);
      Serial.println("L");
      return WATER2;
    }
  }else{
    return 0;
  }
  /**int sensorValue = analogRead(WaterFlow);  // Read analog value from water2flow sensor
  float flowRate = map(sensorValue, 0, 1023, 0, 100);  // Convert the sensor value to a flow rate between 0 and 100 (adjust the range as per your sensor's specifications)

  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print(", Flow Rate: ");
  Serial.println(flowRate);

  delay(1000);  // Delay between readings (adjust as needed)**/
}

//ultrasonic distance reader
float ultrasonicFilter(int trigPin, int echoPin){
  // Define the number of readings to average
  const int numReadings = 10;
  // Array to store sensor readings
  float readings[numReadings];
  // Variable to hold the filtered average value
  float filteredDistance = 0;
  // Counter to track the number of readings
  int readingCount = 0;

  for(int i=1; i<=numReadings; i++){
    Serial.print("Calculating Distance: ");
    Serial.print(i*10);
    Serial.println("%");
    //Wait 6 seconds to calm the water - every reading (10x = 1min)
    delay(6000);
    // Trigger the ultrasonic sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the pulse duration
    float duration = pulseIn(echoPin, HIGH);

    // Calculate the distance in centimeters
    float distance = duration * 0.034 / 2;

    // Store the reading in the array
    readings[readingCount] = distance;

    // Increment the reading counter
    readingCount++;

    // Check if the desired number of readings is reached
    if (readingCount >= numReadings) {
      // Calculate the average
      filteredDistance = calculateAverage(numReadings, readings);

      //Print the filtered distance
      Serial.print("Filtered Distance: ");
      Serial.print(filteredDistance);
      Serial.println(" cm");

      // Reset the counter and clear the array
      readingCount = 0;
      clearReadings(numReadings, readings);
    }
  }
  return filteredDistance;
}
float calculateAverage(int numReadings, float readings[]) {
  float sum = 0;
  for (int i = 0; i < numReadings; i++) {
    sum += readings[i];
  }
  return sum / numReadings;
}
void clearReadings(int numReadings, float readings[]) {
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
}

