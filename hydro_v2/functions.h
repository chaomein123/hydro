#include <dht.h>

#ifndef functions_H
  #define functions_H
  #include <Arduino.h>
  #include <Wire.h>
  #include <ThreeWire.h>
  #include <RtcDS1302.h>
  int getMedianNum(int bArray[], int iFilterLen);
	int UltraMeasureDistance(int trigPin, int echoPin);
	int readWaterLevel();
	float GetTDS();
	float GetpH();
	float getTemperature();
	float getHumidity();
	int getLightIntensity();
	float getWaterFlow();

  void pump(int in1, int in2, int en, bool control);
  float ultrasonicFilter(int trigPin, int echoPin);
  float calculateAverage(int numReadings, float readings[]);
  void clearReadings(int numReadings, float readings[]);

#endif 
