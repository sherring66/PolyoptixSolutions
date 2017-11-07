// ---------------------------------------------------------------------------
// Example NewPing library sketch that pings 3 sensors 20 times a second.
// ---------------------------------------------------------------------------

#include <NewPing.h>

#define SONAR_NUM 1      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

int sensor0 = 0;
int sensor1 = 0;
int sensor2 = 0;
int sensor3 = 0;
int sensor4 = 0;
int sensor5 = 0;
int sensor6 = 0;
int sensor7 = 0;


NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(8, 7, MAX_DISTANCE) // Each sensor's trigger pin, echo pin, and max distance to ping. 
  
};

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop() {
  //sensor0 
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
  Serial.print("sensor0 = ");
  sensor0 = sonar[0].ping_cm();
  Serial.print(sensor0);
  Serial.print("cm ");

  //sensor1 
  digitalWrite(4, HIGH);
  //digitalWrite(5, LOW);
  //digitalWrite(6, LOW);
  
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
  Serial.print("sensor1 = ");
  sensor1 = sonar[0].ping_cm();
  Serial.print(sensor1);
  Serial.print("cm ");

  //sensor2 
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  //digitalWrite(6, LOW);
  
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
  Serial.print("sensor2 = ");
  sensor2 = sonar[0].ping_cm();
  Serial.print(sensor2);
  Serial.print("cm ");

  //sensor3 
  digitalWrite(4, HIGH);
  //digitalWrite(5, HIGH);
  //digitalWrite(6, LOW);
  
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
  Serial.print("sensor3 = ");
  sensor3 = sonar[0].ping_cm();
  Serial.print(sensor3);
  Serial.print("cm ");

  //sensor4 
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, HIGH);
  
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
  Serial.print("sensor4 = ");
  sensor4 = sonar[0].ping_cm();
  Serial.print(sensor4);
  Serial.print("cm ");
  
  //sensor5 
  digitalWrite(4, HIGH);
  //digitalWrite(5, LOW);
  //digitalWrite(6, HIGH);
  
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
  Serial.print("sensor5 = ");
  sensor5 = sonar[0].ping_cm();
  Serial.print(sensor5);
  Serial.print("cm ");

  //sensor6 
  //digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
  
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
  Serial.print("sensor6 = ");
  sensor6 = sonar[0].ping_cm();
  Serial.print(sensor6);
  Serial.print("cm ");

  //sensor7
  //digitalWrite(4, HIGH);
  //digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    
  Serial.print("sensor7 = ");
  sensor7 = sonar[0].ping_cm();
  Serial.print(sensor7);
  Serial.print("cm ");
  
  Serial.println();
}

