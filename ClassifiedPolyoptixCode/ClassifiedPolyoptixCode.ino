#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>
//#include <Servo.h>

//Camera library
#include <Adafruit_VC0706.h>

// include the SD library:
#include <SPI.h>
#include <SD.h>

#define SONAR_NUM 1      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(8, 7, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
};

int sensor0 = 0;
int sensor1 = 0;
int sensor2 = 0;
int sensor3 = 0;
int sensor4 = 0;
int sensor5 = 0;
int sensor6 = 0;
int sensor7 = 0;

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);


// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;


// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
//const int chipSelect = 10;
#define chipSelect 10 //think this uses less space than const int

// Using SoftwareSerial (Arduino 1.0+) or NewSoftSerial (Arduino 0023 & prior):
#if ARDUINO >= 100
// On Uno: camera TX connected to pin 2, camera RX to pin 3:
SoftwareSerial cameraconnection = SoftwareSerial(2, 3);
// On Mega: camera TX connected to pin 69 (A15), camera RX to pin 3:
//SoftwareSerial cameraconnection = SoftwareSerial(69, 3);
#else
NewSoftSerial cameraconnection = NewSoftSerial(2, 3);
#endif

Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);

void setup() {

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT); //ABC to control MUX
   
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  //Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }

  /* Display some basic information on this sensor */
  //displaySensorDetails();
  
/* Comment out card print info for space saving
  Serial.print("\nInitializing SD card...");
  

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    return;
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.print("\nCard type: ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("\nVolume type is FAT");
  Serial.println(volume.fatType(), DEC);
  Serial.println();

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  Serial.print("Volume size (bytes): ");
  Serial.println(volumesize);
  Serial.print("Volume size (Kbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Mbytes): ");
  volumesize /= 1024;
  Serial.println(volumesize);


  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
  
End of card info printing */

  // When using hardware SPI, the SS pin MUST be set to an
  // output (even if not connected or used).  If left as a
  // floating input w/SPI on, this can cause lockuppage.
#if !defined(SOFTWARE_SPI)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if(chipSelect != 53) pinMode(53, OUTPUT); // SS on Mega
#else
  if(chipSelect != 10) pinMode(10, OUTPUT); // SS on Uno, etc.
#endif
#endif

  Serial.begin(9600);
  Serial.println("VC0706 Camera snapshot test");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }  
  
  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  // Print out the camera version information (optional)
  char *reply = cam.getVersion();
  if (reply == 0) {
    Serial.print("Failed to get version");
  } else {
    Serial.println("-----------------");
    Serial.print(reply);
    Serial.println("-----------------");
  }

  // Set the picture size - you can choose one of 640x480, 320x240 or 160x120 
  // Remember that bigger pictures take longer to transmit!
  
  cam.setImageSize(VC0706_640x480);        // biggest
  //cam.setImageSize(VC0706_320x240);        // medium
  //cam.setImageSize(VC0706_160x120);          // small

  // You can read the size back from the camera (optional, but maybe useful?)
  uint8_t imgsize = cam.getImageSize();
  Serial.print("Image size: ");
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");

  Serial.println("Snap in 3 secs...");
  delay(3000);

  if (! cam.takePicture()) 
    Serial.println("Failed to snap!");
  else 
    Serial.println("Picture taken!");
  
  // Create an image with the name IMAGExx.JPG
  char filename[13];
  strcpy(filename, "IMAGE00.JPG");
  for (int i = 0; i < 100; i++) {
    filename[5] = '0' + i/10;
    filename[6] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }
  
  // Open the file for writing
  File imgFile = SD.open(filename, FILE_WRITE);

  // Get the size of the image (frame) taken  
  uint16_t jpglen = cam.frameLength();
  Serial.print("Storing ");
  Serial.print(jpglen, DEC);
  Serial.print(" byte image.");

  int32_t time = millis();
  pinMode(8, OUTPUT);
  // Read all the data up to # bytes!
  byte wCount = 0; // For counting # of writes
  while (jpglen > 0) {
    // read 32 bytes at a time;
    uint8_t *buffer;
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);
    imgFile.write(buffer, bytesToRead);
    if(++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
      Serial.print('.');
      wCount = 0;
    }
    //Serial.print("Read ");  Serial.print(bytesToRead, DEC); Serial.println(" bytes");
    jpglen -= bytesToRead;
  }
  imgFile.close();

  time = millis() - time;
  Serial.println("done!");
  Serial.print(time); Serial.println(" ms elapsed");
  
}

void loop() { 
  //ultrasonic();
  //accelerometer();

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

void accelerometer(){
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  /* Note: You can also get the raw (non unified values) for */
  /* the last data sample as follows. The .getEvent call populates */
  /* the raw values used below. */
  //Serial.print("X Raw: "); Serial.print(accel.raw.x); Serial.print("  ");
  //Serial.print("Y Raw: "); Serial.print(accel.raw.y); Serial.print("  ");
  //Serial.print("Z Raw: "); Serial.print(accel.raw.z); Serial.println("");

  /* Delay before the next sample */
  delay(50);
}
void ultrasonic(){
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


