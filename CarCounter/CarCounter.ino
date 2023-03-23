#include "config.h"
#include <HCSR04.h>

// Define the length of the data store vector
#define LENGTH 7

// Söfnunartíðnin
#define Fs 7.6 

// Define the wait times
#define TIMEWAIT 750
#define TIMESEND 5000

int SENSITIVITY30 = 91;
int SENSITIVITY60 = 94;

int MAXDIST = 400;
int KLEN = 50; // Föst fjarlægð skynjara frá götu
int X1 = 50; // Minnsta fjarlægð bíls ofan á Klen
int X2 = 100; // Mesta fjarlægð bíls ofan á Klen
int XAVG = (X1+X2)/2;

// Initialize the measurement array
int measarray[LENGTH] = {1, 2, 3, 4, 5, 6, 7};

// Initialize the comparison arrays
int comparray30[LENGTH] = {MAXDIST, MAXDIST, (KLEN+XAVG), (KLEN+XAVG), (KLEN+XAVG), MAXDIST, MAXDIST}; // Comparison array for 30km/h
int comparray60[LENGTH] = {MAXDIST, MAXDIST, (KLEN+XAVG), (KLEN+XAVG), MAXDIST, MAXDIST, MAXDIST}; // Comparison array for 60km/h

// Set up pins for HC-SR04 Sensor
byte triggerPin = 12;
byte echoPin = 13;

// set up the 'Car counter' feed
AdafruitIO_Feed *counter = io.feed("counter");
// Set up the Minimum distance to car feed
AdafruitIO_Feed *min_dist_to_car = io.feed("min-dist-to-car");
AdafruitIO_Feed *max_dist_to_car = io.feed("max-dist-to-car");
AdafruitIO_Feed *dist_to_curb = io.feed("dist-to-curb");
AdafruitIO_Feed *max_meas_dist = io.feed("max-dist");
// Set up sensitivity feeds
AdafruitIO_Feed *sensitivity30_feed = io.feed("sensitivity30");
AdafruitIO_Feed *sensitivity60_feed = io.feed("sensitivity60");
// Set up start stop functionality
AdafruitIO_Feed *startstop_feed = io.feed("start-stop");

//Start Stop variable initialized
int startstop = 1;
// Car counter variable initialized
int carstotal = 0;
int carpassed = 0;
// Initialize wait variable
int wait = 0;
// Initialize timenow variable
unsigned long timenow;
// Initialize time since last data pack
unsigned long timefromsend = 0;

void setup() {

  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while(! Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  min_dist_to_car->onMessage(handleMinDist);
  max_dist_to_car->onMessage(handleMaxDist);
  dist_to_curb->onMessage(handleDistCurb);
  max_meas_dist->onMessage(handleMaxMeasDist);
  // Set up sensitivity feed handling
  sensitivity30_feed->onMessage(handleSensitivity30);
  sensitivity60_feed->onMessage(handleSensitivity60);
  // Set up start stop handling
  startstop_feed->onMessage(handleStartStop);

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  // Get the value set on minimum distance to car
  min_dist_to_car->get();
  max_dist_to_car->get();
  dist_to_curb->get();
  max_meas_dist->get();
  // Get sensitivity settings
  sensitivity30_feed->get();
  sensitivity60_feed->get();
  // Get Start Stop State
  startstop_feed->get();

  // HCSR04 sensor initialized
  HCSR04.begin(triggerPin, echoPin);

}

/* Decides wether or not to count the data as a car */
int countcar(int corrolation30, int corrolation60) {
  int count = 0;
  if (corrolation30 >= SENSITIVITY30 || corrolation60 >= SENSITIVITY60) {
    count = 1;

    Serial.print("\n\n----------------––––\n\n");
    Serial.print("        CAR COUNTED         ");
    Serial.print("\n\n----------------––––\n\n");

  }
  return count;
}

/* Loops array in time series data */
void looparray(int* inparray, int newval) {
  for(int i = 0; i <= LENGTH; i++) {
    // Remove print in order to speed up code
    /*
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(inparray[i]);
    */
    if (i < (LENGTH-1)) {
      inparray[i] = inparray[i+1];
    }
    else {
      inparray[i] = newval; 
    }
  }
}

/* Multiply arrays elementwise and sum */
unsigned long multiplyarray(int* x, int* y) {
  unsigned long sum = 0;
  for (int i = 0; i <= LENGTH-1; i++) {
    sum = sum + (x[i] * y[i]);
  }
  return sum/1000;
}

/* Multiply arrays elementwise in power of 2 and sum */
float multiplyarraypow2(int* x, int* y) {
  unsigned long suma = 0;
  unsigned long sumb = 0;
  for (int i = 0; i <= LENGTH-1; i++) {
    suma = suma + (x[i] * x[i]);
    sumb = sumb + (y[i] * y[i]);
  }
  suma = suma / 1000;
  sumb = sumb / 1000;
  double sqrd = sqrt((suma * sumb));
  return sqrd;
}

/* Cross corrolation */
int crosscor(int* inparray) {
  float cross_cor30, cross_cor60;
  
  cross_cor30 = (float) (multiplyarray(inparray, comparray30) / multiplyarraypow2(inparray, comparray30));
  cross_cor60 = (float) (multiplyarray(inparray, comparray60) / multiplyarraypow2(inparray, comparray60));
  /*
  Serial.print("Cross Correlation 30kmh: ");
  Serial.println(cross_cor30);

  Serial.print("Cross Correlation 60kmh: ");
  Serial.println(cross_cor60);
  */
  int iscar = countcar(cross_cor30*100, cross_cor60*100);
  return iscar;
}

int getdist() {
  double* distances = HCSR04.measureDistanceCm();

  return int(distances[0]);
}

void loop() {

  io.run();
  if (startstop){
    looparray(measarray, getdist());

    if (!wait) {
      carpassed = crosscor(measarray);
    }

    // If car is measured
    if (carpassed && !wait){
      wait = 1;
      timenow = millis();
      carpassed = 0;
      carstotal++;
    }

    // If wait is set and the wait time has passed
    if(wait && ((millis() - timenow) > TIMEWAIT)) {
      wait = 0;

    // If time from send is greater then 15 seconds then a data packet is sent
    if(millis() - timefromsend > TIMESEND) {
      counter->save(carstotal);
      timefromsend = millis();
    }
    

    }
  }
}

void handleMinDist(AdafruitIO_Data *data) {
  X1 = data->toInt();
  XAVG = (X1+X2)/2;
  Serial.print("Min dist to car set to: ");
  Serial.println(X1);
  setCompareArrays();
}

void handleMaxDist(AdafruitIO_Data *data) {
  X2 = data->toInt();
  XAVG = (X1+X2)/2;
  Serial.print("Max dist to car set to: ");
  Serial.println(X2);
  setCompareArrays();
}

void handleDistCurb(AdafruitIO_Data *data) {
  KLEN = data->toInt();
  Serial.print("Dist to curb set to: ");
  Serial.println(KLEN);
  setCompareArrays();
}

void handleMaxMeasDist(AdafruitIO_Data *data) {
  MAXDIST = data->toInt();
  Serial.print("Max measurable distance set to: ");
  Serial.println(MAXDIST);
  setCompareArrays();
}

void handleSensitivity30(AdafruitIO_Data *data) {
  SENSITIVITY30 = data->toInt();
  Serial.print("Sensitivity30 set to: "); 
  Serial.println(SENSITIVITY30);
}

void handleSensitivity60(AdafruitIO_Data *data) {
  SENSITIVITY60 = data->toInt();
  Serial.print("Sensitivity60 set to: ");
  Serial.println(SENSITIVITY60);
}

void handleStartStop(AdafruitIO_Data *data) {
  startstop = data->toInt();
}

/* Initialize comparison arrays */
void setCompareArrays(){
  for (int i = 0; i <= LENGTH; i++)
  {
    if (i < 2 || i > 4) {
    comparray30[i] = MAXDIST;
    comparray60[i] = MAXDIST;
    }
    if (i >= 2 && i <= 4){
    comparray30[i] = (KLEN+XAVG);
    comparray60[i] = MAXDIST;
    if (i != 4) {
      comparray60[i] = (KLEN+XAVG);
    }
    }
  }
  Serial.print("Comparray30: ");
  printarray(comparray30);
  Serial.print("Comparray60: ");
  printarray(comparray60);
}

void printarray(int* array) {
  
  for(int i = 0; i < 7; i++)
  {
    Serial.print(array[i]);
    Serial.print(", ");
  }
  Serial.print("\n");

}

/*
  Serial.print("\n\n");
  for(int i = 0; i < 7; i++)
  {
    Serial.print(comparray30[i]);
    Serial.print(", ");
  }
  Serial.print("\n");
  */

  /*
  // Print commented out to speed up code
  // save count to the 'counter' feed on Adafruit IO
  Serial.print("sending -> ");
  Serial.println(count);
  counter->save(count);
  // increment the count by 1
  count++;
  */