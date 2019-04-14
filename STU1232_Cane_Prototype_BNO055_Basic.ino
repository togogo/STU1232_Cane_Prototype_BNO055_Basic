/*
   From the HC05 Code
*/
#include <SoftwareSerial.h>
SoftwareSerial EEBlue(10, 11); // RX | TX

/*
   From the BNO055 Code
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define LED 13

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/*
   From the Flex Censor Code
*/
//Constants:
//const int ledPin = 3;   //pin 3 has PWM funtion
const int flexPin = A0; //pin A0 to read analog input

//Variables:
int value; //save analog value

//Variables...that are being sent out
int flex;//value from the flex censor
int x;//x-value of the accelerometer
int y;//y-value of the accelerometer
int z;//z-value of the accelerometer



void setup() {
  //from flex censor code
  pinMode(LED, OUTPUT);  //Set pin 13 as 'output'
  Serial.begin(9600);       //Begin serial communication

  //from the HC05 Code
  EEBlue.begin(9600);  //Default Baud for comm, it may be different for your Module.
  //Serial.println("The bluetooth gates are open.\n Connect to HC-05 from any other bluetooth device with 1234 as pairing key!.");

  //from BNO055 Code
  //Serial.println("Orientation Sensor Test");
  //Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  //delay(1000);

  bno.setExtCrystalUse(true);



}

void loop() {


  //from HC05 code
  // Feed any data from bluetooth to Terminal.
  if (EEBlue.available())
    //Serial.println("EEBlue is available");
    Serial.write(EEBlue.read());

  // Feed all data from termial to bluetooth
  if (Serial.available())
    //Serial.println("Serial is available");
    EEBlue.write(Serial.read());


  //from flex censor code

  value = analogRead(flexPin);         //Read and save analog value from potentiometer
  //Serial.println(value);               //Print value
  value = map(value, 500, 900, 0, 100);//Map value 0-1023 to 0-255 (PWM)
  //analogWrite(ledPin, value);          //Send PWM value to led
  //delay(20);                          //Small delay

  //from the BNO055 code
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */

    /*
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: "                                                          );
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");
    */
  

  //Serial.println(EEBlue.read());//debugging ... why is it showing -1 all the time?
  //Serial.println(EEBlue.read());

  //preprocessing before sending stuff out
  flex = value;
  x = event.orientation.x;
  y = event.orientation.y;
  z = event.orientation.z;

  //sending stuff out to processing
  //Serial.println(flex);
  //Serial.println(x);
  //Serial.println(y);
  //Serial.println(z);

  //alternative way of sending stuff to processing
  Serial.print(flex);
  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);


  if(z > 0){
  //the LED Part
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(1000);
  }
  else if(z < 0){
  //the LED Part
  digitalWrite(LED, HIGH);
  delay(10);
  digitalWrite(LED, LOW);
  delay(10);
  }
  




  delay(20);//small delay
}
