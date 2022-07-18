#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>

#define strobe 2   //strobe is white led
#define beacon 3 
#define status 5
#define chipSelect 6
#define BMP280_ADDRESS 0x76

Adafruit_BMP280 bmp;
//Variables

float pressure;		//To store the barometric pressure (Pa)
float temperature;	//To store the temperature (oC)
int altimeter; 		//To store the altimeter (m) (you can also use it as a float variable)
unsigned long time;

File myFile;
File myFileLog;

void strobelight();
void getData();

void setup() {
  Serial.begin(9600);
  bmp.begin(BMP280_ADDRESS);


  if (SD.begin(chipSelect))
  {
    Serial.println("SD card is present & ready");
  } 
  else
  {
    Serial.println("SD card missing or failure");
    while(1); //halt program
  }

  if (SD.exists("log.txt")) 
  {
    Serial.println("Removing log.txt");
    SD.remove("log.txt");
    Serial.println("Done");
  }


  myFileLog = SD.open("log.txt", FILE_WRITE); 
  myFileLog.println("Session start!");


  if (SD.exists("csvData.txt")) 
  {
    Serial.println("Removing csvData.txt");
    myFileLog.println(">>>Removing csvData.txt");
    SD.remove("csvData.txt");
    Serial.println("Done");
    myFileLog.println("> Success!");
  }

   myFile = SD.open("csv.txt", FILE_WRITE);  

   if (myFile) // it opened OK
    {
    Serial.println("Writing headers to csv.txt");
    myFile.println("Time,Pressure,Temperature,Altitude");
    myFileLog.println(">>>Headers added! Success!");
    myFile.close(); 
    Serial.println("Headers written");
    }
  else {
    Serial.println("Error opening csv.txt");  
  Serial.println("Enter w for write, r for read or s for split csv");  
  }

  /* LED's Armed! */
  Serial.print("start!");
  pinMode(strobe, OUTPUT);
  pinMode(beacon, OUTPUT);
  pinMode(status, OUTPUT);
  digitalWrite(status,LOW);
  digitalWrite(strobe,LOW);
  digitalWrite(beacon,LOW);
  digitalWrite(status,HIGH);

  /* BMP 280 setup! */
  myFileLog.println(">>> All LED's ARMED!");
  Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
  //myFileLog.print(">>> Sensor ID: 0x"); myFileLog.println(bmp.sensorID(),16);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  /* LED's test */
  delay(2000); // led delay for system to get ready
  digitalWrite(status,LOW);
  delay(300); 
  digitalWrite(status,HIGH);
  digitalWrite(strobe,HIGH);
  digitalWrite(beacon,HIGH);
  delay(500); 
  digitalWrite(status,LOW);
  digitalWrite(strobe,LOW);
  digitalWrite(beacon,LOW);

  Serial.println(">>> Initializtion Success!");
  myFileLog.println(">>> Initializtion Success!");
}

void loop() {
  // put your main code here, to run repeatedly:
  strobelight();
  getData();

}

void strobelight(){
  /* Light show! */
  digitalWrite(strobe,HIGH);
  delay(150);
  digitalWrite(strobe,LOW);
  delay(500);
  for (int i=1;i<=3;i++){
    digitalWrite(beacon,HIGH);
    delay(40);
    digitalWrite(beacon,LOW);
    delay(40);
  }
  delay(500);
}

void getData(){
  time = millis(); //time
  time=time/1000;
  pressure = bmp.readPressure();
	temperature = bmp.readTemperature();
	altimeter = bmp.readAltitude (1050.35); //Change the "1050.35" to your city current barrometric pressure
  
  /* serial print for testing and debug! */
  Serial.print(time);Serial.print(",");Serial.print(pressure);Serial.print(",");
  Serial.print(temperature);Serial.print(",");Serial.println(altimeter);

  /* Write to File! */
  myFile.print(time);myFile.print(",");myFile.print(pressure);myFile.print(",");
  myFile.print(temperature);myFile.print(",");myFile.println(altimeter);
}