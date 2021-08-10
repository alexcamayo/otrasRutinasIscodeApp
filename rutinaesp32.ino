/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX30105 Breakout
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout
  This is a demo to show the reading of heart rate or beats per minute (BPM) using
  a Penpheral Beat Amplitude (PBA) algorithm.
  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.
  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)
  -INT = Not connected
  The MAX30105 Breakout can handle 5V or 3.3V I2C logic. We recommend powering the board with 5V
  but it will also run at 3.3V.
*/

#include <Wire.h>
#include "MAX30105.h"
#define LED_INTEGRADO  2 // Led azul integradl en el ESP32 con el fin de mostrar cuando el sensor está leyendo un valor, es decir, cuando el dedo está puesto.

#include "heartRate.h"

MAX30105 particleSensor;

//Incluir bluetooth
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
float beatAvg;

//Datos de salida
char DataBluetooth = 'N';
String dataout = "";
char buf[17];
int counter=0;

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("Sensor de Frecuencia Cardiaca"); //Bluetooth device name
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");
  pinMode(LED_INTEGRADO, OUTPUT); // Declaramos el led azul (pin 2) del ESP32 como salida.
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop() {

  if (SerialBT.available()) {
    DataBluetooth = SerialBT.read();
   Serial.print(DataBluetooth);
  }

  if (DataBluetooth=='N'){
      rates[RATE_SIZE]={}; //Array of heart rates
      rateSpot = 0;
      lastBeat = 0; //Time at which the last beat occurred
      beatsPerMinute=0;
      beatAvg=0;
      particleSensor.shutDown();
      SerialBT.write((const uint8_t*)"@APAGADO",8);
      digitalWrite(LED_INTEGRADO,LOW);
      counter=0;
  }
  else if(DataBluetooth=='S'){

    //Despertar el sensor
    counter=counter+1;
    if (counter==1){
      particleSensor.wakeUp();
    }
    if(counter>=10){
      counter=10;
    }

    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true){
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();
  
      beatsPerMinute = 60 / (delta / 1000.0);
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20){
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
  
       // Enviamos los valores del pulso y oxígeno los puertos virtuales V4 y V5 al celular.
         if(beatAvg>=0){
              dataout="";
              dataout.concat('@');
              dataout.concat('+');
              dataout.concat(beatsPerMinute);
              dataout.concat('+');
              dataout.concat(beatAvg);
              dataout.concat('+');
              Serial.print("DATBeast: "+dataout);
              Serial.print("\n");
              dataout.toCharArray(buf,17);
              SerialBT.write((const uint8_t*)buf,17);
              dataout="";
            }
      }
    }
    else{
      if (irValue > 50000){
        SerialBT.write((const uint8_t*)"@LEYENDO",8);
        //Serial.print("Leyendo");
      }
    }
    
  
    if (irValue < 50000){
      SerialBT.write((const uint8_t*)"@NOTDEDO",8);
      //Serial.print("No finger?");
      digitalWrite(LED_INTEGRADO,LOW);
    }
    else{
       digitalWrite(LED_INTEGRADO,HIGH);
    }
    //Serial.println();
      
    }
}
