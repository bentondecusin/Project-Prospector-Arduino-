#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <math.h>

#include <Digital_Light_TSL2561.h>

//Define our pins
#define TRIG 2
#define ECHO 3

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

//General setup
void setup() 
{
  // put your setup code here, to run once:

  //buzzer
  pinMode(6, OUTPUT);
  
  //Light Sensor
  Serial.begin(9600);//initialize the serial monitor at 9600 baud rate
  Wire.begin();
  TSL2561.init();
  
  //Sonic Distance Sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //Accelerometer/Gyroscope/Magnometer
  lsm.begin();
  //setupSensor();
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}




//Light Sensor
  const int sensor=A0; //set the sensor to pin A0
//Temp Sensor
  const int B=4275;                 // B value of the thermistor
  const int R0 = 100000;            // R0 = 100k
  const int pinTempSensor = A1;     // Grove - Temperature Sensor connect to A0

//Sonic distance Sense
  double distance = 0;
  double total = 0;
  double avgDistance = 0;
  double values[5];
  double printDist;
  double brightness = 0;
  int sensorValue = 0;//create a var to store the value of the sensor
  int analogTemp;
  float temperature;
  float R;
  int urgencyVal = -1;
  
void loop() 
{
  //Loop for catching outliers from innaccurate sensor
  for(int i = 0; i < 5; i++)
  {
    digitalWrite(TRIG, HIGH);//Set TRIG pin HIGH
    delayMicroseconds(10);   //Pause 10 microseconds
    digitalWrite(TRIG, LOW); //Set TRIG pin LOW
    distance = pulseIn(ECHO, HIGH);
    distance = (distance / 58.0) / 100.0;  
    delay(25);             //Delay for half a second
    values[i]=distance;
  }
  quickSort(values,0,4);
  printDist = values[0];
  //Print Distance Sensor Data
  Serial.print("Distance:\t\t");
  Serial.print(printDist); //Print the distance 
  Serial.println(" Meters\n");
   //Light Data
  sensorValue=analogRead(sensor);//store the value of the sensor
  brightness = sensorValue;
  //brightness = 1000 - (sensorValue/10);
  //brightness = 10000.0/pow((sensorValue*10.0),(4.0/3.0));
  //brightness = (log((sensorValue*10.0)/(sensorValue*100.0))/log(10.0/100.0));
  Serial.print("Brightness:\t\t");//print on the serial monitor what's in the ""
  Serial.println(brightness);
  //Serial.print(TSL2561.readVisibleLux()); // print the light value
  Serial.println("");

 //Temp Sensor
 analogTemp = analogRead(pinTempSensor );
 R = 1023.0/((float)analogTemp)-1.0;
 R = 100000.0*R;
 float temperature=(1.0/(log(R/100000.0)/B+1/298.15)-273.15);//convert to temperature via datasheet ;
 Serial.print("Temperature:\t\t");Serial.print(temperature);Serial.println("°C");
 
 //Accelerometer/Gyroscope/Magnometer output data
 lsm.read();  /* ask it to read in the data */ 
 /* Get a new sensor event */ 
 sensors_event_t a, m, g, temp;

 lsm.getEvent(&a, &m, &g, &temp); 

  Serial.print("\nAcceleration:\n\tX:\t\t"); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\n\tY:\t\t"); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\n\tZ:\t\t"); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("\nMagnetism:\n\tX:\t\t"); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\n\tY:\t\t"); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\n\tZ:\t\t"); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("\nGyroscope:\n\tX:\t\t"); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\n\tY:\t\t"); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\n\tZ:\t\t"); Serial.print(g.gyro.z);      Serial.println(" dps");
  Serial.println("");

  
  //Serial.print("urgencyVal after default: ");Serial.println(urgencyVal);
  if(urgencyVal < checkUrgency(temperature, 't'))
  {
    urgencyVal = checkUrgency(temperature, 't');
  }
  //Serial.print("urgencyVal after temp: ");Serial.println(urgencyVal);
  if(urgencyVal < checkUrgency(distance, 'd'))
  {
    urgencyVal = checkUrgency(distance, 'd');
  }
  //Serial.print("urgencyVal after dist: ");Serial.println(urgencyVal);
  if(urgencyVal < checkUrgency(brightness, 'l'))
  {
    urgencyVal = checkUrgency(brightness, 'l');
  }
  //Serial.print("urgencyVal after brig: ");Serial.println(urgencyVal);
  if(urgencyVal < checkUrgency(a.acceleration.x, 'a'))
  {
    urgencyVal = checkUrgency(a.acceleration.x, 'a');
  }
  //Serial.print("urgencyVal after accX: ");Serial.println(urgencyVal);
  if(urgencyVal < checkUrgency(a.acceleration.y, 'a'))
  {
    urgencyVal = checkUrgency(a.acceleration.y, 'a');
  }
  //Serial.print("urgencyVal after accY: ");Serial.println(urgencyVal);
  if(urgencyVal < checkUrgency(a.acceleration.z, 'a'))
  {
    urgencyVal = checkUrgency(a.acceleration.z, 'a');
  }
  //Serial.print("urgencyVal after accZ: ");Serial.println(urgencyVal);
  playBeep(urgencyVal);
  delay(100);
  urgencyVal = -1; // resets urgency
}





int checkUrgency(double num, char measurement)
{
  int highest = -1;
  //vals : -1 null      0 Just A lil beep       1 notUR\rgent     2 urgent      3 veryUrgent
  switch (measurement)
  {
    case 't': //temperature
    {
      if(num >= 35 && num <= 42)
      {
        return 1;
      }
      else if(num > 42 && num <= 50)
      {
        return 2;
      }
      else if(num > 50)
      {
        return 3;
      }
      else
      {
        return -1;
      }
    }
    case 'd': //distance
    {
      if(num < 0.30 && num > 0.20)
      {
        return 1;
      }
      else if(num < 0.20 && num > 0.10)
      {
        return 2;
      }
      else if (num < 0.10)
      {
        return 3;
      }
    }
    /*case 'l': //light
    {
      if(num >= 790 || num <=40)
      {
        return 0;
      }
      else
      {
        return -1;
      }
    }*/
    case 'a': //acceleration
    {
      if((num >= (9.81*2.5)) && (num  < (9.81*3.5)))
      {
        return 1;
      }
      else if((num >= (9.81*3.5)) && (num < (9.81*4.5)))
      {
        return 2;
      }
      else if(num >= (9.81*4.5))
      {
        return 3;
      }
      else return -1;
    }
    default:
    {
      return -1;
    }
    break;
  }
}



void playBeep(int urgBeep)
{
  if(urgBeep > -1)
  {
    if(urgBeep == 0)
    {
      justALilBeep();
    }
    else if(urgBeep == 1)
    {
      notUrgent();
    }
    else if(urgBeep == 2)
    {
      urgent();
    }
    else if(urgBeep == 3)
    {
      veryUrgent();
    }
  }
  /*switch(urgBeep)
  {
    case 0:
    {
      justALilBeep();
    }
    case 1:
    {
      notUrgent();
    }
    case 2:
    {
      urgent();
    }
    case 3:
    {
      veryUrgent();
    }
    default:
    {
      
    }
    break;
  }*/
}


void justALilBeep()
{
  digitalWrite(6, HIGH);
  delay(20);
  digitalWrite(6, LOW);
  delay(20);
}
void notUrgent()
{
  digitalWrite(6, HIGH);
  delay(256);
  digitalWrite(6, LOW);
  delay(256);
}
void urgent()
{
  digitalWrite(6, HIGH);
  delay(128);
  digitalWrite(6, LOW);
  delay(128);
  digitalWrite(6, HIGH);
  delay(128);
  digitalWrite(6, LOW);
  delay(128);
}
void veryUrgent()
{
  digitalWrite(6, HIGH);
  delay(85);
  digitalWrite(6, LOW);
  delay(85);
  digitalWrite(6, HIGH);
  delay(85);
  digitalWrite(6, LOW);
  delay(85);
  digitalWrite(6, HIGH);
  delay(85);
  digitalWrite(6, LOW);
  delay(85);
}
//*****************************





void quickSort(double arr[], int left, int right) 
{
     int i = left, j = right;
     double tmp;
     double pivot = arr[(left + right) / 2];

     /* partition */
     while (i <= j) 
   {
           while (arr[i] < pivot)
                 i++;
           while (arr[j] > pivot)
                 j--;
           if (i <= j) 
       {
                 tmp = arr[i];
                 arr[i] = arr[j];
                 arr[j] = tmp;
                 i++;
                 j--;
           }
     }

     /* recursion */
     if (left < j)
           quickSort(arr, left, j);
     if (i < right)
           quickSort(arr, i, right);
}
