#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define gravity 10.0
#define N 20

typedef float vect[N];

/*Decide the model of the accelerometer*/
//Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

/*Define variables*/
const int heartPin = A1;
int previousHV,HR;
int currentPeakTime=0, previousPeakTime=0;
int i,j,n;
float t=0;
vect heart_rate, batez_beste;
vect vect_x,vect_y,vect_z,time;

unsigned long previousMillis = 0;

//We start connection
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "test.mosquitto.org";
int port = 1883;

const char topic1[] = "arduino/ardatzak";
const char topic2[] = "arduino/ECG";
const char topic3[] = "arduino/HR";
const char topic4[] = "arduino/alarma/ecg";
const char topic5[] = "arduino/alarma/acc";


/*Necessary functions*/
float maximoa(vect v)
{
  int i;
  float maximoa=v[0];
  for(i=0;i<N;i++)
  {
    if (v[i] >= maximoa)
    {
      maximoa = v[i];
    }
  }
  return maximoa;
}
float minimoa(vect v)
{
  int i;
  float minimoa=v[0];
  for(i=0;i<N;i++)
  {
    if (v[i] <= minimoa)
    {
      minimoa = v[i];
    }
  }
  return minimoa;
}
float average(vect v)
{
  int i;
  float batura=0;
  for(i=0;i<N;i++)
  {
    batura+=v[i];
  }
  float batazbeste = batura/N;
  return batazbeste;
}
/*Mathematic model of accelerometers*/
int mathematical_model1(float meanX, float meanY, float meanZ,int n)
{
  float r;

  r=sqrt(pow(meanX,2)+pow(meanY,2)+pow(meanZ,2));

  if (r>3.0)
  {
    n+=1;
  }
  else if (r<=3.0)
  {
    n=0;
  }
  return n;
}




/*Hasieraketa*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
  {
    ;//Wait for   serial port to connect. Needed for native USB only.
  }
  //attempt to connect to Wifi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid,pass) != WL_CONNECTED)
  {
    //failed,retry
    Serial.print(".");
    delay(5000);
  }
  Serial.println("You are connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) 
  {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();


  //We set up the sensor
  Serial.println("Accelerometer Test"); Serial.println("");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the accelerometer ... check your connections */
    Serial.println("Ooops, no accelerometer detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  //accel.setRange(ADXL345_RANGE_16_G);
  //accel.setRange(ADXL345_RANGE_8_G);
 //accel.setRange(ADXL345_RANGE_4_G);
  accel.setRange(ADXL345_RANGE_2_G);
  // Range is fixed at +-200g
  Serial.println("");
}

void loop(){
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 20){
    previousMillis = currentMillis;

    currentPeakTime = millis();
    int Interval = currentPeakTime - previousPeakTime;

    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);
    int heartValue = analogRead(heartPin);
    Serial.println(heartValue);
    
    /*Get the acceleration values and put them in spherical coordinates*/
    float rawX = event.acceleration.x;
    float rawY = event.acceleration.y;
    float rawZ = event.acceleration.z;

    float raw = sqrt(rawX*rawX + rawY*rawY + rawZ*rawZ);

    float theta = atan2(sqrt(rawY*rawY + rawX*rawX),rawZ);
    float phi = atan2(rawY,rawX);

    float accelX =raw*sin(theta)*cos(phi) - gravity*sin(theta)*cos(phi);
    float accelY =raw*sin(theta)*sin(phi) - gravity*sin(theta)*sin(phi);
    float accelZ =raw*cos(theta) - gravity*cos(theta);

    vect_x[i] = accelX;
    vect_y[i] = accelY;
    vect_z[i] = accelZ;

    /*Calculate the mean of last N values*/
    
    float meanX = average(vect_x);
    float meanY = average(vect_y);
    float meanZ = average(vect_z);
    

    /*Mathematic model*/
    n=mathematical_model1(meanX,meanY,meanZ,n);
    
    /*Calculate HR*/
    if (heartValue - previousHV <= -100)
    {
      currentPeakTime = millis();
      int Interval = currentPeakTime - previousPeakTime;

      int new_HR = 60000/Interval;
      if (j==0 && heart_rate[j]!=0)
      {
        if (float(new_HR/heart_rate[N-1]) <= 1.2 && float(new_HR/heart_rate[N-1]) >=0.8 && new_HR<250)
        {
          heart_rate[j] = new_HR;
          j+=1;        

          mqttClient.beginMessage(topic3);
          mqttClient.print(new_HR);
          mqttClient.endMessage();
        }
      }
      else if(j==0 && heart_rate[j]==0)
      {
        if (new_HR<100 && new_HR > 40)
        {
          heart_rate[j] = new_HR;
          j+=1;
          mqttClient.beginMessage(topic3);
          mqttClient.print(new_HR);
          mqttClient.endMessage();
        }
      }
      else if (j!=0)
      {
        if (float(new_HR/heart_rate[j-1]) <= 1.2 && float(new_HR/heart_rate[j-1]) >=0.8 && new_HR<250)
        {
          heart_rate[j] = new_HR;
          j+=1;        

          mqttClient.beginMessage(topic3);
          mqttClient.print(new_HR);
          mqttClient.endMessage();
        }
      }
      previousPeakTime = currentPeakTime;
    }
    previousHV = heartValue;

    //Sending the messages
    
    mqttClient.beginMessage(topic1);
    mqttClient.print(vect_x[i]),mqttClient.print(" "),mqttClient.print(vect_y[i]),mqttClient.print(" "),mqttClient.print(vect_z[i]);
    mqttClient.endMessage();
    
    mqttClient.beginMessage(topic2);
    mqttClient.print(heartValue);
    mqttClient.endMessage();



    /*Once values are calculated, call to the alarm depending on the thresholds*/
    
    /*independienteki*/
    
    if (n>=30)
    {
      int alarma=1;
      mqttClient.beginMessage(topic5);
      mqttClient.print(alarma);
      mqttClient.endMessage();      
      delay(20000);

      //restart
      i=0;
      n=0;
      for(int k=0; k<N;k++)
      {
        vect_x[k] = 0;
        vect_y[k] = 0;
        vect_z[k] = 0;
      }
    }
    

    batez_beste[j] = average(heart_rate);
    if (maximoa(batez_beste)==average(heart_rate) && (maximoa(batez_beste)/minimoa(batez_beste)) >=1.15 && average(batez_beste)>=90)
    {
      int alarma=1;
      mqttClient.beginMessage(topic4);
      mqttClient.print(alarma);
      mqttClient.endMessage();   
      //restart
      j=0;
      for (int k=0; k<N;k++)
      {
        heart_rate[k] = 0;
        batez_beste[k] = 0;
      }
    }
    

    /*Restart the indexes*/
    i+=1;
    t+=0.02;
    if (i==N)
    {
      i=0;
    }
    if(j==N)
    {
      j=0;
    }
  }
  delay(20);
}
