// Include Libraries
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP8266WiFi.h>

// Defining Constants
// Ports
#define servoPin    0      //D3
#define motorIn2    16     //D0
#define motorIn1    12     //D6
#define motorE      13     //D7
#define IMU_SCI     5      //D1
#define IMU_SDA     4      //D2
#define Encoder     15     //D8
#define Leds        14     //D5
// Values

#define maxAngle    180    //Max Angle Of Rotation
#define minAngle    0      //Min Angle Of Rotation
#define mainSpeed   255    //The Constant Speed Of Moving Straight
#define maxSpeed    255    //Max Speed Of Rotation
#define minSpeed    255    //Min Speed Of Rotation
#define mainZ       0      //Main Z angle
#define maxZ        4      //Max Z angle
#define minZ        -4     //Min Z angle
#define maxDistance 12.0    //No. of Meters
int servoCenter = 90;    //Center Of The Steering Is At Angle 100 Of The Servo
// Replace with your network credentials
const char* ssid     = "OPPO Reno5";
const char* password = "24101306";

// Set web server port number to 80
WiFiServer server(80);
// Variable to store the HTTP request
String header;

// Creating Servo Object
Servo Servo1;

// Creating IMU Object
Adafruit_MPU6050 mpu;

// Functions Prototypes
void Web_Init(void);
void calibration(void);
void Degree(void);
void Direction(void);
void Speed(void);
void StraightForward(void);
void StraightForward2(void);
void stopMotion(void);
void stopMotion2(void);
void Distance(void);
double map_(double x, double in_min, double in_max, double out_min, double out_max);

// Variables
int iterations = 0;
int outputState = 0;
int pulses = 0;
int servoAngle = servoCenter;
int motionSpeed = mainSpeed;
float distance = 0.0;
float distanceOld = 0.0;
float gyroZ = 0.0;
float gyroZ0 = 0.0;
float gyroAngleZ = 0.0;
float gyroAngleZStore;
float angularZ = 0.0;
float dt; 
unsigned short times = 100;           // No. of Calibration Samples
unsigned long currentTime = millis(); // Current Time
unsigned long lastTime = 0;           // Last Time for Degree Calculation
unsigned long lTime = 0;              // Last Time for Stopping
unsigned long previousTime = 0;       // Previous time for Web Server
const long timeoutTime = 2000;        // Define timeout time in milliseconds (example: 2000ms = 2s)for Web Server
 
void setup() {

  // Serial Initaialize
  Serial.begin(115200);
  // Delaying Printing Until Serial Console Opens
  while(!Serial)
    delay(10);

  // Initializing Pins
  pinMode(servoPin,OUTPUT);
  pinMode(motorIn1,OUTPUT);
  pinMode(motorIn2,OUTPUT);
  pinMode(motorE,OUTPUT);
  pinMode(Encoder,INPUT_PULLUP);
  pinMode(Leds,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(Encoder), Distance, RISING); 

  // Initializing The Servo
  Servo1.attach(servoPin);

  // Initializing Web Server2
  Web_Init();

  // Initializing IMU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
      delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void loop() {
  // Reseting Servo at Forward Position
  Servo1.write(servoCenter);
  delay(250);
  
  calibration();
  
  //Resetting variables
  distance = 0.0;
  pulses = 0;
  gyroAngleZ = 0.0;
  
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    // loop while the client's connected
    while (client.connected() && currentTime - previousTime <= timeoutTime) 
    {
      currentTime = millis();         
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // Changing States
            if (header.indexOf("GET /Track1") >= 0) {
              outputState = 1;
            } else if (header.indexOf("GET /Track2") >= 0) {
              outputState = 2;
            } else if (header.indexOf("GET /Not_Moving") >= 0) {
              outputState = 0;
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>Team 11</h1>");
            client.println("<h1>Camaro Autonoumus Car</h1>");
            
            // Displaying the buttons
            if (outputState== 2)
            {
              client.println("<h2 style=\"color:gray\">Obstacles Track Is Running...</h2>");
              client.println("<p><a href=\"/Track2\"><button class=\"button\">10 Meters Same Track</button></a></p>");
              client.println("<p><a href=\"/Track2\"><button class=\"button button2\">Running...</button></a></p>");
              Servo1.write(servoCenter);
              delay(250);
              while(distance <= 2.0)
              {
                digitalWrite(Leds,HIGH);
                StraightForward();                  
              }
              Servo1.write(140);
              stopMotion2();
              while (distance != distanceOld)
              {
                distanceOld = distance;
                delay(1);
              }
              while(distance <= 2.5)
              {
                StraightForward2();
              }
              Servo1.write(40);
              stopMotion2();
              while (distance != distanceOld)
              {
                distanceOld = distance;
                delay(1);
              }
              while(distance <= 3)
              {
                StraightForward2();
              }
              Servo1.write(servoCenter);
              stopMotion2();
              while (distance != distanceOld)
              {
                distanceOld = distance;
                delay(1);
              }
              while(distance <= 6.0)
              {
                StraightForward();
              }
              Servo1.write(30);
              stopMotion2();
              while (distance != distanceOld)
              {
                distanceOld = distance;
                delay(1);
              }
              while(distance <= 9.0)
              {
                StraightForward2();
              }
              Servo1.write(180);
              stopMotion2();
              while (distance != distanceOld)
              {
                distanceOld = distance;
                delay(1);
              }
              while(distance <= 10.5)
              {
                StraightForward2();
              }
              Servo1.write(servoCenter);
              stopMotion2();
              while (distance != distanceOld)
              {
                distanceOld = distance;
                delay(1);
              }
              while(distance <= 13.0)
              {
                StraightForward();
              }
              stopMotion();
              digitalWrite(Leds,LOW);
              outputState = 0;
              iterations = 0;
            } 
             else if(outputState == 1)
             {
              client.println("<h2 style=\"color:gray\">10 Meters Same Track Is Running</h2>");
              client.println("<p><a href=\"/Track1\"><button class=\"button button2\">Running...</button></a></p>");
              client.println("<p><a href=\"/Track1\"><button class=\"button\">Obstacles Track</button></a></p>");
              Servo1.write(servoCenter);
              delay(250);
              //While Didn't Reach Distance
               while(distance <= maxDistance)
              {
                digitalWrite(Leds,HIGH);
                //Moving
                StraightForward();
                //client.println(Encoder);
                Serial.print("steps = ");
                Serial.print(pulses);
                Serial.print("\tDiatance = ");
                Serial.print(distance);
                Serial.print("\tDegree = ");
                Serial.print(gyroAngleZ);
                Serial.println();
              }
              //Stop
              stopMotion();
              digitalWrite(Leds,LOW);
              outputState = 0;
              iterations = 0;
            }
             else 
             {
              client.println("<h2 style=\"color:gray\">Choose Track</h2>");
              client.println("<p><a href=\"/Track1\"><button class=\"button\">10 Meters Same Track</button></a></p>");
              client.println("<p><a href=\"/Track2\"><button class=\"button\">Obstacles Track</button></a></p>");
             } 
   
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
  }
}

void Web_Init(void)
{
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  
    // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

double map_(double x, double in_min, double in_max, double out_min, double out_max) 
  {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calibration(void)
{
  gyroZ0 = 0.0;
  for(int i = 0; i < times; i++)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZ = g.gyro.z;
    gyroZ0 += gyroZ;
  }
  gyroZ0 /= times;
}

ICACHE_RAM_ATTR void Degree(void)
{
  if(iterations <= 3)
  {
    gyroAngleZ = 0.0;
    iterations++;
  }
  currentTime = millis();   //current time(ms)
  //get the current timestamp in Milliseconds since epoch time which is
  dt = (currentTime - lastTime) / 1000.0; //Differential time(s)
  lastTime = currentTime;                 //Last sampling time(ms)

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  gyroZ = g.gyro.z;
  
  angularZ = (gyroZ - gyroZ0) * (180/PI) * dt; //Convert rad/s to degree
  if (fabs(angularZ) < 0.05)
    angularZ = 0.00;
  gyroAngleZ += angularZ; //returns the absolute value of the z-axis rotazion integral 
}

ICACHE_RAM_ATTR void Direction(void)
{
  Degree();
  if(gyroAngleZ > maxZ)
    gyroAngleZStore = maxZ;
  else if(gyroAngleZ < minZ)
    gyroAngleZStore = minZ;
  else
    gyroAngleZStore = gyroAngleZ;
    
  if(gyroAngleZStore > 0.5)  // In Case Moving Left
    servoAngle = map_(gyroAngleZStore,mainZ,maxZ,servoCenter,minAngle);  // Rotate Right
  else if(gyroAngleZStore < -0.5)  // In Case Moving Right
    servoAngle = map_(gyroAngleZStore,mainZ,minZ,servoCenter,maxAngle);  // Rotate Left
  else
    servoAngle = servoCenter;
  Servo1.write(servoAngle);
}

void Speed(void)
{
  if(servoAngle > (servoCenter + 5))
    motionSpeed = map(servoAngle,servoCenter,maxAngle,maxSpeed,minSpeed);
  else if(servoAngle < (servoCenter - 5))
    motionSpeed = map(servoAngle,servoCenter,minAngle,maxSpeed,minSpeed);
  else
    motionSpeed = mainSpeed;
  analogWrite(motorE,motionSpeed);
}

void StraightForward(void)
{
  Direction();
  Speed();
  digitalWrite(motorIn1,HIGH);
  digitalWrite(motorIn2,LOW);
}

void StraightForward2(void)
{
  analogWrite(motorE,200);
  digitalWrite(motorIn1,HIGH);
  digitalWrite(motorIn2,LOW);
}

void stopMotion(void)
{
  currentTime = lTime = millis();
  while((lTime - currentTime) < 1000)
  {
    lTime = millis();
    Direction();
    digitalWrite(motorIn1,LOW);
    digitalWrite(motorIn2,LOW);
  }
}

void stopMotion2(void)
{
    digitalWrite(motorIn1,LOW);
    digitalWrite(motorIn2,LOW);
    delay(3);
}

ICACHE_RAM_ATTR void Distance(void)
{
  // Increment Pulses
   pulses += 1;
  // Calculating Distance Covered
  distance = (pulses/20.0) * 0.04 * PI; // Distance = (Total No. of Pulses/No. of Pulses Per Revolution) * Radius of Wheel * Pi
}
