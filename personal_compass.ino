#include <FS.h> 
#include <SPIFFS.h>

#include <Adafruit_GPS.h>
#include <math.h>;
#include <ESP32_Servo.h>    //https://github.com/jkb-git/ESP32Servo
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#include <WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#define SERIAL_BAUD   115200
#define FEEDBACK_PIN A1
#define SERVO_PIN 14
//#define LED_PIN 13
#define LED_PIN 33
#define BUTTON_PIN 15
#define GPS_ENABLE_PIN 27
#define BOOST_ENABLE_PIN 12
#define SERIAL_BAUD   115200
#define POWER_TIMEOUT 60*15 // power timeout in seconds
#define ZONE_TIMEOUT 60*1 // how often to do the victory lap when in the zone? (in seconds)
#define PI 3.14159265
#define LONG_PRESS 2000 // amount of microseconds to be considered a long button press
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
//#define GPSECHO true

// what's the name of the hardware serial port?
HardwareSerial Serial1 = HardwareSerial(2); // for ESP32
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

bool shouldSaveConfig = false;
float truenorth = 0.;
// hard coded lat/long will be replaced by user settings
double targetlat = 28.419402;
double targetlon = -81.581192;
float targetzone = 1000; // in meters

Servo servo;
bool button_press = false;
bool button_press_on_boot = false;
uint32_t button_timer;
uint32_t display_timer;
uint32_t power_timer;
uint32_t zone_timer;

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Calibration offsets


float magxOffset = 0;
float magyOffset = 0;
float magzOffset = 0;
float magxScale = 1;
float magyScale = 1;
float magzScale = 1;


//From calibration sketch:

// Mag Minimums: -45.1818  -59.6364  -81.5306
// Mag Maximums: 50.6364  47.7273  0.0000
// Mag Calibration Values: X:-2.73, Y:5.95, Z:40.77
// Mag Scale Values: X:0.90, Y:0.93, Z:1.22
//float magxOffset = -2.7273;
//float magyOffset = 5.9545;
//float magzOffset = 40.7653;
//float magxScale = 0.9043;
//float magyScale = 0.9288;
//float magzScale = 1.2231;

char clat[20] = "";
char clon[20] = "";
char czone[20] = "";
char ctnorth[20] = "";

//flag for saving data

void saveConfigCallback () {
  shouldSaveConfig = true;
}

// config from smart phone
void getConfig()
{

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          if (json.containsKey("targetlat"))
            strcpy(clat, json["targetlat"]);
          if (json.containsKey("targetlon"))
            strcpy(clon, json["targetlon"]);
          if (json.containsKey("targetzone"))
            strcpy(czone, json["targetzone"]);
          if (json.containsKey("truenorth"))
            strcpy(ctnorth, json["truenorth"]);

          Serial.println(String("Config vars: ") + String(clat) + String(",") + String(clon) + String(",") + String(czone) + String(",") + String(ctnorth));
          targetlat = atof(clat);
          targetlon = atof(clon);
          targetzone = atof(czone);
          truenorth = atof(ctnorth);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  if(!digitalRead(BUTTON_PIN))
  {
    while(!digitalRead(BUTTON_PIN))
    {
      digitalWrite(LED_PIN, LOW);
      delay(500);
      digitalWrite(LED_PIN, HIGH);
      delay(500);
    }

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;
  
    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length
    WiFiManagerParameter custom_truenorth("tnorth", "True North (E is minus)", ctnorth, 20);
    WiFiManagerParameter custom_targetlat("lat", "Latitude", clat, 20);
    WiFiManagerParameter custom_targetlon("lon", "Longitude", clon, 20);
    WiFiManagerParameter custom_targetzone("zone", "Zone (in m)", czone, 20);

    //add all your parameters here
    wifiManager.addParameter(&custom_targetlat);
    wifiManager.addParameter(&custom_targetlon);
    wifiManager.addParameter(&custom_targetzone);
    wifiManager.addParameter(&custom_truenorth);
  
    //reset settings - for testing
    //wifiManager.resetSettings();
  
    //set minimum quality of signal so it ignores AP's under that quality
    //defaults to 8%
    //wifiManager.setMinimumSignalQuality();
    
    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    wifiManager.setTimeout(POWER_TIMEOUT);

    //exit after config instead of connecting
    wifiManager.setBreakAfterConfig(true);
    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //and goes into a blocking loop awaiting configuration
    wifiManager.autoConnect("Personal-Compass", "password");
    if(shouldSaveConfig)
    {
      strcpy(clat,custom_targetlat.getValue());
      strcpy(clon,custom_targetlon.getValue());
      strcpy(czone,custom_targetzone.getValue());
      strcpy(ctnorth,custom_truenorth.getValue());

      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
      json["targetlat"] = clat;
      json["targetlon"] = clon;
      json["targetzone"] = czone;
      json["truenorth"] = ctnorth;
    
      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }
      else
      {
        Serial.println(String("Saving config vars: ") + String(clat) + String(",") + String(clon) + String(",") + String(czone) + String(",") + String(ctnorth));
        json.printTo(Serial);Serial.println();
        json.printTo(configFile);
        configFile.close();
        //end save
        
        // load new values into memory
        targetlat = atof(clat);
        targetlon = atof(clon);
        targetzone = atof(czone);
        truenorth = atof(ctnorth);
      }
    }
 
    if(millis() >= power_timer)
    {
      // timeout, go to sleep
      deepSleep();
    }
  }
}

int checkButton()
{
  if(millis() >= power_timer)
  {
    // timeout, go to sleep
    deepSleep();
  }
  if (!digitalRead(BUTTON_PIN))
  {
    if(!button_press)
    {
      button_timer= millis();
      Serial.println("button pressed!");
    }
    button_press = true;
    if(millis() - button_timer > LONG_PRESS)
    {
      if(button_press_on_boot)
      {
        Serial.println("button pressed on boot");
        button_press_on_boot = false;
        // wait until button is unpressed before continuing
        while(!digitalRead(BUTTON_PIN))
        {
          digitalWrite(LED_PIN, LOW);
          delay(500);
          digitalWrite(LED_PIN, HIGH);
          delay(500);
        }
        button_press = false;
        // do something here
      }
      else
      {
        // switch mode after button is released (some blinky would help)
        // for future use
        while(!digitalRead(BUTTON_PIN))
        {
          digitalWrite(LED_PIN, LOW);
          delay(500);
          digitalWrite(LED_PIN, HIGH);
          delay(500);
        }
        Serial.println("button released (long)!");
        button_press = false;
        button_press_on_boot = false;
        return 2; // long press
      }      
    }
  }
  else
  {
    if(button_press)
    {
      Serial.println("button released!");
      if(!button_press_on_boot)
      {
        deepSleep();
      }
    }
    button_press = false;
    button_press_on_boot = false;
    return 1; //short press
  }
  return 0; // no button pressed
}

void Blink(byte pin, byte delayms, byte loops)
{
  for (byte i=0; i<loops; i++)
  {
    digitalWrite(pin,HIGH);
    delay(delayms);
    digitalWrite(pin,LOW);
    delay(delayms);
  }
  digitalWrite(pin,HIGH);
}

// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
double convertDegMinToDecDeg (float degMin) {
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}

float deg2rad(float val)
{
  return val * PI / 180.;
}

float rad2deg(float val)
{
  return val * 180. / PI;
}
// See https://www.movable-type.co.uk/scripts/latlong.html for math

double calculateDistance(float lat1,float lng1,float lat2,float lng2){
  double factor = 111200.; // for meters
  double x = (lat2 - lat1) * cos(deg2rad((lng1+lng2)/2));
  double y = (lng2 - lng1);
  double distance = sqrt(x*x + y*y) * factor;
  return distance;
}

float calculateBearing(double lat1,double lng1,double lat2,double lng2){
  float bearing;
  bearing = atan2(sin(deg2rad(lng2 - lng1))*cos(deg2rad(lat2)),cos(deg2rad(lat1))*sin(deg2rad(lat2)) - sin(deg2rad(lat1))*cos(deg2rad(lat2))*cos(deg2rad(lng2 - lng1)));
  return rad2deg(bearing);
}

// found this code from user aldeba here:
//https://forum.arduino.cc/index.php?topic=524993.0
int readFeedbackPos(int pwmPin)
{
 int tHigh;
 int tLow;
 int tCycle;

 float theta = 0;
 float dc = 0;
 int unitsFC = 360;
 float dcMin = 0.029;
 float dcMax = 0.971;
 float dutyScale = 1;
 while(1) {
   pulseIn(pwmPin, LOW);
   tHigh = pulseIn(pwmPin, HIGH);
   tLow =  pulseIn(pwmPin, LOW);
   tCycle = tHigh + tLow;
   if ((tCycle > 1000) && ( tCycle < 1200)) break;
 }

 dc = (dutyScale * tHigh) / tCycle;
 theta = ((dc - dcMin) * unitsFC) / (dcMax - dcMin);
 return theta;
}

void setup() {
  //while (!Serial); // wait until serial console is open, remove if not tethered to computer. Delete this line on ESP8266
  Serial.begin(SERIAL_BAUD);
  Serial.println("Improved Personal Compass Program");
  Serial.println("See more info at DanTheGeek.com");
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GPS_ENABLE_PIN, INPUT);
  pinMode(BOOST_ENABLE_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(GPS_ENABLE_PIN,HIGH); // enable GPS if disabled after deep sleep
  digitalWrite(BOOST_ENABLE_PIN,HIGH); // enable boost module if disabled after deep sleep
  delay(1000);
  display_timer = millis();
  power_timer = millis() + POWER_TIMEOUT*1000;


  if(!digitalRead(BUTTON_PIN))
  {
    button_press_on_boot = true;
  }
  servo.attach(SERVO_PIN);
  pinMode(FEEDBACK_PIN, INPUT);
  servo.write(90); // stop servo
  
  getConfig();  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    // should blink LED too
    Serial.println("Ooops, no LSM303 detected ... Check your wiring! (mag)");
    Blink(LED_PIN,100,20);
    deepSleep();
  }
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    // should blink LED too
    Serial.println("Ooops, no LSM303 detected ... Check your wiring! (accel)");
    Blink(LED_PIN,100,20);
    deepSleep();
  }
  
  // < 90 clockwise, decreasing
  // = 90 stopped
  // > 90 counterclockwise, increasing

  // wiggle pointer to show we are awake
  for(int i = 0; i < 2; i++)
  {
    servo.write(75);
    delay(10);
    servo.write(75);
    delay(50);
    servo.write(105);
    delay(10);
    servo.write(105);
    delay(50);
  }
  checkButton();
  // set bearing to 0 (x pointer on sensor graphic to verify pointer location)
  Serial.println("Resetting bearing");
  for(int i = 0; i < 300; i++)
  {
    gotoBearing(0);
    delay(10);
  }
  // spin dial until we get a GPS fix

  servo.write(78);
  Serial.println("Waiting for GPS");
  checkGPS();
  Serial.print("Found GPS signal");
}

void checkGPS()
{
  bool gps_blink = true;
  uint32_t gps_blink_timer = millis();
  bool wait = false;
  while((int) GPS.fix == 0)
  {
      wait = true;
      checkButton();
      if(gps_blink_timer + 1000 < millis())
      {
        gps_blink_timer = millis();
        Serial.println("waiting for GPS signal...");
        servo.write(78);
        if(gps_blink)
        {
           digitalWrite(LED_PIN, HIGH);
        }
        else
        {
          digitalWrite(LED_PIN, LOW);         
        }
        gps_blink = !gps_blink;
      }
      // loop until there is a GPS fix or timeout
      GPS.read();
      if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          continue; // we can fail to parse a sentence in which case we should just wait for another
      }
      if(GPSECHO)
      {
        displayGPS();        
      }
  }
  if(wait)
  {
    digitalWrite(LED_PIN, HIGH);
    servo.write(90); //stop
  }
}

void deepSleep()
{
 // go to sleep ( this function never actually returns )

 // pause servo if it is currently rotating
 servo.write(90);
 // blink LED to alert user we are going to sleep
 Blink(LED_PIN,200,3);
 // set bearing to 0 before going to sleep
  Serial.println("Resetting bearing");
  for(int i = 0; i < 200; i++)
  {

    gotoBearing(0);
    delay(10);
  }
  
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  gpio_pullup_en((gpio_num_t)GPS_ENABLE_PIN);
  gpio_pulldown_en((gpio_num_t)BOOST_ENABLE_PIN);
  
  esp_sleep_enable_ext1_wakeup(BIT(BUTTON_PIN),ESP_EXT1_WAKEUP_ALL_LOW);
  
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
}

// found this code from user aldeba to read the feedback from the servo:
//https://forum.arduino.cc/index.php?topic=524993.0
int readPos(int pwmPin)
{
 int tHigh;
 int tLow;
 int tCycle;

 float theta = 0;
 float dc = 0;
 int unitsFC = 360;
 float dcMin = 0.029;
 float dcMax = 0.971;
 float dutyScale = 1;
 while(1) {
   pulseIn(pwmPin, LOW);
   tHigh = pulseIn(pwmPin, HIGH);
   tLow =  pulseIn(pwmPin, LOW);
   tCycle = tHigh + tLow;
   if ((tCycle > 1000) && ( tCycle < 1200)) break;
 }

 dc = (dutyScale * tHigh) / tCycle;
 theta = ((dc - dcMin) * unitsFC) / (dcMax - dcMin);
 return theta;
}

void gotoBearing(float bearing)
{
  static float lastspeed = 0;
  static uint32_t displaytimer = millis();
  // servo direction:
  // < 90 clockwise, decreasing
  // = 90 stopped
  // > 90 counterclockwise, increasing
  float servoangle = readPos(FEEDBACK_PIN);
  float speed;
  int direction = 1; // counterclockwise
  float delta, a1, a2;
  float bearing2 = abs(bearing - 360);
  a1 = bearing2 - servoangle;
  if(a1 > 0)
  {
    a2 = a1 - 360;
  }
  else
  {
    a2 = a1 + 360;
  }
  if(abs(a2) < abs(a1))
  {
    delta = abs(a2);
    direction = a2 / abs(a2) * -1.; 
  }
  else
  {
    delta = abs(a1);
    direction = a1 / abs(a1) * -1.;
  }

  if(delta > 90)
  {
    speed = 18;
  }
  else if(delta > 45)
  {
    speed = 14;
  }
  else if(delta > 15)
  {
    speed = 10;
  }
  else if(delta > 2)
  {
    speed = 8;
  }
  else
  {
    speed = 0;
  }
  servo.write(90 + speed*direction);

  if((displaytimer + 200) < millis())
  {
    displaytimer = millis();
    Serial.println(String(bearing2) + String(" ") + String(servoangle) + String(" ") + String(speed * direction));
  }
}

/*
 * Get Magnetic Heading
 * Tilt/roll compensation algorithm used is discussed here in Appendix A:
 * https://www.st.com/resource/en/application_note/cd00269797.pdf
 */
float getHeading()
{
  sensors_event_t event, eventa;
  mag.getEvent(&event);
  accel.getEvent(&eventa);
  
  double mx0 = (event.magnetic.x + magxOffset) * magxScale;
  double my0 = (event.magnetic.y + magyOffset) * magyScale;
  double mz0 = (event.magnetic.z + magzOffset) * magzScale;
  double mlength = sqrt(mx0*mx0 + my0*my0 + mz0*mz0);
  double mx1 = mx0 / mlength;
  double my1 = my0 / mlength;
  double mz1 = mz0 / mlength;  
  
  double ax0 = eventa.acceleration.x;
  double ay0 = eventa.acceleration.y;
  double az0 = eventa.acceleration.z;
  double alength = sqrt(ax0*ax0 + ay0*ay0 + az0*az0);
  double ax1 = ax0 / alength;
  double ay1 = ay0 / alength;
  double az1 = az0 / alength;
  
  double pitch = asin(-ax1);
  double roll = asin(ay1 / cos(pitch));
  double mx2 = mx1*cos(pitch) + mz1*sin(pitch);
  double my2 = mx1*sin(roll)*sin(pitch) + my1*cos(roll) - mz1*sin(roll)*cos(pitch);
  double mz2 = -mx1*cos(roll)*sin(pitch) + my1*sin(roll) + mz1*cos(roll)*cos(pitch);
  
  return (float) atan2((double)my2,(double)mx2);
}

void displayGPS()
{
  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); Serial.print(':');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
  }
}

void loop() {
  static bool inzone = false;
  sensors_event_t event;

  checkGPS(); // make sure we still have a GPS signal
  
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  checkButton();
    
  double gpslat = convertDegMinToDecDeg(GPS.latitude);
  if(GPS.lat == 'S')
  {
    gpslat = 0. - gpslat;
  }
  double gpslon = convertDegMinToDecDeg(GPS.longitude);
  if(GPS.lon == 'W')
  {
    gpslon = 0. - gpslon;
  }
  
  float distance = calculateDistance(gpslat, gpslon, targetlat, targetlon);
  float bearing = calculateBearing(gpslat, gpslon, targetlat, targetlon);
  mag.getEvent(&event);
  
  //Serial.println(String("Ma:" + String("(") + String(event.magnetic.x) + "," + String(event.magnetic.y) + /*String(",") + String(event.magnetic.z)+ */ String(")")));

  // Calculate the angle of the vector y,x
  // non-tilt compensated heading:
  //float heading = atan2(event.magnetic.y + magyOffset,event.magnetic.x + magxOffset) * (180. / PI);
  // tilt compensated heading:
  float heading = getHeading() * (180. / PI);
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  
  float truebearing = heading - bearing + truenorth;
  if(truebearing < 0)
    truebearing = truebearing + 360;
  if(truebearing >= 360)
    truebearing = truebearing - 360;
      
  if(millis() - display_timer > 2000)
  {
    display_timer = millis();
    if(GPSECHO)
      displayGPS();

    /**/
    Serial.println(String("Location:  ") + String(GPS.latitude) + String(", ") + String(GPS.longitude));  
    Serial.println(String("Converted: ") + String(gpslat,8) + String(", ") + String(gpslon,8));  
    Serial.println(String("Target:   ") + String(targetlat,8) + String(", ") + String(targetlon,8));  
    Serial.print("Distance: "); Serial.print(distance,2); Serial.println(" m");
    Serial.println(String("Mag: " + String("(") + String(event.magnetic.x,4) + "," + String(event.magnetic.y,4) + String(",") + String(event.magnetic.z,4) + String(")")));
    Serial.print("Mag Heading: "); Serial.println(heading,2);
    Serial.print("Bearing: "); Serial.print(bearing,2); Serial.println(" deg");
    Serial.print("True North: "); Serial.print(truenorth,2); Serial.println(" deg");
    Serial.print("True Bearing: "); Serial.print(truebearing,2); Serial.println(" deg");
    Serial.println();

    //float oldheading = atan2(event.magnetic.y + magyOffset,event.magnetic.x + magxOffset) * (180. / PI);
    /**/
  }

  gotoBearing(truebearing);

  if(distance < targetzone)
  {
    if(!inzone)
    {
      inzone = true;
    }
    if(zone_timer < millis())
    {
      // victory lap time
      Serial.println(String("In the zone! ") + String(distance));
      servo.write(70); // rotate
      delay(50);
      servo.write(70); // just in case the first servo write didn't work
      Blink(LED_PIN,200,15); 
      servo.write(110); //now rotate the other way
      delay(50);
      servo.write(110); // just in case the first servo write didn't work
      Blink(LED_PIN,200,15);
      servo.write(90); // stop
      zone_timer = ZONE_TIMEOUT*1000 + millis();    
    }
  }
  else
  {
    inzone = false;
  }
}
