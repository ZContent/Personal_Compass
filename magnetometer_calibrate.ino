#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#define SERIAL_BAUD   115200

// prevent large spike from skewing calibrations
#define MAXVAL  150

/* Assign a unique ID to these sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

float AccelMinX, AccelMaxX;
float AccelMinY, AccelMaxY;
float AccelMinZ, AccelMaxZ;

float MagMinX=1000., MagMaxX=-1000.;
float MagMinY=1000., MagMaxY=-1000.;
float MagMinZ=1000., MagMaxZ=-1000.;

uint32_t lastDisplayTime=0;

void setup(void) 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("LSM303 Calibration"); Serial.println("");
  
  /* Initialise the accelerometer */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  /* Initialise the magnetometer */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void loop(void) 
{
  
  
  /* Get a new sensor event */ 
  sensors_event_t accelEvent; 
  sensors_event_t magEvent; 
  
  accel.getEvent(&accelEvent);
  mag.getEvent(&magEvent);
  
  if (accelEvent.acceleration.x < AccelMinX) AccelMinX = accelEvent.acceleration.x;
  if (accelEvent.acceleration.x > AccelMaxX) AccelMaxX = accelEvent.acceleration.x;
  
  if (accelEvent.acceleration.y < AccelMinY) AccelMinY = accelEvent.acceleration.y;
  if (accelEvent.acceleration.y > AccelMaxY) AccelMaxY = accelEvent.acceleration.y;

  if (accelEvent.acceleration.z < AccelMinZ) AccelMinZ = accelEvent.acceleration.z;
  if (accelEvent.acceleration.z > AccelMaxZ) AccelMaxZ = accelEvent.acceleration.z;

  if ((magEvent.magnetic.x < MagMinX) && (abs(magEvent.magnetic.x) < MAXVAL)) MagMinX = magEvent.magnetic.x;
  if ((magEvent.magnetic.x > MagMaxX) && (abs(magEvent.magnetic.x) < MAXVAL)) MagMaxX = magEvent.magnetic.x;
  
  if ((magEvent.magnetic.y < MagMinY) && (abs(magEvent.magnetic.y) < MAXVAL)) MagMinY = magEvent.magnetic.y;
  if ((magEvent.magnetic.y > MagMaxY) && (abs(magEvent.magnetic.y) < MAXVAL)) MagMaxY = magEvent.magnetic.y;

  if ((magEvent.magnetic.z < MagMinZ) && (abs(magEvent.magnetic.z) < MAXVAL)) MagMinZ = magEvent.magnetic.z;
  if ((magEvent.magnetic.z > MagMaxZ) && (abs(magEvent.magnetic.z) < MAXVAL)) MagMaxZ = magEvent.magnetic.z;

  if ((millis() - lastDisplayTime) > 2000)
  {
    lastDisplayTime = millis();
    //Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
    //Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
    Serial.print("X: "); Serial.print(magEvent.magnetic.x,4); Serial.print(", Y: "); Serial.print(magEvent.magnetic.y,4); Serial.print(", Z:");Serial.println(magEvent.magnetic.z,4);
    float offsetx = (MagMaxX + MagMinX)/2*-1.;
    float offsety = (MagMaxY + MagMinY)/2*-1.;
    float offsetz = (MagMaxZ + MagMinZ)/2*-1.;
    float deltax = (MagMaxX-MagMinY)/2.;
    float deltay = (MagMaxY-MagMinY)/2.;
    float deltaz = (MagMaxZ-MagMinZ)/2.;
    float delta = (deltax + deltay + deltaz)/3.;
    float scalex = delta / deltax;
    float scaley = delta / deltay;
    float scalez = delta / deltaz;
    Serial.print("// Mag Minimums: "); Serial.print(MagMinX,4); Serial.print("  ");Serial.print(MagMinY,4); Serial.print("  "); Serial.println(MagMinZ,4);
    Serial.print("// Mag Maximums: "); Serial.print(MagMaxX,4); Serial.print("  ");Serial.print(MagMaxY,4); Serial.print("  "); Serial.println(MagMaxZ,4);
    Serial.print("// Mag Calibration Values: X:"); Serial.print(offsetx); Serial.print(", Y:"); Serial.print(offsety); Serial.print(", Z:"); Serial.println(offsetz);
    Serial.print("// Mag Scale Values: X:"); Serial.print(scalex); Serial.print(", Y:"); Serial.print(scaley); Serial.print(", Z:"); Serial.println(scalez);
    Serial.print("float magxOffset = "); Serial.print(offsetx,4); Serial.println(";");
    Serial.print("float magyOffset = "); Serial.print(offsety,4); Serial.println(";");
    Serial.print("float magzOffset = "); Serial.print(offsetz,4); Serial.println(";");
    Serial.print("float magxScale = "); Serial.print(scalex,4); Serial.println(";");
    Serial.print("float magyScale = "); Serial.print(scaley,4); Serial.println(";");
    Serial.print("float magzScale = "); Serial.print(scalez,4); Serial.println(";");
    Serial.println();
  } 
  if (Serial.available() > 0) {
    // key was pressed, reset stats
    Serial.read();
    Serial.println("Restarting calibration...");
    MagMinX=1000.;
    MagMaxX=-1000.;
    MagMinY=1000.;
    MagMaxY=-1000.;
    MagMinZ=1000.;
    MagMaxZ=-1000.;
  }

}
