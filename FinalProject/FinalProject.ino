/* LIDAR SCANNING SYSTEM 
  Authors:
    Connor Hunt
    Joseph Lovin
    Micah Haack
*/


#include <ctype.h>
#include <LIDARLite.h>
#include <Wire.h>
#include <Servo.h>

#define MAX_POINT_COUNT 100

//enum op_mode{PIXEL, LINE, CLOUD}mode;
enum line_mode{HORIZ, VERT}line;

String inStr = "";
bool runScan = false;
volatile byte echoState = HIGH;  // HIGH -> echo on LOW -> echo off

int horiz_start_angle = -1;
int horiz_stop_angle = -1;
int vert_start_angle = -1;  // adjust by some constant
int vert_stop_angle = -1;
int num_points = 0;
int status_LED = PA5;

LIDARLite lidarLite;
Servo horizontal, vertical;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Initialize serial connection

  lidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  lidarLite.configure(0); // Change this number to try out alternate configurations

  horizontal.attach(PC6);
  vertical.attach(D8);
  horizontal.write(90);
  vertical.write(90);


  while( !Serial ) { ; }  // wait for serial to connect
  Serial.println("Enter anything to validate connection:");

  establishContact();
  Serial.println("LIDAR scanning system:");
  Serial.println("Enter start to begin scanning");
  Serial.println("Enter stop at any time to abort the scan process");
  
  grabInput();
  pinMode(status_LED, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  grabInputNoWait();

  inStr.toLowerCase();
  if(!inStr.compareTo("start\r")) {  // check if user wants to begin scan
    int mode_pick;
    do {
      Serial.println("Select scan mode:");
      Serial.println("1: 3-D point cloud (DEFAULT)");
      Serial.println("2: Point (custom)");
      Serial.println("3: 2-D line scan (custom)");
      Serial.println("4: 3-D point cloud (custom)");
      Serial.println("(5: Attempt LidarLite Reset)");
      grabInput();
      mode_pick = validAngle(inStr, 1, 5);
    } while(mode_pick <= 0);

    runScan = true;

    //Default point cloud
    if( mode_pick == 1) {
      Serial.println("Estimated scan time: 687 seconds");
      TakePointCloud(0, 180, 30, 120, 36, 18);
    }
    
    //Single point mode
    else if( mode_pick == 2 ) {
      obtainPixelParams();      
      Serial.println("Estimated scan time: 1.06 seconds");
      TakePoint(horiz_start_angle, vert_start_angle);
    }
    
    //Line mode
    else if( mode_pick == 3 ) {
      obtainLineParams();
      Serial.print("Estimated scan time: ");
      Serial.print(1.06*num_points);
      Serial.println(" seconds");
      if(line == HORIZ) {
        TakeLine(horiz_start_angle, horiz_stop_angle, num_points, vert_start_angle, true);
      }
      else {
        TakeLine(vert_start_angle, vert_stop_angle, num_points, horiz_start_angle, false);
      }
    }
    
    //Cloud mode
    else if( mode_pick == 4 ) {
      obtainCloudParams();
      Serial.print("Estimated scan time: ");
      Serial.print(1.06*pow(num_points,2));
      Serial.println(" seconds");
      TakePointCloud(horiz_start_angle, horiz_stop_angle, vert_start_angle, vert_stop_angle, num_points, num_points);
    }
    
    //LidarLite Reset
    else if( mode_pick == 5) {
      Serial.println("Attempting reset...");
      delay(100);
      lidarLite.begin(0, true);
      lidarLite.configure(0);
      delay(100);
    }

    runStop();
  }
}

void TakePoint(double x, double y) {
  if( checkForStop() )
    return;
  horizontal.write(x);
  vertical.write(y);
  
  delay(1000);

  digitalWrite(status_LED, HIGH);
  Serial.print("At (");
  Serial.print(horizontal.read());
  Serial.print(", ");
  Serial.print(vertical.read());
  Serial.print("), the distance is ");
  int dist = lidarLite.distance();
  Serial.print(dist);
  Serial.print(" cm. Intensity: ");
  uint8_t inTentCity = intensity();
  Serial.println(inTentCity);
  delay(10);
  digitalWrite(status_LED, LOW);
}

void TakeLine(double minNum, double maxNum, double numPoints, double otherAxis, bool isHorizontal) {
  if( checkForStop())
    return;
  double res = (maxNum-minNum) / numPoints;
  if(isHorizontal) {
    double y = otherAxis;
    for(double x = minNum; x < maxNum; x+=res) {
      if( checkForStop() )
        return;
      TakePoint(x,y);
    }
  }
  else {
    double x = otherAxis;
    for(double y = minNum; y < maxNum; y+=res) {
      TakePoint(x,y);
      if( checkForStop() )
        return;
    }
  }
}

void TakePointCloud(int minX, int maxX, int minY, int maxY, int xPoints, int yPoints) {
  double yRes = (maxY-minY)/yPoints;
  for(double y = minY; y < maxY; y+=yRes) {
    TakeLine(minX, maxX, xPoints, y, true);
    if( checkForStop() )
      return;
  }
}

/* helper function to check if the user input is a valid angle */
/* returns the angle on success, -1 on error */
int validAngle(String str, int min_val, int max_val) {

  // check if a number
  int i;
  for( i = 0; i < str.length() - 1; i++ ) {
    if( !isDigit(str[i]))
      return -1;
  }

  // convert to int and check bounds
  int angle = str.toInt();

  if( angle < min_val || angle > max_val ) {
    return -1;
  }

  return angle;
  
}

/* helper function to collect input from serial com */
void grabInput() {

    // wait for user to begin inputing
    while (Serial.available() <= 0) { delay(100); }
  
    if( Serial.available() > 0 ) {  // read serial input
    inStr = Serial.readString();    
    if (echoState) {
      Serial.println(inStr);
    }
  } else {
    Serial.flush();
  }
}

/* helper function to collect input from serial com */
void grabInputNoWait() {
    
    if( Serial.available() > 0 ) {  // read serial input
    inStr = Serial.readString();    
    if (echoState) {
      Serial.println(inStr);
    }
  } else {
    Serial.flush();
  }
}

/*
 * helper function to collect cloud mode operating params
 * need to pick start / stop angles for horiz and vert
 * collect num_points as well
 */
void obtainCloudParams() {

  // prompt for start / stop horiz angles
  while( horiz_start_angle == -1 ) {
    Serial.println("Select Horizontal Start Angle ( 0 to 180 ): ");
    grabInput();
    horiz_start_angle = validAngle(inStr, 0, 180);
  }

  while( horiz_stop_angle == -1 ) {
    Serial.println("Select Horizontal Stop Angle ( " + String(horiz_start_angle) + " to 180 ):");
    grabInput();
    horiz_stop_angle = validAngle(inStr, horiz_start_angle, 180);
  }

  // prompt for start / stop vert angles
  while( vert_start_angle == -1 ) {
    Serial.println("Select Vertical Start Angle ( 30 to 120 ): ");
    grabInput();
    vert_start_angle = validAngle(inStr, 30, 120);
  }

  while( vert_stop_angle == -1 ) {
    Serial.println("Select Vertical Stop Angle ( " + String(vert_start_angle) + " to 120 ):");
    grabInput();
    vert_stop_angle = validAngle(inStr, vert_start_angle, 120);
  }

  // grab number of points

  // find max horiz, vert
  int mhnp = ( (horiz_stop_angle - horiz_start_angle) / 5 ) + 1;
  int mvnp = ( (vert_stop_angle - vert_start_angle) / 5 ) + 1;

  // max num points
  int mnp = mhnp * mvnp;

  while( num_points <= 0 ) {
    Serial.println("Select number of points to record ( 1 to " + String(mnp) +  " ):");
    grabInput();
    num_points = validAngle(inStr, 1, mnp);
  }

  num_points = sqrt(mnp);

  String printStr = "Starting cloud scan starting at (" + String(horiz_start_angle) + ", " + String(vert_start_angle) + ") and ending at (" + String(horiz_stop_angle) + ", " + String(vert_stop_angle) + ") with " + String(num_points) + " data points";
  Serial.println(printStr);
  
}

/*
 *  helper function to collect line mode operating params
 *  need to pick which direction (horiz or vert)
 *  as well as starting / stopping angle
 *  collect num_points as well
 */
void obtainLineParams() {

  int mode_val = -1;
  while( mode_val == -1) {
    Serial.println("Select a vertical or horizontal line scan:");
    Serial.println("1: Horizontal");
    Serial.println("2: Vertical");
    grabInput();
    mode_val = validAngle(inStr, 1, 2);
  }
  
  // update enum on direction
  if( mode_val == 1 )
    line = HORIZ;
  if( mode_val == 2 )
    line = VERT;

  String dir = "horizontal";

  int mnp = 0; // max num points to keep angle change >= 5 deg

  // prompt for start / stop horizontal angles, and fixed vert angle
  if( line == HORIZ ) {

    while(horiz_start_angle == -1) {
      Serial.println("Select Horizontal Start Angle ( 0 to 180 ): ");
      grabInput();
      horiz_start_angle = validAngle(inStr, 0, 180);
    }

    while(horiz_stop_angle == -1) {
      Serial.println("Select Horizontal Stop Angle ( " + String(horiz_start_angle) + " to 180 ):");
      grabInput();
      horiz_stop_angle = validAngle(inStr, horiz_start_angle, 180);
    }

    while(vert_start_angle == -1) {
      Serial.println("Select Vertical Angle ( 30 to 120 ):");
      grabInput();
      vert_start_angle = validAngle(inStr, 30, 120);
    }

    mnp = ( (horiz_stop_angle - horiz_start_angle) / 5 ) + 1;
    vert_stop_angle = vert_start_angle;

  }

  // prompt for start / stop vertical angles, and fixed horizontal angle
  if( line == VERT ) {

    while(vert_start_angle == -1) {
      Serial.println("Select Vertical Start Angle ( 30 to 120 ): ");
      grabInput();
      vert_start_angle = validAngle(inStr, 30, 120);
    }

    while(vert_stop_angle == -1) {
      Serial.println("Select Vertical Stop Angle ( " + String(vert_start_angle) +" to 120 ):");
      grabInput();
      vert_stop_angle = validAngle(inStr, vert_start_angle, 120);
    }

    while(horiz_start_angle == -1) {
      Serial.println("Select Horizontal Angle ( 0 to 180 ):");
      grabInput();
      horiz_start_angle = validAngle(inStr, 0, 180);
    }

    mnp = ( (vert_stop_angle - vert_start_angle) / 5 ) + 1;
  
    horiz_stop_angle = horiz_start_angle;

    dir = "vertical";
  }

  // grab number of points
  while( num_points <= 0 ) {
    Serial.println("Select number of points to record ( 1 to " + String(mnp) + " ):");
    grabInput();
    num_points = validAngle(inStr, 1, mnp);
  }
  
  String printStr = "Starting " + dir + " line scan starting at (" + String(horiz_start_angle) + ", " + String(vert_start_angle) + ") and ending at (" + String(horiz_stop_angle) + ", " + String(vert_stop_angle) + ") with " + String(num_points) + " data points";
  Serial.println(printStr);
  
}


/*
 * helper function to collect pixel mode operating parameters.
 * since we only need 1 angle for horiz / verical pixel mode stores them in 
 * the start_angle values
 * only one data collection so set num_points == 1;
 */
void obtainPixelParams() {

  while(horiz_start_angle == -1) {
    Serial.println("Select Horizontal Angle ( 0 to 180 ): ");
    grabInput();

    horiz_start_angle = validAngle(inStr, 0, 180);
  }

  while(vert_start_angle == -1) {
    Serial.println("Select Vertical Angle ( 30 to 120 ): ");
    grabInput();

    vert_start_angle = validAngle(inStr, 30, 120);
  }

  num_points = 1;

  String printStr = "Starting pixel scan at (" + String(horiz_start_angle) + ", " + String(vert_start_angle) + ")";
  Serial.println(printStr);
 
}

bool checkForStop() {
  if(!runScan)
    return true;
    
  grabInputNoWait();

  if( !inStr.compareTo("stop\r") ) {
    runScan == false;
    runStop();
    return true;
  }

  return false;
  
}

void runStop() {
    Serial.println("Stopping scan...");
    horiz_start_angle = -1;
    horiz_stop_angle = -1;
    vert_start_angle = -1;
    vert_stop_angle = -1;
    num_points = 0;
    
    runScan = false;
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('?');   // send a single character
    delay(300);
  }
  Serial.println("");
  Serial.println("CONNECTED");   // carriage return
}

uint8_t intensity()
{
byte isBusy = 1;
uint8_t intensity;
int loopCount;

// Poll busy bit in status register until device is idle
while(isBusy)
{
// Read status register
Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
Wire.write(0x01);
Wire.endTransmission();
Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
isBusy = Wire.read();
isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit

loopCount++; // Increment loop counter
// Stop status register polling if stuck in loop
if(loopCount > 9999)
{
  return -1;
}
}

// read register 0x0e which is signal strength
Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
Wire.write(0x0e);
Wire.endTransmission();
Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
intensity = Wire.read();

// Return the measured intensity

return intensity;
}
