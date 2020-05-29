#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>

VL53L1X sensor[4];
int sensor_address[4] = {40, 45, 50, 55};
int XSHUT[4] = {9,10,11,12};
int MIN_DIST[4] = {50,50,50,50};
int MAX_DIST[4] = {500,500,500,500};
int range[4];
int range_status[4];
bool detect[4] = {false, false, false, false};

Servo servo;
const byte SERVO_DELAY = 30;
int servoCurrentPos = 0;
int servoDesiredPos = 0;
int servoPin = 7;
int servoPositions[3] = {0, 90, 180}; 

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  for(int i=0; i<3; i++) {
    pinMode(XSHUT[i], OUTPUT);
  }

  for(int i=0; i<3; i++) {
    
    sensor[i].setTimeout(500);
  
    // Toggle XSHUT to reset the sensor    
    pinMode(XSHUT[i], INPUT);
    
    if (!sensor[i].init())
    {
      Serial.println("Failed to detect and initialize sensor!");
      while (1);
    }
    
    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    sensor[i].setDistanceMode(VL53L1X::Long);
    sensor[i].setMeasurementTimingBudget(50000);
  
    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    sensor[i].startContinuous(50);
    sensor[i].setAddress(sensor_address[i]);
  }

  servo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  moveServo(servoCurrentPos, servoDesiredPos);
}

void loop()
{
  // Read sensors
  for(int i=0; i<3; i++) {
    
    sensor[i].read();  
    range[i] = sensor[i].ranging_data.range_mm;
    range_status[i] = sensor[i].ranging_data.range_status; // 0: valid measurement

    // Person detection
    if(range[i] >= MIN_DIST[i] && range[i] <= MAX_DIST[i] && range_status[i] == 0)
      detect[i] = true;
    else
      detect[i] = false;
  }

  // Set desired position if only one sensor is active
  if(detect[0] && !detect[1] && !detect[2])
    servoDesiredPos = servoPositions[0];   
  else if(!detect[0] && detect[1] && !detect[2])
    servoDesiredPos = servoPositions[1];         
  else if(!detect[0] && !detect[1] && detect[2])
    servoDesiredPos = servoPositions[2];

  // Move servo if desired position changes
  if(servoCurrentPos != servoDesiredPos) {
      moveServo(servoCurrentPos, servoDesiredPos);    
      servoCurrentPos = servoDesiredPos;
  }
}

/***************************************************************************************
**                          Control servo
***************************************************************************************/
// Drive from current to desired position with specified speed
void moveServo(int posInit, int posDes) {

  if (posInit > posDes) {
    for (int i = posInit; i >= posDes; i--) {
      servo.write(i); 
      delay(SERVO_DELAY);
    }
  }
  else {
    for (int i = posInit; i <= posDes; i++) {
      servo.write(i);
      delay(SERVO_DELAY);
    }
  }
}

/***************************************************************************************
**                          Print sensor status
***************************************************************************************/
void sensorStatus(byte i) {
  
    Serial.print("This is sensor ");
    Serial.println(i+1);
    Serial.print(sensor[i].getAddress());
    Serial.print("\trange: ");
    Serial.print(range[i]);
    Serial.print("\tstatus: ");
    Serial.print(VL53L1X::rangeStatusToString(sensor[i].ranging_data.range_status));
    Serial.print(range_status[i]); // 0: In ordnung
    Serial.print("\tpeak signal: ");
    Serial.print(sensor[i].ranging_data.peak_signal_count_rate_MCPS);
    Serial.print("\tambient: ");
    Serial.print(sensor[i].ranging_data.ambient_count_rate_MCPS);
    Serial.println();
}
