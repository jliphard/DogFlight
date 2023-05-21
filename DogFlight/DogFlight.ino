/*
  Dual Deploy Rocket Control on PixRacer r14 drone controller

  Jan Liphardt
  (JTLiphardt@gmail.com)

  No warranties of any kind - use for whatever you want
*/

// ****************************************
// FLIGHT CONFIGURATION
// NOTE: MEL = mission elapsed time
// MEL starts from zero at launch detection
int failsafe_MEL_deploy_drouge_ms =  22000;
int failsafe_MEL_deploy_main_ms   = 127000;
int post_Burnout_Lockout_ms       =   3000;

// Everything should be deployed at this AGL
// Uses altimeter data
float deploy_everything_AGL_m = 400.0;

// Accelerometer data
// For launch detect and primary apogee detect
float launch_threshold_accel_g = 5.0;
float apogee_threshold_accel_g = 1.0;
// these values refer to the "total" acceleration,
// which is always positive
// total_A = SQRT(ax^2+ay^2+az^2)

// Altimeter noise
// Used for secondary apogee detection
float apogee_threshold_noise = 0.2;

// ****************************************
// Hardware configuration

// LEDs
#define LED_RED   PB11
#define LED_GREEN PB1
#define LED_BLUE  PB3 

// PRESSURE SENSOR
#define MS5611_CS   PD7
#define MS5611_CLK  PB10
#define MS5611_MISO PB14
#define MS5611_MOSI PB15

// PWM SERVOS
#define SERVO_MAIN_PIN   PE13
#define SERVO_DROUGE_PIN PE14

// Battery Voltage Sense
#define BATT_VOLT_SENS PA2
#define VDD_5V_SENS    PA4

// IMU Chip Select
#define MPU9250_CS PC2

// ****************************************
// DRIVERS and OBJECTS

// PRESSURE
#include "MS5611_SPI.h"
MS5611_SPI MS5611_SPI(MS5611_CS, MS5611_MOSI, MS5611_MISO, MS5611_CLK);
uint32_t start, stop, masterCount, mainLoopTimeLast, mainLoopTimeDiff;

// PWM OUTPUTS
#include <Servo.h>
Servo drougeServo;
Servo mainServo;

#define ACTUATOR_SHORT 0      // 0 mm extension 
#define ACTUATOR_EXTENDED 180 // 100 mm extension

// SD Card on the SDIO bus
#include <STM32SD.h>
File dataFile;

// IMU
#include "mpu9250.h"
bfs::Mpu9250 imu(&SPI, MPU9250_CS);

// These are specific to each IMU and IMU encironment - 
// Poor calibration values will prevent AHRS function
float mag_hardiron[]  = { 6.41, 31.07, -12.36 }; // in uTesla
float mag_softiron[]  = { 0.990, 0.007, -0.005, 0.007, 0.990, 0.010, -0.005, 0.010, 1.020 }; 
float gyro_zerorate[] = { 0.01, -0.02, -0.01 }; // in Radians/s

#include "quaternionFilters.h"
#define MPS2_TO_G 9.80665
#define RAD_TO_DEG 57.2958

// Control/state variables
unsigned long missionStart_ms = 0;
unsigned long missionBurnout_ms = 0;
unsigned long missionElapsedTime_ms = 0;
unsigned long timeSinceBurnout_ms = 0;

enum FlightState: uint8_t {
  ON_PAD = 0,
  MOTOR_BURNING = 1,
  COASTING = 2,
  DESCENT = 3
};
enum FlightState state;

// hardware errors
bool altimeter_failed = false;

bool boot_failed = false;
String master_fail_reason = "";

float battery_V = 0.0;
float VDD_Bus_V = 0.0;

float temperature_C = 0; // Temperature from Altimeter

// Altimeter/Pressure data
float pressure_raw_mbar     = 0;
float pressure_200msRA_mbar = 0;
float pressure_pad_mbar     = 0;
float pressure_noise        = 0;
float AGL_m                 = 0;

// Accelerometer data
float acceleration_Total_200msRA_g = 0.0;
float acceleration_RollAxis_400msRA_g = 0.0;
int IMU_LAST_READ = 0;

// Averaging datastructures
#include "RunningAverage.h"
RunningAverage pressure_10_mbar(10);
RunningAverage acceleration_10_g(10);
RunningAverage acceleration_RollAxis_20_g(20);

// Given a pressure measurement (mbar) and the pressure at a baseline (mbar),
// return altitude (in meters) for the delta in pressures.
double pressureToAGL_m(double currentPressure_mbar, double baselinePressure_mbar)
{
  return(44330.0*(1-pow(currentPressure_mbar/baselinePressure_mbar,1/5.255)));
}

void getVoltages(float &battery_V, float &VDD_Bus_V)
{
  // Battery voltage
  int battery_SENS = analogRead(BATT_VOLT_SENS);
  // NOTE - crazily - the Arduino STM32 core maps the 
  // STM32 12 bit ADC readings onto a 10 bit range (0-1023) 
  // battery_V = battery_SENS / 1023.0 * 3.3 * 15.3 /*Voltage Divider*/ / 1.105 /*= empirical calibration factor*/; 
  // Math: 1 / 1023.0 * 3.3 * 15.3 / 1.105 = 0.0446
  battery_V = float(battery_SENS) * 0.0446;
  //Serial.print("Battery Voltage (V): ");
  //Serial.println(battery_V);

  // 5V Bus Voltage
  int bus_5V_SENS = analogRead(VDD_5V_SENS);
  // NOTE: There is voltage divider (10kOhm and 10kOhm)
  // VDD_Bus_V = (bus_5V_SENS / 1023.0) * 3.3 * (10.0 + 10.0) / 10.0;
  //  Math: 1 / 1023.0 * 3.3 * (10.0 + 10.0) / 10.0 = 0.00645
  VDD_Bus_V = float(bus_5V_SENS) * 0.00645;
  //Serial.print("5V Bus Voltage (V): ");
  //Serial.println(VDD_Bus_V);
}

void setup() {

  // initialize LED control pins
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // turn off all LEDS
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);

  // Enable 3V3 Sensors via PE3 Pin 2 - sigh - this only took a day to figure out
  pinMode(PE3, OUTPUT);
  digitalWrite(PE3, HIGH); 

  /* Serial to display data */
  Serial.begin(115200);

  /* Start the SPI bus */
  SPI.begin();

  /* Initialize and configure IMU */
  int tryCount = 0;
  while (!imu.Begin()) {
    // The IMU does not like to wake up on Monday mornings... 
    // Try a few times...
    Serial.println("Error initializing communication with IMU... Retrying");
    tryCount++;
    delay(100);
    if(tryCount > 10) {
      break;
    }
  }
  /* Set the accel range to 16G by default */
  /* Set the gyro range to 2000DPS by default*/
  /* Set the DLPF to 184HZ by default */
  /* Set the SRD to 0 by default */
  /* Set the IMU sample rate divider */
  /* Set sample rate divider for 50 Hz */
  /* rate_Hz = 1000 / (srd + 1) */
  /* srd setting of 0 means the MPU-9250 samples the accelerometer and gyro at 1000 Hz */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configuring SRD");
    while(1) {}
  }

  if (!imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_16G)) {
    Serial.println("Error configuring Accel Range");
    while(1) {}
  }

  if (MS5611_SPI.begin() == true)
  {
    Serial.print("MS5611 found: ");
    Serial.println(MS5611_SPI.getDeviceID(), HEX);
    digitalWrite(LED_GREEN, LOW); // Green on
  }
  else
  {
    Serial.println("MS5611 not found. Blocking.");
    digitalWrite(LED_RED, LOW); // Red on
    while (1);
  }

  // Timers and mission time and state
  missionStart_ms = 0;
  missionElapsedTime_ms = 0;
  state = ON_PAD;
  masterCount = 0; // integer watchdog counter

  // Clear averaging arrays
  pressure_10_mbar.clear();
  acceleration_10_g.clear();
  acceleration_RollAxis_20_g.clear();

  // collect pressure data for 2 seconds
  for(int i=0; i<40; i++){
    // yup, this will discard the first 30 readings - which is what we want
    int result = MS5611_SPI.read(12);
    pressure_10_mbar.addValue(MS5611_SPI.getPressure());
    delay(50);
  }

  pressure_pad_mbar = pressure_10_mbar.getAverage();
  Serial.println("Pad Pressure (mbar): ");
  Serial.println(pressure_pad_mbar);

  if(pressure_10_mbar.getStandardDeviation() < 0.001) {
    boot_failed = true;
    altimeter_failed = true;
    Serial.println("BOOT Failed: Pressure Too Quiet");
    master_fail_reason += "padpPressureTooQuiet;";
  }

  if(pressure_10_mbar.getStandardDeviation() > 0.1) {
    boot_failed = true;
    Serial.println("BOOT Failed: Pad Barometric Pressure Not Stable");
    master_fail_reason += "padPressureNotStable;";
  }

  if(pressure_pad_mbar > 1030.0) {
    boot_failed = true;
    Serial.println("BOOT Failed: Rocket Underground");
    master_fail_reason += "padPressureTooHigh;";
  }

  if(pressure_pad_mbar < 811.0) {
    boot_failed = true;
    Serial.println("BOOT Failed: Launch Site Too High");
    master_fail_reason += "padElevationTooHigh;";
  }

  getVoltages(battery_V, VDD_Bus_V);

  if(battery_V < 7.6) {
    boot_failed = true;
    Serial.println("BOOT Failed: Battery Voltage Too Low");
    master_fail_reason += "batteryVoltageTooLow;";
  }

  if(VDD_Bus_V < 4.5) {
    boot_failed = true;
    Serial.println("BOOT Failed: MCU Supply Rail Voltage Too Low");
    master_fail_reason += "5V_VDD_BUS_VoltageTooLow;";
  }

  // Connect the Drouge and Main Actuators
  /*
    For the Actuonix L12-100-100-6-R, a 1.0 ms pulse commands the controller to fully retract the actuator, 
    and a 2.0 ms pulse signals it to fully extend (i.e. correct settings are 1000 and 2000). 

    This is not true - if you feed them a 1 ms pulse they will current spike at the 
    end of travel as they try to contract further than they actually can...
    So set to 1050 ms. Also the actual upperlimit is 2050...
    Or the MCU timing is slightly off?
  */
  drougeServo.attach(SERVO_DROUGE_PIN, 1050, 2050);
  mainServo.attach(SERVO_MAIN_PIN, 1050, 2050); // need to confirm for main 

  // Init the Actuators - we want them to be 
  // compact (SHORT) so the rocket can be assembled
  drougeServo.write(ACTUATOR_SHORT);
  mainServo.write(ACTUATOR_SHORT);

  Serial.print("Initializing SD card... ");
  while (!SD.begin())
  {
    delay(10);
  }
  delay(100);
  Serial.println("SD Card initialized");

  // open a new file for logging
  char NewLogFile[] = "datafile_000.txt";
  for (uint8_t i = 0; i < 1000; i++) {
    sprintf(NewLogFile, "datafile_%03d.txt", i);
    if (SD.exists(NewLogFile)) {
      continue;
    } else {
      break;
    }
  }

  // open the new file
  dataFile = SD.open(NewLogFile, FILE_WRITE);
  if (dataFile) {
    Serial.print("Logging to ");
    Serial.println(NewLogFile);
  } else {
    // if the file isn't open, pop up an error:
    Serial.print("Error opening ");
    Serial.println(NewLogFile);
  }

  if (dataFile && master_fail_reason != "") {
    dataFile.println(master_fail_reason);
    dataFile.flush(); // use flush to ensure the data are written
    Serial.println(master_fail_reason); // print to the serial port too
  }

}

void loop() {

  if(boot_failed) {
    digitalWrite(LED_RED, LOW); // Red on
    //return;
  } else {
    digitalWrite(LED_GREEN, LOW); // Green on
  }

  delay(20); // main loop runs at 50Hz

  // System Heartbeat
  masterCount++;
  String dataLog = String("DATA:") + String(masterCount);
  String eventLog = String("EVENT:") + String(masterCount);

  // Failsafe 1 - Misson elapsed time > failsafe_MEL_deploy_drouge_ms
  if(missionElapsedTime_ms > failsafe_MEL_deploy_drouge_ms) {
    // whatever else is happening, 
    // pop the drouge after failsafe_MEL_deploy_drouge_ms mission time
    // customize based on your anticipated flight profile
    // this will (probably) "double actuate" the drouge servo but that's fine
    state = DESCENT; // redundant state change to prepare system to pop main
    drougeServo.write(ACTUATOR_EXTENDED);
    eventLog += String(",FS1_DS_EXTENDED");
  }

  // Failsafe 2 - Misson elapsed time > failsafe_MEL_deploy_main_ms
  if(missionElapsedTime_ms > failsafe_MEL_deploy_main_ms) {
    // whatever else is happening, 
    // pop everything after failsafe_MEL_deploy_main_ms seconds mission time
    // customize based on your anticipated flight profile
    // this will probably "double actuate" the drouge and main servos but that's fine
    drougeServo.write(ACTUATOR_EXTENDED);
    mainServo.write(ACTUATOR_EXTENDED); 
    eventLog += String(",FS2_DS_MS_EXTENDED");
  }

  // Failsafe 3 - We launched AND accleration is small AND we are close to the ground
  if(missionElapsedTime_ms > 0 && 
      acceleration_Total_200msRA_g < apogee_threshold_accel_g && 
      AGL_m < deploy_everything_AGL_m) {
    // pop everything
    // this will (probably) "double actuate" the drouge and main servos but that's fine
    drougeServo.write(ACTUATOR_EXTENDED); 
    mainServo.write(ACTUATOR_EXTENDED); 
    eventLog += String(",FS3_DS_MS_EXTENDED");
  }

  // Measure and Record Voltages
  getVoltages(battery_V, VDD_Bus_V);
  dataLog += String(",VBATT:") + String(battery_V);
  dataLog += String(",VDD:") + String(VDD_Bus_V);

  // Timing checks
  start = micros();
  mainLoopTimeDiff = start - mainLoopTimeLast;
  mainLoopTimeLast = start;
  dataLog += String(",T:") + String(start/1000);

  // Altimeter; Pressure, Noise, and AGL math
  int result = MS5611_SPI.read(12); 

  temperature_C = MS5611_SPI.getTemperature();
  pressure_raw_mbar = MS5611_SPI.getPressure();
  pressure_10_mbar.addValue(pressure_raw_mbar);

  pressure_200msRA_mbar = pressure_10_mbar.getAverage();
  pressure_noise = pressure_10_mbar.getStandardDeviation();

  AGL_m = pressureToAGL_m(pressure_200msRA_mbar, pressure_pad_mbar);
  
  dataLog += String(",TC_ALT:") + String(temperature_C) + String(",P:") + String(pressure_raw_mbar);
  dataLog += String(",AGL:") + String(AGL_m) + String(",NOISE:") + String(pressure_noise);
  
  // IMU
  if (imu.Read()) {
    // NOTE: Below comment copied from MPU9250 Basic Example Code written by Kris Winer
    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientation mismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    int time_now_ms = millis();
    float imu_dt_s = float(time_now_ms - IMU_LAST_READ) / 1000;
    IMU_LAST_READ = time_now_ms;
    Serial.print("DT(s):");
    Serial.println(imu_dt_s);

    float ax_G = imu.accel_x_mps2() / MPS2_TO_G;
    float ay_G = imu.accel_y_mps2() / MPS2_TO_G;
    float az_G = imu.accel_z_mps2() / MPS2_TO_G;
    
    // this will differ from rocket to rocket
    // specify roll axis here and check signs
    acceleration_RollAxis_20_g.addValue(ax_G);
    acceleration_RollAxis_400msRA_g = acceleration_RollAxis_20_g.getAverage();

    float totalAccel = sqrt(ax_G*ax_G + ay_G*ay_G + az_G*az_G);
    acceleration_10_g.addValue(totalAccel);
    acceleration_Total_200msRA_g = acceleration_10_g.getAverage();

    float gx = imu.gyro_x_radps() + gyro_zerorate[0];
    float gy = imu.gyro_y_radps() + gyro_zerorate[1];
    float gz = imu.gyro_z_radps() + gyro_zerorate[2];
        
    // hard iron cal
    float mx = imu.mag_x_ut() - mag_hardiron[0];
    float my = imu.mag_y_ut() - mag_hardiron[1];
    float mz = imu.mag_z_ut() - mag_hardiron[2];
    
    // soft iron cal
    float mxCal = mx * mag_softiron[0] + my * mag_softiron[1] + mz * mag_softiron[2];
    float myCal = mx * mag_softiron[3] + my * mag_softiron[4] + mz * mag_softiron[5];
    float mzCal = mx * mag_softiron[6] + my * mag_softiron[7] + mz * mag_softiron[8];

    MahonyQuaternionUpdate(ax_G, ay_G, az_G, gx, gy, gz, mxCal, myCal, mzCal, imu_dt_s);

    // Serial.print("q0 = ");  Serial.print(*getQ());
    // Serial.print(" qx = "); Serial.print(*(getQ() + 1));
    // Serial.print(" qy = "); Serial.print(*(getQ() + 2));
    // Serial.print(" qz = "); Serial.println(*(getQ() + 3));

    dataLog += String(",AX:") + String(ax_G) + String(",AY:") + String(ay_G) + String(",AZ:") + String(az_G);
    dataLog += String(",GX:") + String(gx) + String(",GY:") + String(gy) + String(",GZ:") + String(gz);
    dataLog += String(",MX:") + String(mxCal) + String(",MY:") + String(myCal) + String(",MZ:") + String(mzCal);
    dataLog += String(",TC_IMU:") + String(imu.die_temp_c());
  
    // NOTE: Below comment copied from MPU9250 Basic Example Code written by Kris Winer
    // Define output variables from updated quaternion---these are Tait-Bryan
    // angles, commonly used in aircraft orientation. In this coordinate system,
    // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
    // x-axis and Earth magnetic North (or true North if corrected for local
    // declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    float myIMUyaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                  * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                  * *(getQ()+3));
    float myIMUpitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                  * *(getQ()+2)));
    float myIMUroll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                  * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                  * *(getQ()+3));
  
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(myIMUyaw *= RAD_TO_DEG, 2);
    Serial.print(", ");
    Serial.print(myIMUpitch *= RAD_TO_DEG, 2);
    Serial.print(", ");
    Serial.println(myIMUroll *= RAD_TO_DEG, 2);
  }

  if(pressure_noise < 0.001) {
    // The sensor is frozen or has failed
    // A working sensor will be noisy - a typical value for the SD is 
    // 0.01 or 0.02
    altimeter_failed = true;
    eventLog += String(",ERROR:PRESSURE_FROZEN");
  } 

  // On the pad, the pressure will be very stable
  // although it might drift slightly due to weather 
  // changes or brutal sun
  // The acceleration will also be very stable except 
  // for transients when people close access doors etc.
  // or wind
  if(state == ON_PAD) {
    if(acceleration_RollAxis_400msRA_g <= launch_threshold_accel_g) {
      // we are on the pad and life is boring...
      eventLog += String(",STATUS:ON_PAD");
    } else if (acceleration_RollAxis_400msRA_g > launch_threshold_accel_g) {
      // we just took off
      // this value should be tuned to your sensor and anticipated/measured
      // signal during early ascent
      // Will fail for very slow ascent rates 
      state = MOTOR_BURNING;
      missionStart_ms = millis();
      eventLog += String(",EVENT:LAUNCH");
    }
  }

  if(state != ON_PAD) {
    missionElapsedTime_ms = millis() - missionStart_ms;
  }
  dataLog += String(",MEL:") + String(missionElapsedTime_ms);

  // We are ascending; motor is burning. Detect motor burnout. 
  // Roll axis g will immediately cross zero and go negative due to 
  // aerodynmic drag opposing forward motion of rocket.
  // Cp is moving towards the base of the rocket and 
  // then flipping e.g. past Mach 1.5 
  // Altimeter readings are unreliable
  if(state == MOTOR_BURNING) {
    if (acceleration_RollAxis_400msRA_g < 0.0) {
      // burnout; roll axis acceleration goes from >> 0 to < 0
      state = COASTING;
      missionBurnout_ms = millis();
      eventLog += String(",STATUS:BURNOUT");     
    }
  }

  if(state == COASTING) {
    timeSinceBurnout_ms = millis() - missionBurnout_ms;
  }
  dataLog += String(",TSB:") + String(timeSinceBurnout_ms);

  // PRIMARY APOGEE DETECTION
  // Use dip in acceleration as rocket coasts to, and reaches,
  // apogee. For a moment the rocket will experience almost zero G.
  if(state == COASTING && timeSinceBurnout_ms > post_Burnout_Lockout_ms) {
    // look for apogee
    if (acceleration_Total_200msRA_g >= apogee_threshold_accel_g) {
      // we are still ascending
      eventLog += String(",STATUS:G_STILL_ASCENDING");     
    } else if (acceleration_Total_200msRA_g < apogee_threshold_accel_g) {
      // Obviously you will need to customize these values to your flight profile
      eventLog += String(",EVENT:G_APOGEE_DROUGE_EXTEND"); 
      state = DESCENT;
      drougeServo.write(ACTUATOR_EXTENDED);
    }
  }

  // SECONDARY APOGEE DETECTION 
  if(state == COASTING && timeSinceBurnout_ms > post_Burnout_Lockout_ms) {
    if (pressure_noise >= apogee_threshold_noise) {
      // We are still ascending 
      // Altimeter continues to be unreliable 
      eventLog += String(",STATUS:P_STILL_ASCENDING");     
    } else if (pressure_noise < apogee_threshold_noise) {
      // We are either extremely high up or moving slowly as we approach apogee
      // At this point the altimeter is reliable
      // Customize these values to your flight profile
      eventLog += String(",EVENT:P_APOGEE_DROUGE_EXTEND"); 
      state = DESCENT;
      drougeServo.write(ACTUATOR_EXTENDED);
    }
  }

  if(state == DESCENT) {
    // the drouge should be out and we should be descending
    // goal now is to pop the main chute at deploy_everything_AGL_m 
    // (e.g. 200m)
    if (AGL_m < deploy_everything_AGL_m) {
      eventLog += String(",EVENT:DESCENDING_MAIN_EXTEND");  
      mainServo.write(ACTUATOR_EXTENDED);
    } 
  }

  if (dataFile) {
    dataFile.println(dataLog);
    dataFile.println(eventLog);
    dataFile.flush();        // use flush to ensure the data are written
    Serial.println(dataLog); // print to the serial port too
    Serial.println(eventLog);
  }

  digitalWrite(LED_GREEN, HIGH);
  
}