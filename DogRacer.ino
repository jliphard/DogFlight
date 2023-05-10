/*
  Dual Deploy Rocket Control on PixRacer r14 drone controller

  Jan Liphardt
  (JTLiphardt@gmail.com)

  No warranties of any kind
*/

// LEDs
#define LED_RED   PB11
#define LED_GREEN PB1
#define LED_BLUE  PB3 

// PRESSURE SENSOR
#include "MS5611_SPI.h"
#define MS5611_CS   PD7
#define MS5611_CLK  PB10
#define MS5611_MISO PB14
#define MS5611_MOSI PB15

MS5611_SPI MS5611_SPI(MS5611_CS, MS5611_MOSI, MS5611_MISO, MS5611_CLK);
uint32_t start, stop, masterCount, mainLoopTimeLast, mainLoopTimeDiff;

#include "RunningAverage.h"
// used for basic pressure averaging
RunningAverage pressure_AVE10_mbar(10);
// used for basic acceleration averaging
RunningAverage acceleration_AVE10_g(10);

// PWM OUTPUTS
#include <Servo.h>
Servo drougeServo;
Servo mainServo;

#define SERVO_DROUGE_PIN PE14
#define SERVO_MAIN_PIN PE13

#define ACTUATOR_SHORT 0      // 0 mm extension 
#define ACTUATOR_EXTENDED 180 // 100 mm extension

// SD Card on the SDIO bus
#include <STM32SD.h>
File dataFile;

// IMU
#include "mpu9250.h"
#define MPU9250_CS PC2 // MPU9250_CS
// Mpu9250 object, SPI bus, CS
bfs::Mpu9250 imu(&SPI, MPU9250_CS);

float mag_hardiron[3]; 
float mag_softiron[9]; 
float gyro_zerorate[3]; 

/*
full scale ±4800μT
±250, ±500, ±1000, and ±2000°/sec (dps)
±2g, ±4g, ±8g, and ±16g
*/

#define MPS2_TO_G 9.80665
#define RAD_TO_DEG 57.2958
#include "quaternionFilters.h"

// Control variables
unsigned long missionStart_ms;
unsigned long missionElapsedTime_ms;

bool status_on_pad = true;
bool status_ascent = false;
bool status_apogee = false;
bool status_main = false;

bool boot_failed = false;
String master_fail_reason = "";

float temperature_C = 0;
float battery_V = 0.0;
float VDD_Bus_V = 0.0;

float pressure_pad_mbar = 0;
float pressure_now_mbar = 0;
float pressure_noisy_mbar = 0;
float pressure_noise = 0;
float AGL_m = 0;

int IMU_LAST_READ = 0;

// FLIGHT CONFIGURATION
float emergency_AGL_deploy_everything_m = 200.0; // meters
// MEL = mission elapsed time
int failsafe_MEL_deploy_drouge_ms = 10000; // 10 seconds
int failsafe_MEL_deploy_main_ms = 15000; // 15 seconds
float launch_threshold_noise = 0.7;
float apogee_threshold_noise = 0.2;

// Given a pressure measurement (mbar) and the pressure at a baseline (mbar),
// return altitude (in meters) for the delta in pressures.
double pressureToAGL_m(double currentPressure_mbar, double baselinePressure_mbar)
{
  return(44330.0*(1-pow(currentPressure_mbar/baselinePressure_mbar,1/5.255)));
}

  enum AccelRange : int8_t {
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x08,
    ACCEL_RANGE_8G = 0x10,
    ACCEL_RANGE_16G = 0x18
  };

// Battery Voltage Sense
#define BATT_VOLT_SENS PA2
#define VDD_5V_SENS PA4

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

// in uTesla
  mag_hardiron[0] =   6.41; // -3.35;
  mag_hardiron[1] =  31.07; // -0.74;
  mag_hardiron[2] = -12.36; //-40.79;

  // in uTesla
  mag_softiron[0] =  0.990; //0.965;
  mag_softiron[1] =  0.007; //0.018;
  mag_softiron[2] = -0.005; //0.010;  
  mag_softiron[3] =  0.007; //0.018;
  mag_softiron[4] =  0.990; //0.960;
  mag_softiron[5] =  0.010; //0.003;  
  mag_softiron[6] = -0.005; //0.010;
  mag_softiron[7] =  0.010; //0.003;
  mag_softiron[8] =  1.020; //1.080;  

  // in Radians/s
  gyro_zerorate[0] =   0.01; // 0.05;
  gyro_zerorate[1] = - 0.02; //-0.01;
  gyro_zerorate[2] = - 0.01; //-0.01;

    // float gx = imu.gyro_x_radps() + 0.01;
    // float gy = imu.gyro_y_radps() - 0.02;
    // float gz = imu.gyro_z_radps() - 0.01;

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
/*
  enum AccelRange : int8_t {
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x08,
    ACCEL_RANGE_8G = 0x10,
    ACCEL_RANGE_16G = 0x18
  };
  */

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

  // Clear main Pressure averaging array
  pressure_AVE10_mbar.clear();

  // collect pressure data for 2 seconds
  for(int i=0; i<40; i++){
    // yup, this will discard the first 30 readings - which is what we want
    int result = MS5611_SPI.read(12);
    pressure_AVE10_mbar.addValue(MS5611_SPI.getPressure());
    delay(50);
  }

  pressure_pad_mbar = pressure_AVE10_mbar.getAverage();
  Serial.println("Pad Pressure (mbar): ");
  Serial.println(pressure_pad_mbar);

  if(pressure_AVE10_mbar.getStandardDeviation() < 0.001) {
    boot_failed = true;
    Serial.println("BOOT Failed: Pressure Too Quiet");
    master_fail_reason += "padpPressureTooQuiet;";
  }

  if(pressure_AVE10_mbar.getStandardDeviation() > 0.1) {
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

  // timers
  missionStart_ms = 0;
  missionElapsedTime_ms = 0;

  getVoltages(battery_V, VDD_Bus_V);

  // if(battery_V < 7.6) {
  //   boot_failed = true;
  //   Serial.println("BOOT Failed: Battery Voltage Too Low");
  //   master_fail_reason += "batteryVoltageTooLow;";
  // }

  if(VDD_Bus_V < 4.5) {
    boot_failed = true;
    Serial.println("BOOT Failed: MCU Supply Rail Voltage Too Low");
    master_fail_reason += "5V_VDD_BUS_VoltageTooLow;";
  }

  masterCount = 0;

  // Connect to the Drouge and Main Actuators
  /*
    For the Actuonix L12-100-100-6-R, a 1.0 ms pulse commands the controller to fully retract the actuator, 
    and a 2.0 ms pulse signals it to fully extend (i.e. correct settings are 1000 and 2000). 

    This is not completely true - if you feed them a 1 ms pulse they will current spike at the 
    end of travel as they try to contract further than they actually can...
    So set to 1050 ms
    Also the actual upperlimit is 2050...
    Or the MCU timing is slightly off?
  */
  drougeServo.attach(SERVO_DROUGE_PIN, 1050, 2050);
  mainServo.attach(SERVO_MAIN_PIN, 1050, 2050); // need to confirm for main 

  // Init the Actuators - we want them to be 
  // compact (SHORT) so the rocket can be assembled
  //drougeServo.write(ACTUATOR_SHORT);

  drougeServo.write(180);

  //mainServo.write(ACTUATOR_SHORT);

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
    dataFile.flush();        // use flush to ensure the data are written
    Serial.println(master_fail_reason); // print to the serial port too
  }

}

void loop() {

  if(boot_failed) {
    digitalWrite(LED_RED, LOW); // Red on
    return;
  }

  delay(20); // main loop runs at 50Hz
  digitalWrite(LED_GREEN, LOW); // ON

  // Integrity counter
  masterCount++;
  String dataLog = String("DATA:") + String(masterCount);
  String eventLog = String("EVENT:") + String(masterCount);

  // Failsafe 1 - Misson elapsed time > failsafe_MEL_deploy_drouge_ms
  if(missionElapsedTime_ms > failsafe_MEL_deploy_drouge_ms) {
    // whatever else is happening, 
    // pop the drouge after failsafe_MEL_deploy_drouge_ms mission time
    // customize based on your anticipated flight profile
    // this will (probably) "double actuate" the drouge servo but that's fine
    status_apogee = true; // redundant state change to prepare system to pop main
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

  // Failsafe 3 - We launched AND the sensor is quiet AND we are close to the ground
  if(missionElapsedTime_ms > 0 && pressure_noise < apogee_threshold_noise && AGL_m < emergency_AGL_deploy_everything_m) {
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
  pressure_noisy_mbar = MS5611_SPI.getPressure();
  pressure_AVE10_mbar.addValue(pressure_noisy_mbar);
  pressure_now_mbar = pressure_AVE10_mbar.getAverage();
  AGL_m = pressureToAGL_m(pressure_now_mbar, pressure_pad_mbar);
  pressure_noise = pressure_AVE10_mbar.getStandardDeviation();

  dataLog += String(",TC_ALT:") + String(temperature_C) + String(",P:") + String(pressure_noisy_mbar);
  dataLog += String(",AGL:") + String(AGL_m) + String(",NOISE:") + String(pressure_noise);
  
  // IMU
  if (imu.Read()) {
    
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
    Serial.print("DT(s):");
    Serial.println(imu_dt_s);
    // TODO: Calibrate/zero/convert the raw accelerometer data
    // This depends on scale being set
    // values seem too big by a factor of 10?
    float ax_G = imu.accel_x_mps2() / MPS2_TO_G;
    float ay_G = imu.accel_y_mps2() / MPS2_TO_G;
    float az_G = imu.accel_z_mps2() / MPS2_TO_G;
    
    // TODO: Calibrate/zero/convert the raw gyro data
    // This depends on scale being set
    float gx = imu.gyro_x_radps() + gyro_zerorate[0];
    float gy = imu.gyro_y_radps() + gyro_zerorate[1];
    float gz = imu.gyro_z_radps() + gyro_zerorate[2];
    
    // TODO: Calibrate/zero/convert the magnetometer values
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    // myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
    //            * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    // myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
    //            * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    // myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
    //            * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
    
    // hard iron cal
    float mx = imu.mag_x_ut() - mag_hardiron[0];
    float my = imu.mag_y_ut() - mag_hardiron[1];
    float mz = imu.mag_z_ut() - mag_hardiron[2];
    
    // soft iron cal
    float mxCal = mx * mag_softiron[0] + my * mag_softiron[1] + mz * mag_softiron[2];
    float myCal = mx * mag_softiron[3] + my * mag_softiron[4] + mz * mag_softiron[5];
    float mzCal = mx * mag_softiron[6] + my * mag_softiron[7] + mz * mag_softiron[8];

    IMU_LAST_READ = time_now_ms;

  // 'Raw' values to match expectation of MotionCal
  // accel_event.acceleration.z*8192/9.8)
  // MotionCal expects units of G's, DPS, and uT
  /*
Raw: 36, -72, 8412, -8, 0, -3, -415, -310, 687
Raw: 52, -72, 8420, -6, 0, -1, - 424, -325, 673

the third number is so big because that's gravity....

*/
  Serial.print("Raw:");
  Serial.print(int(ax_G*10000)); Serial.print(","); // this looks like mG?
  Serial.print(int(ay_G*10000)); Serial.print(",");
  Serial.print(int(az_G*10000)); Serial.print(",");
  Serial.print(int(gx*RAD_TO_DEG)); Serial.print(","); // Deg resolution, that's ok
  Serial.print(int(gy*RAD_TO_DEG)); Serial.print(",");
  Serial.print(int(gz*RAD_TO_DEG)); Serial.print(",");
  Serial.print(int(mx*10)); Serial.print(","); // uT*10
  Serial.print(int(my*10)); Serial.print(",");
  Serial.print(int(mz*10)); Serial.println("");

  // unified data
  Serial.print("Uni:");
  Serial.print(ax_G); Serial.print(",");
  Serial.print(ay_G); Serial.print(",");
  Serial.print(az_G); Serial.print(",");
  Serial.print(gx, 4); Serial.print(",");
  Serial.print(gy, 4); Serial.print(",");
  Serial.print(gz, 4); Serial.print(",");
  Serial.print(mxCal); Serial.print(",");
  Serial.print(myCal); Serial.print(",");
  Serial.print(mzCal); Serial.println("");

    MahonyQuaternionUpdate(ax_G, ay_G, az_G, gx, gy, gz, mxCal, myCal, mzCal, imu_dt_s);

    Serial.print("q0 = ");  Serial.print(*getQ());
    Serial.print(" qx = "); Serial.print(*(getQ() + 1));
    Serial.print(" qy = "); Serial.print(*(getQ() + 2));
    Serial.print(" qz = "); Serial.println(*(getQ() + 3));

    dataLog += String(",AX:") + String(ax_G) + String(",AY:") + String(ay_G) + String(",AZ:") + String(az_G);
    dataLog += String(",GX:") + String(gx) + String(",GY:") + String(gy) + String(",GZ:") + String(gz);
    dataLog += String(",MX:") + String(mxCal) + String(",MY:") + String(myCal) + String(",MZ:") + String(mzCal);
    dataLog += String(",TC_IMU:") + String(imu.die_temp_c());
  
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
  
    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    //myIMUyaw -= 8.5;

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
    // Without information, we default to doing nothing
    // return early; go back to start of loop
    eventLog += String(",ERROR:PRESSURE_FROZEN");
    // Emergency log flush
    if (dataFile) {
      dataFile.println(eventLog);
      dataFile.flush();
    }
    return;
  } else {
    // We have a working pressure sensor... continue
  }

  // On the pad, the pressure will be very stable
  // although it might drift slightly due to weather 
  // changes or brutal sun
  if(status_on_pad) {
    if(pressure_noise <= launch_threshold_noise) {
      // we are on the pad and life is boring...
      eventLog += String(",STATUS:ON_PAD");
    } else if (pressure_noise > launch_threshold_noise) {
      // we just took off
      // this value should be tuned to your sensor and anticipated/measured
      // signal during early ascent
      // NOTE - this calculation will fail for very slow ascent rate (e.g. weather ballons) 
      // and/or very high initial launch pad levels
      // Examples 
      // for a pad at MSL and an initial ascent rate of 200m/s we expect noise > 3
      // for a pad at MSL and an initial ascent rate of 100m/s we expect noise > 1.5
      // at 10,000 ft and initial ascent rate of 20m/s we expect noise of ~0.1
      status_on_pad = false;
      status_ascent = true;
      missionStart_ms = millis();
      eventLog += String(",EVENT:LAUNCH");
    }
  }

  if(!status_on_pad) {
    missionElapsedTime_ms = millis() - missionStart_ms;
  }
  dataLog += String(",MEL:") + String(missionElapsedTime_ms);

  // We are ascending and Mach number is increasing  
  // Cp is moving towards the base of the rocket and then flipping e.g. past Mach 1.5 
  // Altimeter readings are unreliable 
  // Other factors to consider include how the altimeter is mounted 
  // relative to the G loads
  if(status_ascent) {
    if (pressure_noise >= apogee_threshold_noise) {
      // we are still ascending - pressure continues to be 
      // unreliable 
      eventLog += String(",STATUS:ASCENDING");     
    } else if (pressure_noise < apogee_threshold_noise) {
      // we are either extremely high up, or moving slowly as we approach apogee
      // at this point the altimeter is reliable
      // Obviously you will need to customize these values to your flight profile
      eventLog += String(",EVENT:APOGEE_DROUGE_EXTEND"); 
      status_apogee = true;
      drougeServo.write(ACTUATOR_EXTENDED);
    }
  }

  if(status_apogee) {
    // the drouge should be out, and we should be descending
    // goal now is to pop the main chute at emergency_AGL_deploy_everything_m 
    // (e.g. 200m) AGL, for example
    if (AGL_m < emergency_AGL_deploy_everything_m) {
      eventLog += String(",EVENT:DESCENDING_MAIN_EXTEND");  
      mainServo.write(ACTUATOR_EXTENDED);
    } 
  }

  if (dataFile) {
    dataFile.println(dataLog);
    dataFile.println(eventLog);
    dataFile.flush();        // use flush to ensure the data are written
    Serial.println(dataLog); // print to the serial port too
    //Serial.println(eventLog);
  }

  digitalWrite(LED_GREEN, HIGH);
  
}