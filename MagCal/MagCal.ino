/*
  Dual Deploy Rocket Control on PixRacer r14 drone controller
  Magnetometer Calibration

  Jan Liphardt
  (JTLiphardt@gmail.com)

  No warranties of any kind - use for whatever you want
*/

#define MPU9250_CS PC2 // IMU Chip Select
#include "mpu9250.h"
bfs::Mpu9250 imu(&SPI, MPU9250_CS);

#define MPS2_TO_G 9.80665
#define RAD_TO_DEG 57.2958

int IMU_LAST_READ = 0;

void setup() {

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

}

void loop() {

  delay(20); // main loop runs at 50Hz

  if (imu.Read()) {
    int time_now_ms = millis();
    float imu_dt_s = float(time_now_ms - IMU_LAST_READ) / 1000;

    float ax_G = imu.accel_x_mps2() / MPS2_TO_G;
    float ay_G = imu.accel_y_mps2() / MPS2_TO_G;
    float az_G = imu.accel_z_mps2() / MPS2_TO_G;
    
    float gx = imu.gyro_x_radps();
    float gy = imu.gyro_y_radps();
    float gz = imu.gyro_z_radps();
        
    float mx = imu.mag_x_ut();
    float my = imu.mag_y_ut();
    float mz = imu.mag_z_ut();
    
    IMU_LAST_READ = time_now_ms;

    // 'Raw' values to match expectation of MotionCal
    // MotionCal expects units of Gs, DPS, and uT
    /* Example:
      Raw: 36, -72, 8412, -8, 0, -3, -415, -310, 687
      Raw: 52, -72, 8420, -6, 0, -1, -424, -325, 673
      the third number is so big because that's gravity....
    */
    Serial.print("Raw:");
    Serial.print(int(ax_G*10000)); Serial.print(",");
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
    Serial.print(mx); Serial.print(",");
    Serial.print(my); Serial.print(",");
    Serial.print(mz); Serial.println("");
  }
  
}