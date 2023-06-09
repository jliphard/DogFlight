# DogFlight

PixRacer Firmware for High Power Rockets with Actuator Dual Deploy and AHRS 

* [Motivation](#motivation)
* [Is writing your own firmware for amateur high power rockets a good idea?](#is-writing-your-own-firmware-for-amateur-high-power-rockets-a-good-idea-)
* [PixHawk and PixRacer drone controllers as rocket flight controllers](#pixhawk-and-pixracer-drone-controllers-as-rocket-flight-controllers)
* [Arduino; Programming the PixHawk/PixRacer](#arduino--programming-the-pixhawk-pixracer)
* [AHRS (attitude and heading reference system)](#ahrs--attitude-and-heading-reference-system-)
* [AHRS calibration](#ahrs-calibration)
* [Dual Deploy Hardware - Linear Actuators](#dual-deploy-hardware---linear-actuators)
* [Dual Deploy Software Settings](#dual-deploy-software-settings)
  
## Motivation

Unlike the amateur rocket community, the drone community has developed sophisticated open source hardware and software. Examples include the [PixHawk](https://pixhawk.org) open standard for drone hardware (in particular, the FMU, flight management unit) and the [Ardupilot](https://ardupilot.org) open source autopilot system supporting multi-copters, submarines, and rovers. In amateur rocketry, however, there is currently a lack of open standards for hardware, especially flight computers for dual deploy and thrust vector control, and open source software that runs on that hardware. 

**Why the strange name?** The firmware was developed for a Journey 75 rocket named "Beware of Dog" built for L1 certification.

## Is writing your own firmware for amateur high power rockets a good idea?

For many dual deploy situations, you will be better off using standard hardware/software with real-world soak time, such as an [Eggtimer Quantum](http://eggtimerrocketry.com/eggtimer-quantum-support/) with ["official" firmware](http://eggtimerrocketry.com/wp-content/uploads/2022/07/Quantum_1_09c.zip). 

However, if your focus is on STEM, education, and creating new things, then at some point you will run up against limitations imposed by closed hardware/software. As I was building a rocket with my kids, I wanted to avoid the complexity of pyro deploy charges and instead build a deployment mechanism based on springs or linear actuators. My main goal was to develop a deployment system kids can understand (even at the middle school level), play with, and actively help to design and build. A simple control logic for rockets can be used to motivate being good at math; help to explain forces and accelerations; and teach basic aspects of using sensors to reliably make sense of the world. 

So, if you care mostly about reliability and are using traditional pyro dual deploy, this firmware is not what you want. However, if you are working with students at any level, including at the university, this firmware could be a good first step for you to review and build on. Indeed, students at the [Stanford Student Space Initiative (SSI)](https://ssi-wiki.stanford.edu/Main_Page) expressed great interest in open source flight control software. 

## PixHawk and PixRacer drone controllers as rocket flight controllers

Fortuitously, it is rather straightforward to use drone autopilots (such as the [PixHawk Mini](https://docs.px4.io/main/en/flight_controller/pixhawk4_mini.html) or the [PixRacer](https://docs.px4.io/main/en/flight_controller/pixracer.html)) to control rockets. A typical drone autopilot features a compact design, a multitude of sensors (triple IMUs, dual magnetometers, altimeters), servo controllers, SD cards, and standard plugs for GPS sensors. Some systems, such as the PixRacer, also can run WiFi servers on an ESP8266 daughter board. Moreover, these boards are fully documented, available off the shelf (even on Amazon), and use powerful processors such as the [STM32H753 ARM Cortex M7](https://www.st.com/en/microcontrollers-microprocessors/stm32h743-753.html) that can handle full/complex AHRS (attitude and heading reference system) math. With the right software, drone autopilots can also handle Thrust Vector Control and interface with optical flow sensors, for precision lateral velocity data prior to touchdown. 

## Arduino; Programming the PixHawk/PixRacer

There are many ways to run and deploy code on drone controllers such as using the [PixHawk Debug Port](https://github.com/pixhawk/Pixhawk-Standards/blob/master/DS-009%20Pixhawk%20Connector%20Standard.pdf). One of the easiest ways is to enable DFU mode, which you can do connecting the PixRacer's 3V3 pad to the BOOT pad - they are right next to one another on the board. At the next power cycle, the system will boot in `STM32CubeProgrammer(DFU)` mode and your new firmware can be downloaded directly to it via the [Arduino IDE](https://www.arduino.cc/en/software). After downloading your firmware to the board, the system will soft-reset into normal mode and appear on the USB bus, which then allows you to capture serial data coming from the PixRacer using the Arduino IDE Serial Monitor.

**NOTE:** For flight, remember to disconnect the 3V3 and BOOT pads so the system boots directly into normal operation mode. 

**NOTE:** If you make the `Serial` connection obligatory (e.g. through something like `while(!Serial) {delay()}`, then your board will not boot unless connected to another computer via USB.

**NOTE:** Using the Arduino IDE with STM32 controllers can be finicky and you will almost certainly need to install various other pieces of software such as the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html). A good place to start is the [documentation for the Arduino STM32 core](https://github.com/stm32duino/Arduino_Core_STM32/wiki/Getting-Started).

## AHRS (attitude and heading reference system)

I added a simple AHRS system that gives the rocket a sense of where it's pointing. This code is not tested / reliable and should be used with great trepidation. Notably, any AHRS solution depends on:

* High performance acceleration, gyro, and magnetometer inputs. **Many IMU chips perform poorly and are not suitable for good AHRS data regardless of what else you do.**
* Precision calibration of all of those inputs within the actual flight vehicle and flight environment. **AHRS will fail if inputs are not correctly calibrated and zeroed.** 
* Correct choice of algorithm
* Correct choice of algorithm sensitivity/convergence parameters 
* Reliable and **regular** polling of the sensors 
* Correct choice of sensor settings e.g. digital band pass filters

Summarizing the above, at this point **AHRS data should not be used for anything you actually care about**, e.g. apogee detection. You are much better off just using the total smoothed acceleration, which will dip < 1G as the rocket coasts to, and reaches, apogee. Note that in real flights, the total acceleration will never reach zero, due to multiple factors such as wind. 

**NOTE:** Although this is obvious from the code, just to be clear the provided AHRS code does not provide any sort of guidance - it just allows the rocket to know which way is "up" and its roll rate. 

## AHRS calibration

Please run `/MagCal/MagCal.ino`. This will collect data from the MPU9250 IMU and stream it over serial. You can feed these data into the excellent [`MotionCal`](https://github.com/PaulStoffregen/MotionCal) software, which will yield hard iron and soft iron calibration/offset factors such as: 

```c++
// These are specific to each IMU and IMU environment - 
// Poor calibration values will prevent AHRS function
float mag_hardiron[]  = { 6.41, 31.07, -12.36 }; // in uTesla
float mag_softiron[]  = { 0.990, 0.007, -0.005, 0.007, 0.990, 0.010, -0.005, 0.010, 1.020 }; 
float gyro_zerorate[] = { 0.01, -0.02, -0.01 }; // in Radians/s
```

## Dual Deploy Hardware - Linear Actuators

The core of the actuator based dual deploy system are two linear actuators, the [Actuonix L12-100-100-6-R](https://www.actuonix.com/l12-100-210-6-r). When triggered, these actuators can exert up to 42N (4kg) force, which can be used to separate the main body tube (and the nosecone) from the dual deploy bay. One drawback of these actuators is that the full travel range of 100mm takes 9 seconds to traverse, which needs to be accounted for in the deployment logic.

**NOTE**: The product documentation states a control range of 1000ms to 2000ms for min to max travel, but in reality most of these actuators seem to take 1050ms to 2050ms as their control range. This is an issue because if you try to drive them with a 1000ms pulse width, they will stall trying to compact past their mechanical travel limits causing current spikes on the servo bus. 

## Dual Deploy Software Settings

These are self explanatory - please see `/DogFlight/DogFlight.ino`. You will need to change them based on your launch site and expected flight and recovery profile. 

```c++

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

```
