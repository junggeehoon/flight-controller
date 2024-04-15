# Flight controller for Mbed quadcopter
This is part of ECE 4180 Team Project.
Our team consists of Geehoon Jung, Mindy Yao, Jooyeon Lee, Nikita Gulati .

## PROJECT OVERVIEW
Drones have a wide range of applications in agriculture, military and defense, and transportation. While commercial drones uses pre-existing flighth controller modules like speedybee, our project works towards constructing a quadcopter with four motors piloted by ARM mbed LPC1786 in integration with an IMU moduel. Due to limited time we have for the project and the unforseenability of the US delivery services, we broke down the project into small chunks and only move forward when the prior step is accomplished.  

# Table of Contents
1. [Hardware](#hardware)
2. [First Step: One-axis PID Seesaw](#step1)
    - [IMU](#IMU)
    - [PID Control](#PID)
5. [Third Example](#third-example)
6. [Fourth Example](#fourth-examplehttpwwwfourthexamplecom)


## Hardware List <a name="hardware"></a>
- Mbed LPC1768
- ICM-20948
- A2212 brushless motor
- 30A ESC
- One-axis PID testing bed

## First Step: One-axis PID Seesaw <a name="step1"></a>
The first goal of our project is to use two motors and IMU to create a seesaw-looking testing bed for PID control on only y axis. This step aims to get teammembers familiarize with the PID control algorithm, test all the existing hardware componenets, and get IMU functional reading the pitch angle. 

- IMU <a name="IMU"></a>
The given example code in class utilizes readings from magnometer, gyroscope, and accelerator to compute roll, pitch, yaw angles, causing a major drawback in the considerable amount of time (>3min) used in just calibrating for the IMU despite its relative accuracy. Knowing the magnometer is the most time-consuming component in calibration, our solution used only the gyroscope and accelerometer readings for pitch calulation.

Accelerometer alone: $acc_pitch = \arctan(\frac{ax}{\sqrt{ay^2+az^2}})$
Gyrscope alone: $gyro_pitch = angle+gyro*dt$

Accelerometer has unstable reading but does not drift much, while gyroscope readings are precise yet tend to drift. A complementary filter is used to combine the feature of both and optimize the pitch reading, thus leading to: 
$Pitch = 0.98 *gyro_pitch + 0.02*acc_pitch $




- PID control <a name="PID"></a>
  $$ u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) \, d\tau + K_d \frac{d}{dt}e(t) $$
