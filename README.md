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
- PID control <a name="PID"></a>
  $$ u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) \, d\tau + K_d \frac{d}{dt}e(t) $$
