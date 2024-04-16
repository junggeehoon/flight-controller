# Flight controller for Mbed quadcopter
This documentation is for our team project in the Embedded Systems (ECE 4180). Our group includes Geehoon Jung, Mindy Yao, Jooyeon Lee, and Nikita Gulati.

# Project Overview
Drones have a wide range of applications in agriculture, military and defense, and transportation. While commercial drones use pre-existing flight controller modules like speedybee, our project works towards constructing a quadcopter with four motors piloted by ARM mbed LPC1768 in integration with an IMU module. Due to limited time we have for the project, we broke down the project into small chunks focusing PID controller system to balance the drone automatically.

# Table of Contents
1. [Hardware](#hardware)
2. [Schematic Diagram](#schematic)
3. [First Step: Measuring accurate angle with IMU](#step1)
    - [Accelerometer](#accel)
    - [Gyroscope](#gyro)
    - [Complementary Filter](#FILTER)
5. [Second Step: Controlling Brushless Motors with ESC’s and PWM](#step2)
6. [Third Step: Stabilizing Seesaw with PID Controllers](#step3)


# Hardware <a name="hardware"></a>
- Mbed LPC1768
- ICM-20948
- A2212 brushless motor
- 30A ESC
- One-axis PID testing bed

# Schematic Diagram <a name="schematic"></a>
![circuit](https://github.com/junggeehoon/flight-controller/assets/23613481/04a26715-009d-446a-9a19-c7e88912c7b0)


# First Step: Measuring accurate angle with IMU <a name="step1"></a>
The first goal of our project is to use two motors and IMU to create a seesaw-looking testing bed for PID control on only y axis. This step aims to get teammembers familiarize with the PID control algorithm, test all the existing hardware componenets, and get IMU functional reading the pitch angle. 

## Accelerometer <a name="accel"></a>
The given example code in class utilizes readings from magnometer, gyroscope, and accelerator to compute roll, pitch, yaw angles, causing a major drawback in the considerable amount of time (>3min) used in just calibrating for the IMU despite its relative accuracy. Knowing the magnometer is the most time-consuming component in calibration, our solution used only the gyroscope and accelerometer readings for pitch calulation.

Accelerometer alone: $\theta_A = \arctan(\frac{ax}{\sqrt{ay^2+az^2}})$
Gyrscope alone: $\theta_G = \theta_G + \omega \cdot dt$

## Gyroscope <a name="gyro"></a>
Accelerometer has unstable reading but it demonstrates minimal long-term drift, maintaining a relatively stable baseline. In contrast, the gyroscope provides highly accurate measurements that are consistent and precise in the short term, but it is susceptible to gradual drift, accumulating errors over extended periods of time.

## Complementary Filter  <a name="filter"></a>
A complementary filter is used to combine the feature of both and optimize the pitch reading, thus leading to: 
$\theta = 0.98*\theta_G + 0.02*\theta_A$

# Second Step: Controlling Brushless Motors with ESC’s and PWM<a name="step2"></a>


# Third Step: Stabilizing Seesaw with PID Controllers<a name="step3"></a>
## PID control <a name="PID"></a>

  $u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) \, d\tau + K_d \frac{d}{dt}e(t)$
