# Flight controller for Mbed quadcopter
This is part of ECE 4180 Team Project.
Our team consists of Geehoon Jung, Mindy Yao, Jooyeon Lee, Nikita Gulati .

## PROJECT OVERVIEW
Drones have a wide range of applications in agriculture, military and defense, and transportation. While commercial drones uses pre-existing flighth controller modules like speedybee, our project works towards constructing a quadcopter with four motors piloted by ARM mbed LPC1786 in integration with an IMU moduel. Due to limited time we have for the project and unforseenability in US delivery speed, we broke down the project in small chunks and only move forward when the prior step is accomplished.  



## Harware List
- Mbed LPC1768
- ICM-20948
- A2212 brushless motor
- 30A ESC

## Schematic Design

## PID control
$$ u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) \, d\tau + K_d \frac{d}{dt}e(t) $$
