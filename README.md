# Flight controller for Mbed quadcopter
This is part of ECE 4180 Team Project.
Our team is consisted of Geehoon Jung, Mindy Yao, Jooyeon Lee, Gulati Nikita.

## PROJECT OVERVIEW

## Harware List
- Mbed LPC1768
- ICM-20948
- A2212 brushless motor
- 30A ESC

## Schematic Design

## PID control
$$ u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) \, d\tau + K_d \frac{d}{dt}e(t) $$
