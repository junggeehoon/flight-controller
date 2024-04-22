# Flight controller for Mbed quadcopter
Documentation of Embedded Systems Design Final Project. Project contributors are Geehoon Jung, Mindy Yao, Jooyeon Lee, Nikita Gulati.

# Project Overview
Drones have a wide range of applications in agriculture, military and defense, and transportation. While commercial drones use pre-existing flight controller modules like speedybee, our project works towards constructing a quadcopter with four motors piloted by ARM mbed LPC1768 in integration with an IMU module. Due to the limited time we have for the project, we broke down the project into small chunks focusing PID controller system to balance the drone automatically.

# Table of Contents
- [Hardware](#hardware)
- [Schematic Diagram](#schematic)
- [First Step: Measuring accurate angle with IMU](#step1)
    - [Accelerometer](#accel)
    - [Gyroscope](#gyro)
    - [Complementary Filter](#FILTER)
- [Second Step: Controlling Brushless Motors with ESC’s and PWM](#step2)
- [Third Step: Stabilizing Seesaw with PID Controllers](#step3)
- [Fourth Step: Bluetooth Angle COntroller](#step4)
- [Results](#results)
- [Conclusion](#conclusion)
- [References](#references)


# Hardware <a name="hardware"></a>
- Mbed LPC1768
- ICM-20948
- Two A2212 brushless motor
- Two 30A ESC
- Adafruit Bluefruit LE UART Friend
- One-axis PID testing bed

# Schematic Diagram <a name="schematic"></a>
![circuit (1)](https://github.com/junggeehoon/flight-controller/assets/23613481/56f3b912-8228-41e1-82d0-ac1d3fcbae46)


# First Step: Measuring accurate angle with IMU <a name="step1"></a>
The first goal of our project is to use two motors and IMU to create a seesaw-looking testing bed for PID control on only the y-axis. A critical initial step in this process involves obtaining a precise and stable measurement of the system's angle, which serves as a fundamental component of our feedback control system. The PID controller begins by reading $\theta$, the current pitch angle, and compares it to a target value. Based on this comparison, adjustments are made using the Proportional, Integral, and Derivative constants to align the actual angle with the desired angle. 


## Accelerometer <a name="accel"></a>
One way to measure the pitch angle is utilizing accelermoter in the IMU which measures linear acceleration in the X, Y, and Z axes. The formula to determine $\theta_A$, the pitch angle from acceleromter reading is:

$\theta_A = \arctan(\frac{a_x}{\sqrt{a_y^2+a_z^2}})$

Here, $a_x, a_y, a_z$ represent the acceleration readings along the X, Y, and Z axes, respectively. This calculation uses the gravitational components of the accelerometer readings to determine how tilted the device is relative to the horizontal plane. 

## Gyroscope <a name="gyro"></a>
Gyroscope measures an object's angular rate with respect to an inertial reference frame. Our IMU, the ICM-20948, outputs three digital signals representing the angular rates for the X, Y, and Z axes. The angle can be calculated using the equation below, where $\omega$ represents the angular rate in degrees per second, $dt$ is the elapsed time, and $\theta_G$ denotes the pitch angle in degrees.

$\theta_G = \theta_G + \omega \cdot dt$


## Complementary Filter  <a name="filter"></a>
The accelerometer has unstable reading but it demonstrates minimal long-term drift, maintaining a relatively stable baseline. In contrast, the gyroscope provides highly accurate measurements that are consistent and precise in the short term, but it is susceptible to gradual drift, accumulating errors over extended periods. To achieve the highest possible level of accuracy, IMU calibration and filtering methods such as low-pass filters, averaging filters, and Kalman filters are widely used. In this project, a complementary filter for sensor fusion was used to balance high accuracy with minimal latency. This approach leads to:

$\theta = 0.98*\theta_G + 0.02*\theta_A$

# Second Step: Controlling Brushless Motors with ESC’s and PWM <a name="step2"></a>

## Introduction to Brushless Motors

Brushless DC (BLDC) motors are increasingly favored in a variety of applications due to their high efficiency, reliability, and superior performance over brushed motor counterparts. These motors operate on the fundamental principles of electromagnetism through the interaction of magnetic fields and electric currents. The motor comprises two main parts: the rotor, which is the rotating part equipped with magnets, and the stator, which remains stationary and houses coils of wire. As the electric current flows through these coils, it creates a magnetic field that interacts with the magnets on the rotor, causing it to turn.

## Role of the Electronic Speed Controller (ESC)

In systems requiring precise control of brushless motors, the Electronic Speed Controller (ESC) is pivotal. The ESC serves as an intermediary between the microcontroller and the motor itself. It interprets control signals from the microcontroller, usually in the form of pulse width modulation (PWM), and translates these into three-phase AC power suitable for driving the motor. This capability allows the ESC to adjust the speed and torque of the motor dynamically, based on real-time inputs from the control system.

## PWM Signals

Pulse Width Modulation (PWM) is a crucial technique in electronic control systems used to regulate the amount of power delivered to an electrical device. The technique involves varying the width of the voltage pulses in a digital signal, thereby controlling the average voltage and power supplied to the device. In motor control, PWM enables precise adjustments to the speed of a brushless motor. The frequency of the PWM signal dictates how fast the pulses are sent, while the duty cycle (expressed as a percentage) determines what fraction of each pulse period is high voltage. By adjusting these parameters, the microcontroller can finely tune the motor's speed and torque.

## Practical Implementation of Motor Control

In practical applications, the microcontroller sends a PWM signal to the ESC, dictating the desired speed of the motor. The ESC receives this input and adjusts the motor phase currents accordingly. This is often managed through a closed-loop system using feedback from sensors to allow real-time speed corrections, ensuring that the motor operates at the desired set point. The integration of a Proportional-Integral-Derivative (PID) controller helps minimize the error between the actual and desired motor states, facilitating precise control of the motor's behavior.

The use of ESCs and PWM for controlling brushless motors is a cornerstone of modern electronic design, enabling advanced functionalities in robotics, aerospace, automotive, and consumer electronics. As technology progresses, we can anticipate further improvements in ESC design, such as enhanced power efficiency, reduced size, and integration of more sophisticated control algorithms. These developments will expand the potential applications for brushless motors, making them integral to the next generation of intelligent systems.



# Third Step: Stabilizing Seesaw with PID Controllers <a name="step3"></a>
## PID control <a name="PID"></a>

  $u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) \, d\tau + K_d \frac{d}{dt}e(t)$

![image](https://github.com/junggeehoon/flight-controller/assets/124639150/ad2658c7-c77c-4942-82cd-7566ace18ea1)


# Fourth Step: Bluetooth Angle Controller <a name="step4"></a>
As using an RF controller in the steering of a drone, we decided to use the Adafruit Bluetooth app and the Bluetooth module with mbed to implement the control of the desired angle of our seesaw. Bluetooth controller was also used when we were finetuning the kp, ki, and kd values in real-time.

![image](https://github.com/junggeehoon/flight-controller/assets/124639150/52f6c0b6-64bc-4566-981f-846d4cbbe369)    ![image](https://github.com/junggeehoon/flight-controller/assets/87514868/715dadea-ae46-4570-b50d-292ee7aa7d9f)

The left and right arrow keys allow easy pitch i.e. tilt control of the prototype. Tilt control is useful in altering the drones orienattion and direction of motion. The arrow keys serve as an intuitive joystick control.

# Results<a name="results"></a>

# Conclusion<a name="conclusion"></a>

# References<a name="references"></a>
