#include "mbed.h"
#include "ahrs.h"
#include "Servo.h"
#include "icm20948.h"
#include <cstdio>
#include <stdint.h>

using namespace std::chrono;
Timer t1;
typedef unsigned char byte;
float selft[6];
static BufferedSerial pc(USBTX, USBRX);

Servo leftMotor(p21);
Servo rightMotor(p22);


char msg[255];

void setup()
{
    pc.set_baud(9600);
    pc.set_format(
        /* bits */ 8,
        /* parity */ BufferedSerial::None,
        /* stop bit */ 1
    );
  // Reset ICM20948
  begin();
 
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAGS);
  thread_sleep_for(100);
  writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
  thread_sleep_for(100);
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(ICM20948_ADDRESS, WHO_AM_I_ICM20948);
//   sprintf(msg,"ICM20948 I AM 0x %x I should be 0x %x",c,0xEA);
//   pc.write(msg, strlen(msg));

  if (c == 0xEA) // WHO_AM_I should always be 0x71
  {
    sprintf(msg,"ICM20948 is online...\n");
    pc.write(msg, strlen(msg));
   // writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);
    // Start by performing self test and reporting values
    ICM20948SelfTest(selft);
    // sprintf(msg,"x-axis self test: acceleration trim within : %f of factory value\n",selft[0]);
    // pc.write(msg, strlen(msg));
    // sprintf(msg,"y-axis self test: acceleration trim within : %f of factory value\n",selft[1]);
    // pc.write(msg, strlen(msg));
    // sprintf(msg,"z-axis self test: acceleration trim within : %f  of factory value\n",selft[2]);
    // pc.write(msg, strlen(msg));
    // sprintf(msg,"x-axis self test: gyration trim within : %f of factory value\n",selft[3]);
    // pc.write(msg, strlen(msg));
    sprintf(msg,"y-axis self test: gyration trim within : %f of factory value\n",selft[4]);
    pc.write(msg, strlen(msg));
    // sprintf(msg,"z-axis self test: gyration trim within : %f of factory value\n",selft[5]);
    // pc.write(msg, strlen(msg));
    // Calibrate gyro and accelerometers, load biases in bias registers
    // calibrateICM20948(gyroBias, accelBias);

    initICM20948();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    // sprintf(msg,"ICM20948 initialized for active data mode....\n");
    // pc.write(msg, strlen(msg));
        // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    tempCount =readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
    temperature = ((float) tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
    // sprintf(msg,"Temperature is %f degrees C\n",temperature);
    // pc.write(msg, strlen(msg));
    byte d = readByte(AK09916_ADDRESS<<1, WHO_AM_I_AK09916);
    // sprintf(msg,"AK8963 I AM 0x %x  I should be 0x %d\n",d,0x09);
    // pc.write(msg, strlen(msg));

    if (d != 0x09)
    {
      // Communication failed, stop here
    sprintf(msg,"Communication with magnetometer failed, abort!\n");
    pc.write(msg, strlen(msg));
    exit(0);
    }

    // Get magnetometer calibration from AK8963 ROM
    // initAK09916();
    // Initialize device for active mode read of magnetometer
    // sprintf(msg,"AK09916 initialized for active data mode....\n");
    // pc.write(msg, strlen(msg));

    // Get sensor resolutions, only need to do this once
    getAres();
    getGres();
    getMres();
    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    // magCalICM20948(magBias, magScale);
//     sprintf(msg,"AK09916 mag biases (mG)\n %f\n%f\n%f\n",magBias[0],magBias[1],magBias[2]);
//     pc.write(msg, strlen(msg));
//    sprintf(msg,"AK09916 mag scale (mG)\n %f\n%f\n%f\n",magScale[0],magScale[1],magScale[2]);
//     pc.write(msg, strlen(msg));
    // thread_sleep_for(2000); // Add delay to see results before pc spew of data
  } // if (c == 0x71)
  else
  {
    sprintf(msg,"Could not connect to ICM20948: 0x%x",c);
    pc.write(msg, strlen(msg));
    // Communication failed, stop here
    sprintf(msg," Communication failed, abort!\n");
    pc.write(msg, strlen(msg));
    exit(0);
  }
}

void calibrateESC() {
    leftMotor = 0.0;
    rightMotor = 0.0;
    wait_us(500000); //ESC detects signal
//Required ESC Calibration/Arming sequence  
//sends longest and shortest PWM pulse to learn and arm at power on
    leftMotor = 1.0; //send longest PWM
    rightMotor = 1.0;
    wait_us(8000000);
    leftMotor = 0.0; //send shortest PWM
    rightMotor = 0.0;
    wait_us(8000000);
}

#define DESIRED_ANGLE 0
#define DEFAULT_THROTTLE 0.44

// PID gains
#define KP 0.01  // Proportional gain
#define KI 0.00  // Integral gain
#define KD 0.00  // Derivative gain


// Complemtary filter for combination with gyro
#define ALPHA 0.98

int main(void) {
    setup();

    sprintf(msg,"Calibrating the ESC...");
    pc.write(msg, strlen(msg));
    calibrateESC();

    sprintf(msg,"Calibrating gyro...");
    pc.write(msg, strlen(msg));
    calibrateGyro();
    
    sprintf(msg, "\ngy bias: %f degrees", gyroBias[1]);
    pc.write(msg, strlen(msg));

    float pitch_gyro = 0.0f;
    float pitch_accel = 0.0f;
    float pitch = 0.0f;
    float dt = 0.0f; // Time delta in seconds
    float error = 0.0f;
    float previous_error = 0.0f;
    float pid_p = 0.0f;
    float pid_i = 0.0f;
    float pid_d = 0.0f;
    float pid = 0.0f;
    Timer timer;
    timer.start();
    int count = 0;

    while (1) {

        if (readByte(ICM20948_ADDRESS, INT_STATUS_1) & 0x01) {

            // Read accelerometer data
            readAccelData(accelCount);
            ax = (float)accelCount[0] * aRes - accelBias[0];
            ay = (float)accelCount[1] * aRes - accelBias[1];
            az = (float)accelCount[2] * aRes - accelBias[2];
            pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

            // Read gyroscope data
            readGyroData(gyroCount);
            gx = (float)gyroCount[0] * gRes - gyroBias[0];
            gy = (float)gyroCount[1] * gRes - gyroBias[1];
            gz = (float)gyroCount[2] * gRes - gyroBias[2];

            // Calculate time delta
            auto duration = timer.elapsed_time();
            dt = duration_cast<milliseconds>(duration).count() / 1000.0f;
            timer.reset();  // Reset timer for the next loop

            pitch_gyro = pitch + gy * dt;

            // Apply complementary filter
            pitch = ALPHA * pitch_gyro + (1 - ALPHA) * pitch_accel;

            // Calculate PID error terms
            error = pitch - DESIRED_ANGLE;
            
            pid_p = KP * error;

            pid_i += KI * error * dt;

            pid_d = KD * (error - previous_error) / dt;

            // Calculate PID output
            pid = pid_p + pid_i + pid_d;

            previous_error = error;

            if (pid > 1) pid = 0.5;

            count++;

            if (count % 200 == 0) {
                count = 0; 
                sprintf(msg, "\nPID: %f", pid);
                pc.write(msg, strlen(msg));
            }
            
            leftMotor = DEFAULT_THROTTLE + pid;
            // rightMotor = DEFAULT_THROTTLE - pid;


        }
    }
    return 0;
}