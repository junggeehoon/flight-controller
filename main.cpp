#include "mbed.h"
#include "ahrs.h"
#include "Servo.h"
#include "icm20948.h"
#include <cstdio>
#include <stdint.h>

using namespace std::chrono;
// Timer t1;
// Timer t2;
typedef unsigned char byte;
float selft[6];
static BufferedSerial pc(USBTX, USBRX);
UnbufferedSerial bluetooth(p13,p14);
volatile float DESIRED_ANGLE = 0.0;
BusOut myled(LED1,LED2,LED3,LED4);
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

    // sprintf(msg,"y-axis self test: gyration trim within : %f of factory value\n",selft[4]);
    // pc.write(msg, strlen(msg));
    // sprintf(msg,"z-axis self test: gyration trim within : %f of factory value\n",selft[5]);
    // pc.write(msg, strlen(msg));
    // Calibrate gyro and accelerometers, load biases in bias registers
    // calibrateICM20948(gyroBias, accelBias);

    initICM20948();

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


    getAres();
    getGres();
    getMres();
  
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


// Complemtary filter for combination with gyro
#define ALPHA 0.98 

volatile bool newDataFlag = false; // Flag to indicate that new data has been processed
volatile char newBnum = 0; // Processed button number
volatile char newBhit = 0; // Processed hit/release state

void bluetooth_callback() {
    static int index = 0;
    static char bnum = 0, bhit = 0, checksum = 0;
    char byte;

    if (bluetooth.read(&byte, 1)) {
        switch (index) {
            case 0:
                if (byte == '!') index++;
                break;
            case 1:
                if (byte == 'B') index++;
                else index = 0;
                break;
            case 2:
                bnum = byte;
                index++;
                break;
            case 3:
                bhit = byte;
                index++;
                break;
            case 4:
                checksum = byte;
                if (checksum == char(~('!' + 'B' + bnum + bhit))) {
                    __disable_irq();  // Disable interrupts to safely update shared variables
                    newBnum = bnum;
                    newBhit = bhit;
                    newDataFlag = true;  // Set flag to indicate new data is ready to be processed
                    __enable_irq();   // Re-enable interrupts
                }
                index = 0;
                break;
            default:
                index = 0;
                break;
        }
    }
}


#define DEFAULT_THROTTLE 0.3

// PID gains
// #define KP 0.006  // Proportional gain
volatile double KP = 0.0031;

// volatile double KI = 0.00002;
volatile double KI = 0.000006;

// #define KI 0.00  // Integral gain
// #define KD 0.000004 // Derivative gain
volatile double KD = 0.000013;

int main(void) {
    setup(); //setup IMU
    bluetooth.baud(9600);

    // sprintf(msg,"\nCalibrating gyro...");
    // pc.write(msg, strlen(msg));
    calibrateGyro();

    // sprintf(msg, "\ngy bias: %f degrees", gyroBias[1]);
    // pc.write(msg, strlen(msg));
    
    // sprintf(msg,"\nCalibrating the ESC...");
    // pc.write(msg, strlen(msg));
    calibrateESC();

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
    Timer timer1;
    timer.start();
    timer1.start();
    int count = 0;
    float t = 0;
    bluetooth.attach(&bluetooth_callback, UnbufferedSerial::RxIrq);
    while (1) {
        if (newDataFlag) {
            // Process the data received
            myled = newBnum - '0'; // Update LED based on button number
            switch (newBnum) {
                case '5':
                    if (newBhit == '1') {
                        KP += 0.00005;
                        // KI += 0.0000001;
                        // sprintf(msg, "\nKD::: %.10f", KD);
                        // pc.write(msg, strlen(msg));
                    }
                    break;
                case '6':
                    if (newBhit == '1') {
                        KP -= 0.00005;
                        // KI -= 0.0000001;
                        // sprintf(msg, "\nKD::: %.10f", KD);
                        // pc.write(msg, strlen(msg));
                    }
                    break;
                case '7': // Button 7 left arrow
                    if (newBhit == '1') {
                        // KI -= 0.000001;
                        // KD -= 0.0000005;
                        // sprintf(msg, "\nKP::: %.10f", KP);
                        // pc.write(msg, strlen(msg));
                        DESIRED_ANGLE -= (DESIRED_ANGLE > -45.0) ? 5.0 : 0;
                        sprintf(msg, "\nAHHHHH %2.0f", DESIRED_ANGLE);
                        pc.write(msg, strlen(msg));
                    }
                    break;
                case '8': // Button 8 right arrow
                    if (newBhit == '1') {
                        // KI += 0.000001;
                        // KD += 0.0000005;
                        // sprintf(msg, "\nKP::: %.10f", KP);
                        // pc.write(msg, strlen(msg));
                        DESIRED_ANGLE += (DESIRED_ANGLE < 45.0) ? 5.0 : 0;
                        sprintf(msg, "\nAHHHHH %2.0f", DESIRED_ANGLE);
                        pc.write(msg, strlen(msg));
                    }
                    break;
                
                default:
                    break;
            }
            newDataFlag = false; // Reset the flag after processing
        }

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
            auto duration1 = timer1.elapsed_time();
            dt = duration_cast<milliseconds>(duration).count() / 1000.0f;
            t = duration_cast<milliseconds>(duration1).count() / 1000.0f;
            timer.reset();  // Reset timer for the next loop

            pitch_gyro = pitch + gy * dt;

            // Apply complementary filter
            pitch = ALPHA * pitch_gyro + (1 - ALPHA) * pitch_accel;

            // Calculate PID error terms
            error = pitch - DESIRED_ANGLE;
            
            pid_p = KP * error;

            if (5 < error < 5) {
                pid_i += KI * error * dt;
            } else {
                pid_i = 0.0;
            }
            

            pid_d = KD * ((error - previous_error) / dt);

            // Calculate PID output
            pid = pid_p + pid_i + pid_d;

            previous_error = error;

            if (pid > 0.5) {pid = 0.4;}
            else if(pid < -0.5){pid= -0.4;}

            count++;

            if (count % 20 == 0) {
                count = 0; 
                // sprintf(msg, "\nKP: %f, KD: %f, KI: %f, pid_p: %f, pid_d: %f, pid_i: %f, pitch: %f", KP, KD, KI, pid_p, pid_d, pid_i, pitch);
                // pc.write(msg, strlen(msg));
                sprintf(msg, "\nPitch: %f duraction: %f", pitch, t);
                pc.write(msg, strlen(msg));
                // sprintf(msg, "\nDesired Angle: %f, Current Angle: %f", DESIRED_ANGLE, pitch);
                // pc.write(msg, strlen(msg));
            }
            
            (DESIRED_ANGLE > -45.0) ? 5.0 : 0;

            leftMotor  = DEFAULT_THROTTLE + 0.093 - pid; 
            rightMotor = DEFAULT_THROTTLE + pid;


        }
    }
    return 0;
}
