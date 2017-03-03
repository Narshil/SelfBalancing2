//bringing mpu, pid pigpio, and opencv libraries togethor.
#include <iostream>
#include <string>
#include <thread>
#include <highgui.hpp> 
#include <imgproc.hpp> //for resize()


#include <pigpio.h>//pigpio library
#include "PID_v1.h" //include pid library

//mpu includes
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "MotionSensor.h"
using namespace std;
using namespace cv;

//   PIGPIO Global Variables //
static int gpioA = 18; //pin 18: PWM channel 0  All models
int duty = 500000;
int frequencyA = 50; //us high/low
static int gpioB = 19; //pin 19: PWM channel 1  All models
int frequencyB = 50; //us high/low
//---------------------------//

/// OPENCV Global Variables   //
string window1 = "window 1";
const int kp_slider_max = 500;
int kp_slider = 350;
const int ki_slider_max = 100;
int ki_slider = 40;
const int kd_slider_max = 100;
int kd_slider = 60;
const int setPoint_slider_max = 20;
int setPoint_slider = 3;
//---------------------------//

//MOTOR PINS//
static int sleepPin = 26;
static int m0 = 5;
static int m1 = 6;
static int m2 = 13;
static int dir = 21;
bool motorState = false;
//------------//

float currentAngle = 0; //somewhere to store updated angle (40hz update from lib)


// PID VARIABLES //
//Define Variables we'll be connecting to
double Setpoint = -2.5, Input = 0, Output = 0;

//Define the aggressive and conservative Tuning Parameters
double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = kp_slider, consKi = 0.0, consKd = (double) kd_slider / 100;
int divider1 =10;

//----------------------------------------------//

void on_trackbarA(int, void*) {
    consKp = (double) kp_slider; //set opecv:slider to pgio:period
}

void on_trackbarB(int, void*) {
    consKi = (double) ki_slider / divider1;
}

void on_trackbarC(int, void*) {
    consKd = (double) kd_slider / divider1;
}
void on_trackbarD(int, void*) {
    Setpoint = setPoint_slider;
}

void enableMotor() {
    //set to 1/8th resolution
    gpioWrite(m0, 1);
    gpioWrite(m1, 1);
    gpioWrite(m2, 0);
    gpioWrite(sleepPin, 1); //motor works when this is low
    motorState = true;
}

void disableMotor() { //set frequency to zero and disable motor drivers
    gpioHardwarePWM(gpioA, 0, duty);
    gpioHardwarePWM(gpioB, 0, duty);
    gpioWrite(sleepPin, 0);
    motorState = false;
}

float grabAngle() {
    //    #define delay_ms(a) usleep(a*1000)
    ms_open();
    do {
        ms_update();
        //		printf("yaw = %2.1f\tpitch = %2.1f\troll = %2.1f\ttemperature = %2.1f\tcompass = %2.1f, %2.1f, %2.1f\n",
        //		 ypr[YAW], ypr[PITCH],
        //		 ypr[ROLL],temp,compass[0],compass[1],compass[2]);
        //		delay_ms(5);
        currentAngle = ypr[PITCH];

    } while (1);
}

int main() {
    if (gpioInitialise() < 0) return -1; //pigio: init to use funcs
    enableMotor();//enable and set resolution

    //  PID SETUP //
    //Specify the links and initial tuning parameters
    PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-18000, 18000);
    myPID.SetSampleTime(10);
    //---------------------------//

    gpioSetMode(gpioA, PI_OUTPUT);
    gpioSetMode(gpioB, PI_OUTPUT);
    gpioSetMode(dir, PI_OUTPUT);

    namedWindow(window1, 1); //opencv window
    //tabs added to allign sliders, thats all.
    createTrackbar("kP\t", window1, &kp_slider, kp_slider_max, on_trackbarA);
    createTrackbar("kI\t", window1, &ki_slider, ki_slider_max, on_trackbarB);
    createTrackbar("kD\t", window1, &kd_slider, kd_slider_max, on_trackbarC);
//    createTrackbar("Setpoint", window1, &setPoint_slider, setPoint_slider_max, on_trackbarD);

    thread t2(grabAngle);//grab angle in a separate thread
    myPID.SetTunings(kp_slider/divider1, ki_slider/divider1, kd_slider/divider1);    
    int motorspeed;
    while (1) {
        myPID.SetTunings(consKp, consKi, consKd);
        Input = currentAngle;
        cout << "Current angle: " << currentAngle << "\t";
        myPID.Compute(); //calculate pid output
        motorspeed = (int) abs(Output);
        if (Output > 1000) {
            enableMotor();
            gpioWrite(dir, 1); // Set GPIO21 high.
            gpioHardwarePWM(gpioA, motorspeed, duty);
            gpioHardwarePWM(gpioB, motorspeed, duty);
        }
        else if (Output < -1000) {
            enableMotor();
            gpioWrite(dir, 0); // Set GPIO21 low
            gpioHardwarePWM(gpioA, motorspeed, duty);
            gpioHardwarePWM(gpioB, motorspeed, duty);
        }
        else
            disableMotor();
        

        cout << "Output : " << motorspeed << "\t";
        cout << "kp: " << myPID.GetKp() << "\t ki:" << myPID.GetKi() << "\t kd:" << myPID.GetKd() << "\t Motor State:" << motorState << endl;
        if (waitKey(5) == 27)break; //opencv: hit esc to exit
    }
    disableMotor();
    gpioTerminate(); //pgpio: end use of func

    return 0;
}

