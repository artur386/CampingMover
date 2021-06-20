#include <Arduino.h>
#include "HC05.h"

#ifdef HC05_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
HC05 btSerial = HC05(A2, A5, A4, A3); // cmd, state, rx, tx
#else
HC05 btSerial = HC05(3, 2); // cmd, state
#endif

#ifdef DEBUG_HC05
#ifdef DEBUG_SW_PORT
extern SoftwareSerial DEBUG_PORT;
#endif
#endif
#define BT_POWER 13
#define RL_R_B 12
#define RL_R_F 9
#define RL_L_B 10
#define RL_L_F 11

#define PWM_L 5
#define PWM_R 6

#define PEROID_TIME 100
#define SOFT_START_TIME 2000

byte BT_KEYS;
unsigned long LastReadTime;
unsigned long LeftMotorTime;
unsigned long RightMotorTime;

bool LeftMotorIsOn, LeftMotorFlag, LeftSoftStart;
bool RightMotorIsOn, RightMotorFlag, RightSoftStart;
bool StopFlag;

String pad;
// String joy;

byte a = 0, b = 0;

void setup()
{

    DEBUG_BEGIN(57600);
    DEBUG_PRINTLN("Setup");
    delay(500); // this delay is for debugging convenience only
    DEBUG_PRINTLN("DelayComplete");
    btSerial.println(btSerial.findBaud());

    // INITALIZE OUTPUT PINS TO HANDLE RELAYS AND BT MODULE.
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RL_L_B, OUTPUT);
    pinMode(RL_L_F, OUTPUT);
    pinMode(RL_R_B, OUTPUT);
    pinMode(RL_R_F, OUTPUT);
    pinMode(BT_POWER, OUTPUT);

    digitalWrite(BT_POWER, LOW); // turn on bt module (low level activity)

    // turn off all relays (high lvl activity)
    digitalWrite(RL_L_B, LOW);
    digitalWrite(RL_L_F, LOW);
    digitalWrite(RL_R_B, LOW);
    digitalWrite(RL_R_F, LOW);

    // sequence flags initialize
    RightMotorIsOn = false;
    LeftMotorIsOn = false;
    RightMotorFlag = false;
    LeftMotorFlag = false;
    RightSoftStart = false;
    LeftSoftStart = false;
    StopFlag = false;
}

void loop()
{
    String INCOMING_TEXT;

    // handeled bt transmition and recognize button press
    if (btSerial.connected())
    {
        if (btSerial.available() > 0)
        {
            INCOMING_TEXT = btSerial.readStringUntil('#');
            pad = INCOMING_TEXT.substring(1, 2);
            // joy = INCOMING_TEXT.substring(0, 1);

            // if you press button on smartphone bt app
            // and they are property received then last read time is updated.
            // if not all motors going to stop
            LastReadTime = millis();
        }
    }
    else
    {
        DEBUG_PRINTLN("Disconnected");
        delay(1000);
        btSerial.println(btSerial.findBaud());
    }

    if (pad.equals("2")) // back
    {
        digitalWrite(RL_L_F, LOW);
        digitalWrite(RL_R_F, LOW);
        digitalWrite(RL_L_B, HIGH);
        digitalWrite(RL_R_B, HIGH);
        RightMotorFlag = true;
        LeftMotorFlag = true;
    }
    else if (pad.equals("3")) // right
    {
        digitalWrite(RL_L_B, LOW);
        digitalWrite(RL_R_F, LOW);
        digitalWrite(RL_L_F, HIGH);
        digitalWrite(RL_R_B, HIGH);

        RightMotorFlag = true;
        LeftMotorFlag = true;
    }
    else if (pad.equals("4")) // forward
    {
        digitalWrite(RL_L_B, LOW);
        digitalWrite(RL_R_B, LOW);
        digitalWrite(RL_L_F, HIGH);
        digitalWrite(RL_R_F, HIGH);
        RightMotorFlag = true;
        LeftMotorFlag = true;
    }
    else if (pad.equals("1")) // left
    {

        digitalWrite(RL_L_B, HIGH);
        digitalWrite(RL_R_B, LOW);
        digitalWrite(RL_L_F, LOW);
        digitalWrite(RL_R_F, HIGH);
        RightMotorFlag = true;
        LeftMotorFlag = true;
    }
    // joy = "";
    pad = "";

    INCOMING_TEXT = ""; // clear text buffor
    btSerial.flush();   // flush not read text in bt serial.

    if (millis() - LastReadTime >= PEROID_TIME)
    {
        StopFlag = true;
        RightMotorFlag = false;
        LeftMotorFlag = false;
    }

    /*start silnika lewego*/
    if (LeftMotorFlag && !LeftMotorIsOn)
    {
        if (!LeftSoftStart)
        {
            LeftSoftStart = true;
            LeftMotorTime = millis();
        }

        if (LeftSoftStart)
        {
            int time = millis() - LeftMotorTime;
            int PWM_LEFT;
            if (time <= SOFT_START_TIME)
            {
                PWM_LEFT = map(time, 0, SOFT_START_TIME, 200, 5);
            }
            else
            {
                PWM_LEFT = 5;
                LeftMotorIsOn = true;
            }
            analogWrite(PWM_L, PWM_LEFT);
        }
    }

    /*start silnika prawego*/
    if (RightMotorFlag && !RightMotorIsOn)
    {
        if (!RightSoftStart)
        {
            RightSoftStart = true;
            RightMotorTime = millis();
        }

        if (RightSoftStart)
        {
            int time = millis() - RightMotorTime;
            int PWM_RIGHT;
            if (time <= SOFT_START_TIME)
            {
                PWM_RIGHT = map(time, 0, SOFT_START_TIME, 200, 5);
            }
            else
            {
                PWM_RIGHT = 5;
                RightMotorIsOn = true;
            }
            analogWrite(PWM_R, PWM_RIGHT);
        }
    }

    if (StopFlag)
    {
        analogWrite(PWM_L, 0);
        analogWrite(PWM_R, 0);
        digitalWrite(RL_L_B, LOW);
        digitalWrite(RL_L_F, LOW);
        digitalWrite(RL_R_B, LOW);
        digitalWrite(RL_R_F, LOW);

        RightMotorIsOn = false;
        LeftMotorIsOn = false;
        RightMotorFlag = false;
        LeftMotorFlag = false;
        RightSoftStart = false;
        LeftSoftStart = false;

        StopFlag = false;
    }
}
