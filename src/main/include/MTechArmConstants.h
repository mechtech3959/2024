#pragma once

#include <ctre/Phoenix.h>

struct ArmConstants{
    int motor1ID;
    int motor2ID;
    int encoderID;

    CANCoderConfiguration EncoderConfig; //ctre config data
    TalonFXConfiguration Motor1Config; 
    TalonFXConfiguration Motor2Config;
}; //declares ArmConstants 
