#pragma once

#include <units/angle.h>
#include <units/math.h>
#include <units/angular_velocity.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "LoggingLevel.h"
#include "MTechArmConstants.h"
#include "Constants.h"
#include "CTREHelpers.h"


class Arm : public wpi::Sendable /*constructor*/{
private:
    CANCoder m_encoder{constants::armConstants::EncoderID, constants::armConstants::CANBus}; //declares encoder

    TalonFX m_motor1{constants::armConstants::MasterMotorID, constants::armConstants::CANBus}; //declares primary motor
    TalonFX m_motor2{constants::armConstants::SlaveMotorID, constants::armConstants::CANBus}; //declares follower motor

    units::degree_t m_targetAngle; //shows the target angle of the motors/arm in degrees

    void configDevices();
    
public:
    

    Arm();
    void Init();

    void SetAngle(units::degree_t goal); //closed loop set point function
    void SetSpeed(double rawMotorSpeed); //sets motors to % output motor control

    void InitSendable(wpi::SendableBuilder& builder){};
    void SendData(LoggingLevel verbose); //sends LoggingLevel data to dashboard
    units::degree_t GetOffsetAngle(); //gives the offset of the angle in degrees
    units::degree_t GetAngle(); //gives the current angle in degrees
    double GetRawAngle(); //sends the raw angle w/o zeroing it out
};