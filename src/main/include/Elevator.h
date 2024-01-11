#pragma once

#include <units/angle.h>
#include <units/math.h>
#include <units/angular_velocity.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "LoggingLevel.h"
#include "Constants.h"
#include "CTREHelpers.h"

class Elevator : public wpi::Sendable {
private:
    CANCoder m_encoder{constants::elevatorConstants::EncoderID, constants::armConstants::CANBus}; //declares encoder

    TalonFX m_motor1{constants::elevatorConstants::MasterMotorID, constants::armConstants::CANBus}; //declares master motor
    TalonFX m_motor2{constants::elevatorConstants::SlaveMotorID, constants::armConstants::CANBus}; //declares slave motor

public:
    
    
    Elevator();
    void Init();
    void configDevices();

    void SetSpeed(double rawMotorSpeed); //sets motors to % output motor control
    void SetHeight(units::inch_t pos);

    units::degrees_per_second_t GetSpeed(); //sends the speed of the motors in deg/sec
    units::inch_t GetHeight();

    void InitSendable(wpi::SendableBuilder& builder){};
    void SendData(LoggingLevel verbose); //sends LoggingLevel data to dashboard
    
};