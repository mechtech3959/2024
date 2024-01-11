


#pragma once

#include "LoggingLevel.h"
#include <units/math.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "CTREHelpers.h"

class Slide : public wpi::Sendable {

TalonFX m_motor{constants::slideConstants::MotorID, constants::slideConstants::CANBus}; //declares motor
//CANCoder m_encoder{constants::slideConstants::EncoderID, constants::slideConstants::CANBus}; //declares encoder

units::inch_t m_target;

public:
    Slide();
    void Init();
    void configDevices();

    void SetSpeed(double MotorSpeed); //sets % motor speed
    void SetPosition(units::inch_t position); // sets motor position between forward or back

    void SlideForward();
    void SlideBack();

    units::inch_t GetTargetPos(){return m_target;};

    void InitSendable(wpi::SendableBuilder& builder){};
    void SendData(LoggingLevel verbose); //sends data to dash board

    units::inch_t GetPosition(); //returns position
};