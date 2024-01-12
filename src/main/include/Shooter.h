// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "LoggingLevel.h"
#include <units/math.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h
#include "Constants.h"

class Shooter : public wpi::Sendable {

TalonFX m_motor{constants::ShooterConstants::MotorID, constants::ShooterConstants::CANBus}; //declares motor
CANCoder m_encoder{constants::ShooterConstants::EncoderID, constants::ShooterConstants::CANBus}; //declares encoder

units::inch_t m_target;

public:
    Shooter();
    void Init();
    void configDevices();

    void SetSpeed(double MotorSpeed); //sets % motor speed

    void InitSendable(wpi::SendableBuilder& builder){};
    void SendData(LoggingLevel verbose); //sends data to dash board

};