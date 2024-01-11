// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "LoggingLevel.h"
#include <units/math.h>
#include <wpi/sendable/Sendable.h>
#include <ctre/Phoenix.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

class Claw {

    rev::CANSparkMax m_motorRightIntake{constants::clawConstants::RightMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; //declares intake motor
    rev::CANSparkMax m_motorLeftIntake{constants::clawConstants::LeftMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; //declares intake motor
    frc::Solenoid m_clawPistion{1,frc::PneumaticsModuleType::REVPH,constants::clawConstants::SolenoidID}; //swap 1 and 0 if pistion works in reverse

public:

    Claw();
    void Init();

    void SetIntakeSpeed(double Speed); //sets intake motor speed
    void ClawOpen(); //set claw position open
    void ClawClose(); //set claw position close
    void ClawToggle();
    

    void SendData(LoggingLevel verbose); //sends data to dash board
};