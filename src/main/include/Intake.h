// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "Constants.h"
#include "LoggingLevel.h"
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <units/math.h>
#include <wpi/sendable/Sendable.h>

class Intake {

    TalonFX m_intakemotor; // declares motor
    TalonFX m_posmotor; // declares motor

public:
  Intake();
  void Init();

  void SetIntakeSpeed(double Speed); // sets intake motor speed
  void IntakeDown();                 // set Intake position down
  void IntakeUp();                   // set Intake position up
  void IntakeToggle();
  units::inch_t m_target;
  void SetPosition(
      units::inch_t position); // sets motor position between forward or back
  units::inch_t GetTargetPos() { return m_target; };
  units::inch_t GetPosition(); // returns position

  void SendData(LoggingLevel verbose); // sends data to dash board
};
