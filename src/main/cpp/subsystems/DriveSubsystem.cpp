// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"
#include "rev/CANSparkMax.h"

DriveSubsystem::DriveSubsystem()
    : m_left1{constants::drive::leftFrontMotorID,
              rev::CANSparkMax::MotorType::kBrushed},
      m_left2{constants::drive::leftRearMotorID,
              rev::CANSparkMax::MotorType::kBrushed},
      m_right1{constants::drive::rightFrontMotorID,
               rev::CANSparkMax::MotorType::kBrushed},
      m_right2{constants::drive::rightRearMotorID,
               rev::CANSparkMax::MotorType::kBrushed} {
  wpi::SendableRegistry::AddChild(&m_drive, &m_left1);
  wpi::SendableRegistry::AddChild(&m_drive, &m_right1);

  m_left1.Follow(m_left2);
  m_right1.Follow(m_right2);

  // We need to invert one side of the drivetrain so that positive voltages
  // result in both sides moving forward. Depending on how your robot's
  // gearbox is constructed, you might have to invert the left side instead.
  m_right1.SetInverted(true);
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}
