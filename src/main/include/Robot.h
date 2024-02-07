// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
#pragma once

#include <optional>

#include <frc/TimedRobot.h>
// #include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/CommandScheduler.h>
#include <ctre/Phoenix.h>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include "RobotContainer.h"

class Robot : public frc::TimedRobot
{
public:
  TalonFX MR{1};
  TalonFX CR{2};
  TalonFX ML{3};
  TalonFX CL{4};
  frc::MotorControllerGroup m_left{ML, CL};
  frc::MotorControllerGroup m_right{MR, CR};
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr>
      m_autonomousCommand;

  RobotContainer m_container;
};
*/