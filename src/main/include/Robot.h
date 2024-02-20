// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Compressor.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc/MathUtil.h>
#include <units/math.h>

#include <ctre/Phoenix.h>
#include <frc/drive/DifferentialDrive.h>

#include <frc/DriverStation.h>

#include "LimeLight.h"
#include "LimelightHelpers.h"
#include <units/pressure.h>
#include <units/time.h>

// #include "RobotContainer.h"

// class Robot : public frc::TimedRobot {}

/* void RobotInit() override;
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
*/

// RobotContainer m_container;
frc::Timer autotimer;
void RunMiddleAuto();
void AmpAuto();
