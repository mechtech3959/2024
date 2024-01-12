// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Compressor.h>

#include <frc/MathUtil.h>
#include <units/math.h>
#include <chrono>
#include <thread>


#include <frc/DriverStation.h>

#include "Drivetrain.h"
#include "SwerveModule.h"
#include "Constants.h"

#include "LimeLight.h"
#include "LoggingLevel.h"
#include "Shooter.h"
#include "Intake.h"
#include "WaypointPoses.h"
#include <units/pressure.h>



class Robot : public frc::TimedRobot {

private:
  frc::XboxController driver{0};
  frc::XboxController codriver{1};

  Shooter m_Shooter{};
  Intake m_Intake{};

  


  double m_speedScale;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{1.5 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{1.5 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  enum DriveMode{
    VelocityMode,
    HeadingControl
    ,TargetTracking
  } driveMode;

  constants::swerveConstants::SwerveConfig config;
  Drivetrain m_swerve{config};
  bool headingControl;

  

  frc::Trajectory traj2Score1;
  frc::Trajectory traj2Piece1;
  frc::Trajectory traj2Score2;
  frc::Trajectory traj2Piece2;
  frc::Trajectory traj2Score3;
  frc::Trajectory trajSwitch;
  frc::Timer autoTimer;
  int autoState;
  waypoints::WaypointPoses waypointLib{};
  frc::SendableChooser<autoSelector::ScorePosition> Score0Chooser;

  enum AutoRoutine {
      kLeft,  
      kMiddle,
      kRight,
      kTest 
  } m_autoSelected;

  frc::SendableChooser<AutoRoutine> m_autoChooser;
  const std::string a_Left = "Left";
  const std::string a_Middle = "Middle";
  const std::string a_Right = "Right";
  const std::string a_Test = "Top Secret";
  

  void Drive();
  void UpdatePose();
  bool FuseLL();
  bool FuseLLNoHeading();

  void GenLeft();
  void RunLeft();
  void GenMiddle();
  void RunMiddle();
  void GenRight();
  void RunRight();
  void GenTest();
  void RunTest();


  void GenTraj();

  void TrackToGoal(frc::Pose2d goal);

 public:

       Robot();
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;
};
