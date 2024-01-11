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
#include "MTechArm.h"
#include "Elevator.h"
#include "Slide.h"
#include "Claw.h"
#include "WaypointPoses.h"
#include "AutoSelector.h"
#include <units/pressure.h>



class Robot : public frc::TimedRobot {

private:
  frc::XboxController driver{0};
  frc::XboxController codriver{1};

  Slide m_slide{};
  Elevator m_elevator{};
  Arm m_arm{};
  Claw m_claw{};

  frc::Compressor m_compressor{1, frc::PneumaticsModuleType::REVPH};

  


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

  LimeLight m_leftLL{"limelight-left"};
  LimeLight m_rightLL{"limelight-right"};
  

  frc::Trajectory traj2Score1;
  frc::Trajectory traj2Piece1;
  frc::Trajectory traj2Score2;
  frc::Trajectory traj2Piece2;
  frc::Trajectory traj2Score3;
  frc::Trajectory trajSwitch;
  bool takeSpeedBump;
  frc::Timer autoTimer;
  int autoState;
  waypoints::WaypointPoses waypointLib{};
  frc::SendableChooser<autoSelector::ScorePosition> Score0Chooser;

  enum AutoRoutine {
      k2PieceCorridor,  
      k2PieceCorridorSwitch,
      kSimpleSwitch,
      kSpeedBump,
      kTest 
  } m_autoSelected;

  frc::SendableChooser<AutoRoutine> m_autoChooser;
  const std::string a_2PieceCorridor = "2 Piece Corridor";
  const std::string a_2PieceCorridorSwitch = "2 Piece Corridor and Switch";
  const std::string a_SimpleSwitch = "1 Piece and Switch";
  const std::string a_SpeedBump = "2 Piece Bump";
  const std::string a_Test = "Top Secret";
  

  void Drive();
  void UpdatePose();
  bool FuseLL();
  bool FuseLLNoHeading();

  void Gen2PieceCorridor();
  void Run2PieceCorridor();
  void GenSimpleSwitch();
  void RunSimpleSwitch();
  void GenSpeedBump();
  void RunSpeedBump();
  void GenTest();
  void RunTest();


  void GenTraj();

  void TrackToGoal(frc::Pose2d goal);

  void PickupPos();
  void GroundPos();
  void MidPos();
  void HighPos();
  void TuckPos();
  void UpTuckPos();

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
