#pragma once

#include "Constants.h"
#include "Drivetrain.h"

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

class Robot : public frc::TimedRobot {

private:
  frc::XboxController driver{0};

  double m_speedScale;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{1.5 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{1.5 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  enum DriveMode { VelocityMode, HeadingControl, TargetTracking,woah } driveMode;

  constants::swerveConstants::SwerveConfig config;
  Drivetrain m_swerve{config};
  bool headingControl;

  enum AutoRoutine { kLeft, kMiddle, kRight, kTest } m_autoSelected;

  frc::SendableChooser<AutoRoutine> m_autoChooser;
  const std::string a_Left = "Left";
  const std::string a_Middle = "Middle";
  const std::string a_Right = "Right";
  const std::string a_Test = "Top Secret";
  double dashScale = 1;
 
 
  void Drive();
  void UpdatePose();

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
