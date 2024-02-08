#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
// #include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>

class Robot : public frc::TimedRobot
{
public:

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  WPI_TalonSRX m_Left0[1];
  WPI_TalonSRX m_Left1[2];
  WPI_TalonSRX m_Right0[3];
  WPI_TalonSRX m_Right1[4];


  frc::DifferentialDrive m_Drive{m_Left0, m_Right0};
  frc::XboxController m_Controller;

  bool m_LimelightHasTarget;
  double m_LimelightTurnCmd;
  double m_LimelightDriveCmd;


double clamp(double in, double minval, double maxval)
{
  if (in > maxval)
    return maxval;
  if (in < minval)
    return minval;
  return in;
}



void RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void RobotPeriodic() {}

void AutonomousInit() {}

void AutonomousPeriodic() {}

void TeleopInit() {}

void TeleopPeriodic()
{
  Update_Limelight_Tracking();

  bool do_limelight = m_Controller.GetAButton();
  if (do_limelight)
  {
    if (m_LimelightHasTarget)
    {
      m_Drive.ArcadeDrive(m_LimelightDriveCmd, m_LimelightTurnCmd);
    }
    else
    {
      m_Drive.ArcadeDrive(0.0, 0.0);
    }
  }
  else
  {
    // Tank Drive
    // double left = -m_Controller.GetY(frc::GenericHID::JoystickHand::kLeftHand);
    // double right = -m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand);
    // m_Drive.TankDrive(left,right);

    // Arcade Drive
    double fwd = -m_Controller.GetLeftY();
    double turn = m_Controller.GetLeftX();
    turn *= 0.7f;
    m_Drive.ArcadeDrive(fwd, turn);
  }
}

void TestPeriodic() {}

void Update_Limelight_Tracking()
{
  // Proportional Steering Constant:
  // If your robot doesn't turn fast enough toward the target, make this number bigger
  // If your robot oscillates (swings back and forth past the target) make this smaller
  const double STEER_K = 0.05;

  // Proportional Drive constant: bigger = faster drive
  const double DRIVE_K = 0.26;

  // Area of the target when your robot has reached the goal
  const double DESIRED_TARGET_AREA = 13.0;
  const double MAX_DRIVE = 0.65;
  const double MAX_STEER = 1.0f;

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double tx = table->GetNumber("tx", 0.0);
  double ty = table->GetNumber("ty", 0.0);
  double ta = table->GetNumber("ta", 0.0);
  double tv = table->GetNumber("tv", 0.0);

  if (tv < 1.0)
  {
    m_LimelightHasTarget = false;
    m_LimelightDriveCmd = 0.0;
    m_LimelightTurnCmd = 0.0;
  }
  else
  {
    m_LimelightHasTarget = true;

    // Proportional steering
    m_LimelightTurnCmd = tx * STEER_K;
    m_LimelightTurnCmd = clamp(m_LimelightTurnCmd, -MAX_STEER, MAX_STEER);

    // drive forward until the target area reaches our desired area
    m_LimelightDriveCmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
    m_LimelightDriveCmd = clamp(m_LimelightDriveCmd, -MAX_DRIVE, MAX_DRIVE);
  }
}

};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif