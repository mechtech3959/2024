#include "Robot.h"

Robot::Robot() {}

void Robot::RobotInit() {
  m_autoChooser.SetDefaultOption(a_Left, AutoRoutine::kLeft);
  m_autoChooser.AddOption(a_Middle, AutoRoutine::kMiddle);
  m_autoChooser.AddOption(a_Right, AutoRoutine::kRight);
  m_autoChooser.AddOption(a_Test, AutoRoutine::kTest);
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

  m_swerve.SetHeading(0_deg);
  headingControl = true;
  driveMode = DriveMode::HeadingControl;

  m_swerve.ResetDriveEncoders();
}

void Robot::UpdatePose() { m_swerve.UpdateOdometry(); }

void Robot::RobotPeriodic() {
  UpdatePose();

  frc::Pose2d bp = m_swerve.GetPose();
  frc::SmartDashboard::PutNumber("Bot Pose X", units::inch_t(bp.X()).value());
  frc::SmartDashboard::PutNumber("Bot Pose Y", units::inch_t(bp.Y()).value());
  frc::SmartDashboard::PutNumber("Bot Pose Heading",
                                 bp.Rotation().Degrees().value());

  m_swerve.SendData();
}

void Robot::TeleopInit() {

  // Set current heading as target heading to avoid unwanted motion
  m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees());

  m_swerve.SetRotationGain(constants::swerveConstants::RotGain);
}

/*
  Driver Back Button Resets Drive Pose
*/
void Robot::TeleopPeriodic() {
  m_swerve.SendData();

  // Reset Drive Pose
  if (driver.GetBackButtonPressed())
    m_swerve.SetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));

  Drive();
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

/*
Driver Control Function

Left Driver Joystick is heading/rotation
Right Driver Joystick is translation in x/y

Driver A Button is Heading Control, points in direction left joystick is
pointing Driver B Button is Velocity Control, just turns based on Left X axis

Driver Right Trigger Scales drive speed

*/
void Robot::Drive() {
  double drivex = -driver.GetLeftY();
  double drivey = -driver.GetLeftX();
  // deadband
  if ((fabs(drivex) + fabs(drivey)) / 2.0 < .1) {
    drivex = 0.0;
    drivey = 0.0;
  }

  // Get Right joystick for heading and velocity control
  double hy = -driver.GetRightX(); // hy is w for vel control
  double hx = -driver.GetRightY();

  // Scale Speed
  units::scalar_t scale = 1.0 - driver.GetRightTriggerAxis() * .9;
  scale = scale * .8;

  // Set control mode
  if (driver.GetAButtonPressed()) {
    driveMode = DriveMode::HeadingControl;
  }

  if (driver.GetBButtonPressed()) {
    driveMode = DriveMode::VelocityMode;
  }

  switch (driveMode) {
  case DriveMode::VelocityMode:
    if (fabs(hy) < .1) { // deadband rot vel, translations were done
      hy = 0.0;
    }
    m_swerve.Drive(m_xspeedLimiter.Calculate(pow(drivex, 3)) *
                       constants::swerveConstants::MaxSpeed * scale,
                   m_yspeedLimiter.Calculate(pow(drivey, 3)) *
                       constants::swerveConstants::MaxSpeed * scale,
                   m_rotLimiter.Calculate(pow(hy, 3)) *
                       constants::swerveConstants::MaxAngularVelocity * scale,
                   true);
    break;
  case DriveMode::HeadingControl:
    if (sqrt(hx * hx + hy * hy) >
        0.9) // make sure the joystick is begin used by calculating magnitude
    {
      m_swerve.SetTargetHeading(frc::Rotation2d(hx, hy).Degrees());
      frc::SmartDashboard::PutNumber("Heading Stick Value",
                                     sqrt(hx * hx + hy * hy));
    }
    m_swerve.DriveXY(m_xspeedLimiter.Calculate(pow(drivex, 1)) *
                         constants::swerveConstants::MaxSpeed * scale,
                     m_yspeedLimiter.Calculate(pow(drivey, 1)) *
                         constants::swerveConstants::MaxSpeed * scale);
    break;
  default:
    break;
  }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
