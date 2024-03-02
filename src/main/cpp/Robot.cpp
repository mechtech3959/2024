#include "Robot.h"

void Robot::RobotInit() {
  // Auto stuff
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::AutonomousPeriodic() {
  limelight.AmpAuto();

  /* autotimer.Start();
   if (autotimer.Get() < 0.5_s) {
     _diffDrive.ArcadeDrive(0.0, 0.0);
   };*/
  /*(m_LimelightHasTarget)
      ? _diffDrive.ArcadeDrive(m_LimelightDriveCmd, m_LimelightTurnCmd, true)
      : _diffDrive.ArcadeDrive(0.0, 0.0);*/
  /*void RunMiddleAuto() {
    autotimer.Reset();
    if(autotimer.Get() < 0.5_s){
      _diffDrive.ArcadeDrive(0.5, 0.0);
    }*/
}

void Robot::TeleopPeriodic() {
  // Get inputs/controller mapping
  double forw = -_controller.GetLeftY();
  double spin = _controller.GetRightX();
  double intakeSpeed = _controller.GetLeftTriggerAxis();
  double shooterSpeed = _controller.GetRightTriggerAxis();
  // bool climbUp = _controller.GetXButton();
  // bool climbDown = _controller.GetYButton();
  bool reverse = _controller.GetAButton();

  // Deadzone the joysticks
  if (fabs(forw) < 0.10)
    forw = 0;
  if (fabs(spin) < 0.10)
    spin = 0;
  // Set the differential drive to the commanded speed
  drive.diffDrive.ArcadeDrive(forw, spin, true);

  // Reverse everything if the button is pressed
  if (reverse) {
    intakeSpeed = -intakeSpeed;
    shooterSpeed = -shooterSpeed;
  }

  intake.SetSpeed(intakeSpeed);
  shooter.SetSpeed(shooterSpeed);

  // Set the climber motor speed
  // Commented out until we have a climber
  /*
  if (climbUp)
    _climber.Set(1);
  if (climbDown)
    _climber.Set(1);
  */
}

void Robot::DisabledPeriodic() { limelight.autotimer.Reset(); }

void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::TeleopInit() {}
void Robot::DisabledInit(){};
void Robot::SimulationInit(){};
void Robot::SimulationPeriodic(){};
void Robot::TestPeriodic() {}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
