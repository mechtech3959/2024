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
  bool loadFromIntake = _controller.GetRightBumper();
  bool loadFromFront = _controller.GetLeftBumper();
  // bool climbUp = _controller.GetXButton();
  // bool climbDown = _controller.GetYButton();
  bool forward = _controller.GetAButton();
  bool reverse = _controller.GetBButton();
  // Once we actually have a limit switch this will need to be fixed
  bool noteDetected = false;

  // Deadzone the joysticks
  if (fabs(forw) < 0.10)
    forw = 0;
  if (fabs(spin) < 0.10)
    spin = 0;

  // Run the shooter at 50% for the amp and feed the note in
  if (_controller.GetLeftTriggerAxis() > 0) {
    shooter.ShootAmp();
    intake.Forward();
  }
  // Run the shooter at 100% for the speaker and feed the note in
  if (_controller.GetRightTriggerAxis() > 0) {
    shooter.ShootSpeaker();
    intake.Forward();
  }

  // Set the differential drive to the commanded speed
  drive.diffDrive.ArcadeDrive(forw, spin, true);

  // Load the note in from the front
  if (loadFromFront) {
    if (!noteDetected) {
      shooter.Reverse();
      intake.Reverse();
    } else {
      shooter.Stop();
      intake.Stop();
    }
  }
  // Load the note in from the intake
  if (loadFromIntake) {
    if (!noteDetected) {
      shooter.Reverse();
      intake.Reverse();
    } else {
      shooter.Stop();
      intake.Stop();
    }
  }

  // Run everything forward if the button is pressed
  if (forward) {
    shooter.Forward();
    intake.Forward();
  }

  // Reverse everything if the button is pressed
  if (reverse) {
    shooter.Reverse();
    intake.Reverse();
  }

  // Set the climber motor speed
  // Commented out until we have a climber
  /*
  if (climbUp)
    _climber.Set(1);
  if (climbDown)
    _climber.Set(1);
  */

  // If nothing is commanded stop the motors
  if (_controller.GetXButton()) {
    shooter.Stop();
    intake.Stop();
  }
}

void Robot::DisabledPeriodic() { limelight.autotimer.Reset(); }

void Robot::RobotPeriodic() {}
void Robot::AutonomousInit() {}
void Robot::TeleopInit() {}
void Robot::DisabledInit() {}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
void Robot::TestPeriodic() {}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
