#include "Robot.h"
#include "LimeLight.h"

void Robot::RobotInit() {}

void Robot::AutonomousInit() {
  limelight.autoTimer.Start();
  while (limelight.autoTimer.Get() < 1.0_s) {
    drive.diffDrive.ArcadeDrive(-0.6, 0.0);
  }
  while (limelight.autoTimer.Get() < 1.7_s) {
    drive.diffDrive.ArcadeDrive(0.0, 0.6);
  }
  /*
  while (limelight.autoTimer.Get() < 15.0_s) {
    limelight.updateTracking();
    drive.diffDrive.ArcadeDrive(0.4, limelight.m_LimelightTurnCmd);
  }
  */

  limelight.updateTracking();
  long long ID = llround(limelight.aprilTagID);
  switch (ID) {
  case 6:
    limelight.ampAuto();
    break;
  default:
    drive.diffDrive.ArcadeDrive(0.0, 0.0);
    break;
  }
}
void Robot::AutonomousPeriodic() {
  /*
  limelight.autoTimer.Start();
  if (limelight.autoTimer.Get() < 0.5_s) {
    drive.diffDrive.ArcadeDrive(0.0, 0.0);
  }
  (limelight.m_LimelightHasTarget)
      ? drive.diffDrive.ArcadeDrive(limelight.m_LimelightDriveCmd,
                                    limelight.m_LimelightTurnCmd, true)
      : drive.diffDrive.ArcadeDrive(0.0, 0.0);
  */
}

void Robot::TeleopPeriodic() {
  // Get inputs/controller mapping
  double forw = -_controller.GetLeftY();
  double spin = _controller.GetRightX();
  bool loadFromIntake = _controller.GetRightBumper();
  bool loadFromFront = _controller.GetLeftBumper();
  bool forward = _controller.GetAButton();
  bool reverse = _controller.GetBButton();
  // Once we actually have a limit switch this will need to be fixed
  bool noteDetected = false;
  // Once we have a climber this will need to be fixed
  // bool climbUp = _controller.GetXButton();
  // bool climbDown = _controller.GetYButton();

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

void Robot::DisabledInit() {
  limelight.autoTimer.Stop();
  limelight.autoTimer.Reset();
}

void Robot::RobotPeriodic() {}
void Robot::TeleopInit() {}
void Robot::DisabledPeriodic() {}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
void Robot::TestPeriodic() {}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
