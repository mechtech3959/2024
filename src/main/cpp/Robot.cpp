#include "Robot.h"

void Robot::RobotInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopPeriodic() {
  // Get inputs/controller mapping
  double forw = -_controller.GetLeftY();
  double spin = _controller.GetRightX();

  // Deadzone the joysticks
  if (fabs(forw) < 0.10)
    forw = 0;
  if (fabs(spin) < 0.10)
    spin = 0;

  // Set the differential drive to the commanded speed
  drive.diffDrive.ArcadeDrive(forw, spin, true);
}

void Robot::DisabledPeriodic() {}

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
