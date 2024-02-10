#include "ctre/Phoenix.h"
#include "frc/TimedRobot.h"
#include "frc/XboxController.h"
#include "frc/drive/DifferentialDrive.h"

using namespace frc;

class Robot : public TimedRobot {
public:
  WPI_TalonSRX _driveRightFront{1};
  WPI_TalonSRX _driveRightFollower{2};
  WPI_TalonSRX _driveLeftFront{3};
  WPI_TalonSRX _driveLeftFollower{4};
  WPI_TalonSRX _launcherFront{5};
  WPI_TalonSRX _launcherFollower{6};
  WPI_TalonSRX _intake{7};

  WPI_TalonSRX _climber{10};

  DifferentialDrive _diffDrive{_driveLeftFront, _driveRightFront};

  frc::XboxController _controller{0};

  void TeleopPeriodic() {

    std::stringstream work;

    double forw = -_controller.GetLeftY();
    double spin = _controller.GetLeftX();
    double launcherSpeed = _controller.GetRightTriggerAxis();
    bool launcherReverse = _controller.GetRightBumper();
    bool intake = _controller.GetLeftBumper();
    bool intakeReverse = _controller.GetAButton();

    if (fabs(forw) < 0.10)
      forw = 0;
    if (fabs(spin) < 0.10)
      spin = 0;

    _diffDrive.ArcadeDrive(forw, spin, false);

    if (launcherReverse)
      launcherSpeed = -launcherSpeed;
    _launcherFront.Set(motorcontrol::TalonSRXControlMode::PercentOutput,
                            launcherSpeed);

    if (intake)
      (intakeReverse)
          ? _intake.Set(motorcontrol::TalonSRXControlMode::PercentOutput, -100)
          : _intake.Set(motorcontrol::TalonSRXControlMode::PercentOutput, 100);
  }

  void RobotInit() {
    // Set all motors to factory defaults
    _driveRightFront.ConfigFactoryDefault();
    _driveRightFollower.ConfigFactoryDefault();
    _driveLeftFront.ConfigFactoryDefault();
    _driveLeftFollower.ConfigFactoryDefault();
    _launcherFront.ConfigFactoryDefault();
    _launcherFollower.ConfigFactoryDefault();

    // Set up followers
    _driveRightFollower.Follow(_driveRightFront);
    _driveLeftFollower.Follow(_driveLeftFront);
    _launcherFollower.Follow(_launcherFront);

    // Invert the right side motors
    _driveRightFront.SetInverted(true);
    _driveRightFollower.SetInverted(true);
    _driveLeftFront.SetInverted(false);
    _driveLeftFollower.SetInverted(false);

    // Set TalonSRX LEDs to be the correct colors
    _driveRightFront.SetSensorPhase(true);
    _driveLeftFront.SetSensorPhase(true);
  }

private:
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
