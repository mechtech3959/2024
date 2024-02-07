#include "ctre/Phoenix.h"
#include "frc/TimedRobot.h"
#include "frc/XboxController.h"
#include "frc/drive/DifferentialDrive.h"

using namespace frc;

class Robot: public TimedRobot {
public:
	WPI_TalonSRX _driveRightFront{1};
	WPI_TalonSRX _driveRightFollower{2};
	WPI_TalonSRX _driveLeftFront{3};
	WPI_TalonSRX _driveLeftFollower{4};
	WPI_TalonSRX _launcherLeftFront{5};
	WPI_TalonSRX _launcherLeftFollower{6};
	WPI_TalonSRX _launcherRightFront{7};
	WPI_TalonSRX _launcherRightFollower{8};
	WPI_TalonSRX _intake{9};
	WPI_TalonSRX _climber{10};

	DifferentialDrive _diffDrive{_driveLeftFront, _driveRightFront};

	frc::XboxController _controller{0};

	Faults _faults_L;
	Faults _faults_R;

	void TeleopPeriodic() {

		std::stringstream work;

		double forw = -_controller.GetLeftY();
		double spin = _controller.GetLeftX();
		double launcherSpeed = _controller.GetRightTriggerAxis();
		bool launcherReverse = _controller.GetRightBumper();
		bool intake = _controller.GetLeftBumper();
		bool intakeReverse = _controller.GetAButton();

		if (fabs(forw) < 0.10) forw = 0;
		if (fabs(spin) < 0.10) spin = 0;

		_diffDrive.ArcadeDrive(forw, spin, false);
		
		if (launcherReverse) launcherSpeed = -launcherSpeed;
		_launcherRightFront.Set(motorcontrol::TalonSRXControlMode::PercentOutput, launcherSpeed);

		if (intake) (intakeReverse) ? _intake.Set(motorcontrol::TalonSRXControlMode::PercentOutput, -100) : _intake.Set(motorcontrol::TalonSRXControlMode::PercentOutput, 100);
	}

	void RobotInit() {
		_driveRightFront.ConfigFactoryDefault();
		_driveRightFollower.ConfigFactoryDefault();
		_driveLeftFront.ConfigFactoryDefault();
		_driveLeftFollower.ConfigFactoryDefault();
		_launcherLeftFront.ConfigFactoryDefault();
		_launcherLeftFollower.ConfigFactoryDefault();
		_launcherRightFront.ConfigFactoryDefault();
		_launcherRightFront.ConfigFactoryDefault();

		_driveRightFollower.Follow(_driveRightFront);
		_driveLeftFollower.Follow(_driveLeftFront);
		_launcherLeftFollower.Follow(_launcherLeftFront);
		_launcherRightFollower.Follow(_launcherRightFront);
		_launcherRightFront.Follow(_launcherLeftFront);

		_driveRightFront.SetInverted(true);
		_driveRightFollower.SetInverted(true);
		_driveLeftFront.SetInverted(false);
		_driveLeftFollower.SetInverted(false);

		_driveRightFront.SetSensorPhase(true);
		_driveLeftFront.SetSensorPhase(true);
	}

private:
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
