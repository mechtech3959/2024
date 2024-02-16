#include "Robot.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

using namespace frc;

class Robot : public TimedRobot
{
private:
  LimeLight billy{"limelight-greenie"};

public:
  WPI_TalonSRX _driveRightFront{1};
  WPI_TalonSRX _driveRightFollower{2};
  WPI_TalonSRX _driveLeftFront{3};
  WPI_TalonSRX _driveLeftFollower{4};
  WPI_TalonSRX _launcherFront{5};
  WPI_TalonSRX _launcherFollower{6};
  WPI_TalonFX _intakeFront{7};
  WPI_TalonFX _intakeRear{8};
  WPI_TalonSRX _climber{9};

  // Standard drive
  DifferentialDrive _diffDrive{_driveLeftFront, _driveRightFront};

  // Inverted drive
  // DifferentialDrive _diffDrive{_driveRightFront, _driveLeftFront};

  frc::XboxController _controller{0};

  SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";

  std::shared_ptr<nt::NetworkTable> table =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-greenie");

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
  };

  void RobotInit()
  {
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

    // Set all motors to factory defaults
    _driveRightFront.ConfigFactoryDefault();
    _driveRightFollower.ConfigFactoryDefault();
    _driveLeftFront.ConfigFactoryDefault();
    _driveLeftFollower.ConfigFactoryDefault();
    _launcherFront.ConfigFactoryDefault();
    _launcherFollower.ConfigFactoryDefault();
    _intakeFront.ConfigFactoryDefault();
    _intakeRear.ConfigFactoryDefault();
    _climber.ConfigFactoryDefault();

    // Set up followers
    _driveRightFollower.Follow(_driveRightFront);
    _driveLeftFollower.Follow(_driveLeftFront);
    _launcherFollower.Follow(_launcherFront);

    // Invert the right side motors
    _driveRightFront.SetInverted(true);
    _driveRightFollower.SetInverted(true);
    _driveLeftFront.SetInverted(false);
    _driveLeftFollower.SetInverted(false);
    _intakeFront.SetInverted(false);
    _intakeRear.SetInverted(true);

    // Set TalonSRX LEDs to be the correct colors
    _driveRightFront.SetSensorPhase(true);
    _driveLeftFront.SetSensorPhase(true);
  }

  void RobotPeriodic() override {}

  void AutonomousInit() override {}

  void AutonomousPeriodic() override
  {
    Update_Limelight_Tracking();
    // if (_controller.GetAButton()) {
    if (m_LimelightHasTarget)
    {
      // Proportional steering
      _diffDrive.ArcadeDrive(m_LimelightDriveCmd, m_LimelightTurnCmd, true);
    }
    else
    {
      _diffDrive.ArcadeDrive(0.0, 0.0);
    }
    /*} else {
      // Tank Drive
      // double left = -m_Cont
      // roller.GetY(frc::GenericHID::JoystickHand::kLeftHand); double right =
      // -_controller.GetY(frc::GenericHID::JoystickHand::kRightHand);
      // _diffDrive.TankDrive(left,right);

      // Arcade Drive
      double fwd = -_controller.GetLeftY();
      double turn = _controller.GetLeftX();
      turn *= 0.7f;
      _diffDrive.ArcadeDrive(fwd, turn);
    }*/
  }

  void TeleopInit() override {}

  void TeleopPeriodic()
  {

    double forw = -_controller.GetLeftY();
    double spin = _controller.GetLeftX();
    double launcherSpeed = _controller.GetRightTriggerAxis();
    bool launcherReverse = _controller.GetRightBumper();
    bool intake = _controller.GetLeftBumper();
    bool intakeReverse = _controller.GetAButton();
    bool climbUp = _controller.GetXButton();
    bool climbDown = _controller.GetYButton();

    if (fabs(forw) < 0.10)
      forw = 0;
    if (fabs(spin) < 0.10)
      spin = 0;

    _diffDrive.ArcadeDrive(forw, spin, false);

    if (launcherReverse)
      launcherSpeed = -launcherSpeed;
    _launcherFront.Set(ControlMode::PercentOutput, launcherSpeed);

    if (intake)
    {
      if (intakeReverse)
      {
        _intakeFront.Set(ControlMode::PercentOutput, -1);
        _intakeRear.Set(ControlMode::PercentOutput, -1);
      }
      else
      {
        _intakeFront.Set(ControlMode::PercentOutput, 1);
        _intakeRear.Set(ControlMode::PercentOutput, 1);
      }
    } else {
      _intakeFront.Set(ControlMode::PercentOutput, 0);
        _intakeRear.Set(ControlMode::PercentOutput, 0);
    }

    if (climbUp)
      _climber.Set(ControlMode::PercentOutput, 100);
    if (climbDown)
      _climber.Set(ControlMode::PercentOutput, 100);
  }

  void TestPeriodic() override {}

  void Update_Limelight_Tracking()
  {
    // Proportional Steering Constant:
    // If your robot doesn't turn fast enough toward the target, make this
    // number bigger If your robot oscillates (swings back and forth past the
    // target) make this smaller
    const double STEER_K = 0.03;

    // Proportional Drive constant: bigger = faster drive
    const double DRIVE_K = 0.26;

    // Area of the target when your robot has reached the goal
    const double DESIRED_TARGET_AREA = 2.0;
    const double MAX_DRIVE = 1.0;
    const double MAX_STEER = 1.0f;

    double tx = LimelightHelpers::getTX("limelight-greenie");
    double ty = LimelightHelpers::getTY("limelight-greenie");
    double ta = LimelightHelpers::getTA("limelight-greenie");
    double tv = LimelightHelpers::getTV("limelight-greenie");

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
