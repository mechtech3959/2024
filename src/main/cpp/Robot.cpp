#include "Robot.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"

using namespace frc;

class Robot : public TimedRobot {
public:
  // Initialize motors
  WPI_TalonSRX _driveRightFront{1};
  WPI_TalonSRX _driveRightFollower{2};
  WPI_TalonSRX _driveLeftFront{3};
  WPI_TalonSRX _driveLeftFollower{4};
  WPI_TalonSRX _launcherFront{5};
  WPI_TalonSRX _launcherFollower{6};
  rev::CANSparkMax _intakeFront{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax _intakeRear{8, rev::CANSparkMax::MotorType::kBrushless};
  WPI_TalonSRX _climber{9};

  // Initialize the differential drive
  DifferentialDrive _diffDrive{_driveLeftFront, _driveRightFront};

  // Initialize the controller
  frc::XboxController _controller{0};

  // Everything from here until the next comment is auto stuff
  // I have no idea how it works, ask Zac
  SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";

  std::shared_ptr<nt::NetworkTable> table =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-greenie");

  bool m_LimelightHasTarget;
  double m_LimelightTurnCmd;
  double m_LimelightDriveCmd;

  double clamp(double in, double minval, double maxval) {
    if (in > maxval)
      return maxval;
    if (in < minval)
      return minval;
    return in;
  };

  void RobotInit() override {
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
    _intakeFront.RestoreFactoryDefaults();
    _intakeRear.RestoreFactoryDefaults();
    _climber.ConfigFactoryDefault();

    // Set up followers
    _driveRightFollower.Follow(_driveRightFront);
    _driveLeftFollower.Follow(_driveLeftFront);
    _launcherFollower.Follow(_launcherFront);
    // _intakeRear.Follow(_intakeFront);

    // Set motors to go the correct direction
    _driveRightFront.SetInverted(false);
    _driveRightFollower.SetInverted(false);
    _driveLeftFront.SetInverted(true);
    _driveLeftFollower.SetInverted(true);
    _intakeFront.SetInverted(false);
    _intakeRear.SetInverted(true);

    // Set TalonSRX LEDs to be the correct colors
    _driveRightFront.SetSensorPhase(true);
    _driveLeftFront.SetSensorPhase(true);
  }

  void RobotPeriodic() override {}

  void AutonomousInit() override {}

  // More weird auto stuff
  void AutonomousPeriodic() override {
    Update_Limelight_Tracking();
    (m_LimelightHasTarget)
        ? _diffDrive.ArcadeDrive(m_LimelightDriveCmd, m_LimelightTurnCmd, true)
        : _diffDrive.ArcadeDrive(0.0, 0.0);
  }

  void TeleopInit() override {}

  void TeleopPeriodic() override {
    // Get inputs/controller mapping
    double forw = -_controller.GetLeftY();
    double spin = _controller.GetRightX();
    bool loadFromIntake = _controller.GetRightBumper();
    bool loadFromFront = _controller.GetLeftBumper();
    bool climbUp = _controller.GetXButton();
    bool climbDown = _controller.GetYButton();
    bool forward = _controller.GetAButton();
    bool reverse = _controller.GetBButton();
    // Once we actually have a limit switch this will need to be fixed
    bool noteDetected = false;

    // Deadzone the joysticks
    if (fabs(forw) < 0.10)
      forw = 0;
    if (fabs(spin) < 0.10)
      spin = 0;

    // Run the shooter at 25% for the amp and feed the note in
    if (_controller.GetLeftTriggerAxis() > 0) {
      _launcherFront.Set(ControlMode::PercentOutput, 0.50);
      _intakeFront.Set(1);
      _intakeRear.Set(1);
    }

    // Run the shooter at 100% for the speaker and feed the note in
    if (_controller.GetRightTriggerAxis() > 0) {
      _launcherFront.Set(ControlMode::PercentOutput, 100);
      _intakeFront.Set(1);
      _intakeRear.Set(1);
    }

    // Set the differential drive to the commanded speed
    _diffDrive.ArcadeDrive(forw, spin, true);

    if (loadFromFront) {
      if (!noteDetected) {
        _launcherFront.Set(ControlMode::PercentOutput, -100);
        _intakeFront.Set(-1);
        _intakeRear.Set(-1);
      } else {
        _launcherFront.Set(ControlMode::PercentOutput, 0);
        _intakeFront.Set(0);
        _intakeRear.Set(0);
      }
    }
    if (loadFromIntake) {
      if (!noteDetected) {
        _launcherFront.Set(ControlMode::PercentOutput, 0);
        _intakeFront.Set(1);
        _intakeRear.Set(1);
       } else {
        _intakeFront.Set(0);
        _intakeRear.Set(0);
       }
    }

    // Run everything forward if the button is pressed
    if (forward) {
      _launcherFront.Set(ControlMode::PercentOutput, 100);
      _intakeFront.Set(1);
      _intakeRear.Set(1);
    }

    // Reverse everything if the button is pressed
    if (reverse) {
      _launcherFront.Set(ControlMode::PercentOutput, -100);
      _intakeFront.Set(-1);
      _intakeRear.Set(-1);
    }

    // Set the climber motor speed
    if (climbUp)
      _climber.Set(1);
    if (climbDown)
      _climber.Set(1);

    // If nothing is commanded stop the motors
    if (_controller.GetXButton()) {
      _launcherFront.Set(ControlMode::PercentOutput, 0);
      _intakeFront.Set(0);
      _intakeRear.Set(0);
    }
  }

  void TestPeriodic() override {}

  // This entire function is weird limelight stuff
  // Still no clue how it works, ask Zac
  void Update_Limelight_Tracking() {
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

    if (tv < 1.0) {
      m_LimelightHasTarget = false;
      m_LimelightDriveCmd = 0.0;
      m_LimelightTurnCmd = 0.0;
    } else {
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
