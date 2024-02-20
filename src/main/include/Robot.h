#pragma once

#include "Constants.h"
#include "Intake.h"
#include "LimeLight.h"
#include "LimelightHelpers.h"
#include "Shooter.h"
#include <ctre/Phoenix.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <units/pressure.h>
#include <units/time.h>

// Unneeded headers
// Leaving them here for future use
// #include <frc2/command/CommandPtr.h>
// #include <frc/MathUtil.h>
// #include <frc/filter/SlewRateLimiter.h>
// #include "ctre/phoenix/motorcontrol/ControlMode.h"
// #include <frc/Compressor.h>
// #include <frc/trajectory/Trajectory.h>
// #include <frc/trajectory/TrajectoryGenerator.h>
// #include <frc/MathUtil.h>
// #include <units/math.h>
// #include <frc/DriverStation.h>
// #include "LimeLight.h"
// #include <units/pressure.h>

class Robot : public frc::TimedRobot {
private:
  // Initialize motors
  WPI_TalonSRX _driveRightFront{constants::drive::rightFrontMotorID};
  WPI_TalonSRX _driveRightFollower{constants::drive::rightRearMotorID};
  WPI_TalonSRX _driveLeftFront{constants::drive::leftFrontMotorID};
  WPI_TalonSRX _driveLeftFollower{constants::drive::leftRearMotorID};

  // Initialize the differential drive
  frc::DifferentialDrive _diffDrive{_driveLeftFront, _driveRightFront};

  // Initialize the controller
  frc::XboxController _controller{0};

  Shooter shooter{};
  Intake intake{};

  // Everything from here until the public block is auto stuff
  // I have no idea how it works, ask Zac
  frc::SendableChooser<std::string> m_chooser;
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
  frc::Timer autotimer;
  void RunMiddleAuto();
  void AmpAuto();

public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  void Update_Limelight_Tracking();
};
