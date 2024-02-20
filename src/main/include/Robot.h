#pragma once

#include "LimelightHelpers.h"
#include <ctre/Phoenix.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>

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
  WPI_TalonSRX _driveRightFront{1};
  WPI_TalonSRX _driveRightFollower{2};
  WPI_TalonSRX _driveLeftFront{3};
  WPI_TalonSRX _driveLeftFollower{4};
  WPI_TalonSRX _launcherFront{5};
  WPI_TalonSRX _launcherFollower{6};
  rev::CANSparkMax _intakeFront{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax _intakeRear{8, rev::CANSparkMax::MotorType::kBrushless};
  // Commented out to prevent errors until we actually have a climber
  // WPI_TalonSRX _climber{9};

  // Initialize the differential drive
  frc::DifferentialDrive _diffDrive{_driveLeftFront, _driveRightFront};

  // Initialize the controller
  frc::XboxController _controller{0};

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
