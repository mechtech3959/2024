#pragma once

#include "Constants.h"
#include "Intake.h"
#include "LimeLight.h"
#include "Waypoints.h"

#include "frc/spline/SplineParameterizer.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryConfig.h"
#include "frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/controller/RamseteController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/estimator/PoseEstimator.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelPositions.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RamseteCommand.h>
#include <functional>
#include <memory>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/math.h>
#include <units/pressure.h>
#include <units/time.h>
#include <units/velocity.h>
#include <utility>
#include <vector>
#include <wpi/SymbolExports.h>

class Robot : public frc::TimedRobot {
private:
  // Initialize stuff
  frc::XboxController _controller{0};
  frc::Timer autoTimer;
  frc::PowerDistribution pdh{};
  Intake intake{};
  LimeLight limelight{"limelight-greenie"};
  rev::CANSparkMax leftFrontMotor{constants::drive::leftFrontMotorID,
                                  rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax leftRearMotor{constants::drive::leftRearMotorID,
                                 rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax rightFrontMotor{constants::drive::rightFrontMotorID,
                                   rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax rightRearMotor{constants::drive::rightRearMotorID,
                                  rev::CANSparkMax::MotorType::kBrushed};
  frc::DifferentialDrive diffDrive{rightFrontMotor, leftRearMotor};
  ctre::phoenix6::hardware::TalonFX shooterMotorLeader{
      constants::shooter::motorOneID, constants::canBus};
  ctre::phoenix6::hardware::TalonFX shooterMotorFollower{
      constants::shooter::motorTwoID, constants::canBus};
  // Initialize the Pigeon
  ctre::phoenix6::hardware::Pigeon2 Pigeon{constants::drive::PigeonID,
                                           constants::canBus};
  // Initialize the Encoders
  ctre::phoenix6::hardware::CANcoder m_leftEncoder{
      constants::drive::m_leftEncoderID, constants::canBus};
  ctre::phoenix6::hardware::CANcoder m_rightEncoder{
      constants::drive::m_rightEncoderID, constants::canBus};

  // Creating my kinematics object: track width of 27 inches
  frc::DifferentialDriveKinematics kinematics{27_in};

  frc::DifferentialDriveWheelSpeeds wheelSpeeds{
      units::velocity::meters_per_second_t{
          -m_rightEncoder.GetVelocity().GetValueAsDouble() /
          constants::drive::rotperM},
      units::velocity::meters_per_second_t{
          m_leftEncoder.GetVelocity().GetValueAsDouble() /
          constants::drive::rotperM}};

  frc::DifferentialDriveWheelPositions diffWPos{
      units::meter_t(m_leftEncoder.GetPosition().GetValueAsDouble() /
                     constants::drive::rotperM),
      units::meter_t(-m_rightEncoder.GetPosition().GetValueAsDouble() /
                     constants::drive::rotperM)};
  // Creating my odometry object.
  // Here, our starting pose is 0,0
  frc::DifferentialDriveOdometry m_odometry{
      Pigeon.GetRotation2d(),
      units::meter_t(m_leftEncoder.GetPosition().GetValueAsDouble() /
                     constants::drive::rotperM),
      units::meter_t(-m_rightEncoder.GetPosition().GetValueAsDouble() /
                     constants::drive::rotperM),
      frc::Pose2d{0_in, 0_in, 0_rad}};
  frc::Pose2d pose2dUpdate = m_odometry.GetPose();
  frc::Pose2d pose2d;

  frc::TrajectoryConfig config{1_mps, 1_mps_sq};
  frc::Trajectory traj = frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d{0_m, 0_m, 0_deg},
      {frc::Translation2d{0.125_m, 0.125_m}, frc::Translation2d{0.5_m, 0.3_m}},
      frc::Pose2d{1_m, 0_m, 0_deg}, config);

  frc2::CommandPtr ramseteCommand{frc2::RamseteCommand(
      traj, [this] { return m_odometry.GetPose(); }, frc::RamseteController{},
      frc::SimpleMotorFeedforward<units::meters>{0.22_V, 1.98 * 1_V * 1_s / 1_m,
                                                 0.2 * 1_V * 1_s * 1_s / 1_m},
      kinematics, [this] { return wheelSpeeds; }, frc::PIDController{1, 0, 0},
      frc::PIDController{1, 0, 0},
      [this](auto left, auto right) {
        diffDrive.TankDrive(left.value() / 12, right.value() / 12);
        std::cout << left.value();
      })};

  // Auto selection
  enum AutoRoutine { kTestAuto } m_autoSelected;

  // Auto chooser
  frc::SendableChooser<AutoRoutine> m_autoChooser;
  const std::string a_TestAuto = "secret auto";

  // Intitialize Limelight NetworkTables connection
  std::shared_ptr<nt::NetworkTable> table =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-greenie");

  void waypointtestauto();

  // Shoot functions
  void ShootSpeaker();
  void ShootAmp();
  frc::Rotation2d GetHeading();
  frc::Rotation2d GetGyroHeading();
  // poseupdate
  void poseupdater();

public:
  // Override standard functions
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
};