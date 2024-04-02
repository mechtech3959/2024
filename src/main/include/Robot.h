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
  frc::DifferentialDriveKinematics Kinematics{27_in};

  // double rightEncoderRotationsperMeter =
  // m_rightEncoder.GetVelocity().GetValueAsDouble() *
  // constants::drive::rotperM; double leftEncoderRotationsperMeter =
  // m_leftEncoder.GetVelocity().GetValueAsDouble() * constants::drive::rotperM;

  frc::DifferentialDriveWheelSpeeds wheelSpeeds{
      units::velocity::meters_per_second_t{
          m_rightEncoder.GetVelocity().GetValueAsDouble() *
          constants::drive::rotperM},
      units::velocity::meters_per_second_t{
          m_leftEncoder.GetVelocity().GetValueAsDouble() *
          constants::drive::rotperM}};
  // DifferentialDriveWheelPositions
  frc::DifferentialDriveWheelPositions diffWPos{
      units::meter_t(m_leftEncoder.GetPosition().GetValueAsDouble()),
      units::meter_t(m_rightEncoder.GetPosition().GetValueAsDouble())};
  // Creating my odometry object.
  // Here, our starting pose is 0,0
  frc::DifferentialDriveOdometry m_odometry{
      Pigeon.GetRotation2d(),
      units::meter_t(m_leftEncoder.GetPosition().GetValueAsDouble() *
                     constants::drive::rotperM),
      units::meter_t(m_rightEncoder.GetPosition().GetValueAsDouble() *
                     constants::drive::rotperM),
      frc::Pose2d{0_in, 0_in, 0_rad}};
  frc::Pose2d pose2dUpdate = m_odometry.GetPose();
  frc::Pose2d pose2d;
  frc::Pose2d Xtraj = {m_odometry.GetPose().Translation(),
                       Pigeon.GetRotation2d()};

  frc::TrajectoryConfig trajconfig{1_mps, 1_mps_sq};

  const std::vector<frc::Pose2d> waypoint{};

  frc::Trajectory traj =
      frc::TrajectoryGenerator::GenerateTrajectory(waypoint, trajconfig);
  // Using the default constructor of RamseteController. Here
  // the gains are initialized to 2.0 and 0.7.
  frc::RamseteController controller1;
  frc::ChassisSpeeds adjustedSpeeds{
      controller1.Calculate(pose2dUpdate, traj.Sample(3.4_s))};

  // frc::DifferentialDriveWheelSpeeds wheelSpeeds =
  // kinematics.ToWheelSpeeds(adjustedSpeeds);
  // auto [Tleft, Tright] = kinematics.ToWheelSpeeds(adjustedSpeeds);

  // Using the secondary constructor of RamseteController where
  // the user can choose any other gains.
  // frc::RamseteController controller2{2.1, 0.8};
  // Initialize the field
  frc::Field2d Field;

  // Auto selection
  enum AutoRoutine { kTestAuto } m_autoSelected;

  // Auto chooser
  frc::SendableChooser<AutoRoutine> m_autoChooser;
  const std::string a_TestAuto = "secret auto";

  // Intitialize Limelight NetworkTables connection
  std::shared_ptr<nt::NetworkTable> table =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-greenie");

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

  // Auto routines
  void testAuto();
  void waypointtestauto();

  // LED functions
  void Rainbow();
  void Green();
  void Red();
  void Yellow();
  void Blue();

  // Shoot functions
  void ShootSpeaker();
  void ShootAmp();
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
  frc::Rotation2d GetHeading();
  frc::Rotation2d GetGyroHeading();
  // poseupdate
  void poseupdater();
};