#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/Kinematics.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <units/voltage.h>

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  // Will be called periodically whenever the CommandScheduler runs.
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  void ArcadeDrive(double fwd, double rot);

  /**
   * Controls each side of the drive directly with a voltage.
   *
   * @param left the commanded left output
   * @param right the commanded right output
   */
  void TankDriveVolts(units::volt_t left, units::volt_t right);

  // Resets the drive encoders to currently read a position of 0.
  void ResetEncoders();

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  double GetAverageEncoderDistance();

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  ctre::phoenix6::hardware::CANcoder &GetLeftEncoder();

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  ctre::phoenix6::hardware::CANcoder &GetRightEncoder();

  ctre::phoenix6::hardware::Pigeon2 &GetPigeon();

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive
   * more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  void SetMaxOutput(double maxOutput);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  /**
   * Drives the bot with specified chassis speeds
   *
   * @param speeds Chassis speeds to drive
   */
  void DriveChassisSpeeds(frc::ChassisSpeeds speeds);

  /**
   * Returns the current chassis speeds of the robot.
   *
   * @return The current chassis speeds.
   */
  frc::ChassisSpeeds GetChassisSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  void visionUpdate();

  frc::Pose2d poseXY();
  void setPose(frc::Pose2d p);

private:
  // Declare our motor controllers, encoders, drivetrain, and gyro

  rev::CANSparkMax m_left1;
  rev::CANSparkMax m_left2;
  rev::CANSparkMax m_right1;
  rev::CANSparkMax m_right2;

  frc::DifferentialDrive m_drive{[&](double output) { m_left1.Set(output); },
                                 [&](double output) { m_right1.Set(output); }};

  ctre::phoenix6::hardware::CANcoder m_leftEncoder;

  ctre::phoenix6::hardware::CANcoder m_rightEncoder;

  ctre::phoenix6::hardware::Pigeon2 m_gyro;

  frc::DifferentialDriveOdometry m_odometry;
  frc::DifferentialDriveKinematics m_kinematics;

  frc::SimpleMotorFeedforward<units::meters> m_leftFeedforward;
  frc::SimpleMotorFeedforward<units::meters> m_rightFeedforward;
};
