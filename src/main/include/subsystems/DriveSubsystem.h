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

  ctre::phoenix6::hardware::Pigeon2 &GetPigeon();

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive
   * more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  void SetMaxOutput(double maxOutput);

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
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  void visionUpdate();

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
