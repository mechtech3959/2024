// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
inline constexpr int kLeftMotor1Port = 3;
inline constexpr int kLeftMotor2Port = 2;
inline constexpr int kRightMotor1Port = 5;
inline constexpr int kRightMotor2Port = 4;

inline constexpr int kLeftEncoderPorts[]{0, 1};
inline constexpr int kRightEncoderPorts[]{2, 3};
inline constexpr bool kLeftEncoderReversed = false;
inline constexpr bool kRightEncoderReversed = true;

inline constexpr auto kTrackwidth = 0.69_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;

inline constexpr int kEncoderCPR = 1024;
inline constexpr units::meter_t kWheelDiameter = 6_in;
inline constexpr double kRotationsPerMeter = 1.91370942525;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The Robot Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
inline constexpr auto ks = 0.22_V;
inline constexpr auto kv = 1.98 * 1_V * 1_s / 1_m;
inline constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
inline constexpr double kPDriveVel = 8.5;
} // namespace DriveConstants

namespace ShooterConstants {
inline constexpr double kAmpShootSpeed = 0.15;
inline constexpr double kSpeakerShootSpeed = 1;
inline constexpr int kLeftShooterMotorID = 15;
inline constexpr int kRightShooterMotorID = 16;
constexpr char kShooterCanBus[] = "rio";
} // namespace ShooterConstants
namespace AutoConstants {
inline constexpr auto kMaxSpeed = 3_mps;
inline constexpr auto kMaxAcceleration = 1_mps_sq;

// Reasonable baseline values for a RAMSETE follower in units of meters and
// seconds
inline constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
inline constexpr auto kRamseteZeta = 0.7 / 1_rad;
} // namespace AutoConstants

namespace OIConstants {
inline constexpr int kDriverControllerPort = 0;
} // namespace OIConstants
