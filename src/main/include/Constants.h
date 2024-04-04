// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <numbers>

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

namespace constants {
namespace drive {
inline constexpr int kLeftMotor1Port = 2;
inline constexpr int kLeftMotor2Port = 3;
inline constexpr int kRightMotor1Port = 4;
inline constexpr int kRightMotor2Port = 5;

inline constexpr int kLeftEncoderPorts = 31;
inline constexpr int kRightEncoderPorts = 30;
inline constexpr bool kLeftEncoderReversed = false;
inline constexpr bool kRightEncoderReversed = true;

inline constexpr int kEncoderCpr = 1024;
inline constexpr units::meter_t kWheelDiameter = 6_in;
inline constexpr double kRotPerM = 1.91370942525;
inline constexpr units::meter_t kEncoderDistancePerPulse =
    (kWheelDiameter * std::numbers::pi) / static_cast<double>(kEncoderCpr);
} // namespace drive

namespace intake {
inline constexpr int kMotorPort = 6;
inline constexpr std::array<int, 2> kSolenoidPorts = {2, 3};
} // namespace intake

namespace storage {
inline constexpr int kMotorPort = 7;
inline constexpr int kBallSensorPort = 6;
} // namespace storage

namespace autonomous {
inline constexpr units::second_t kTimeout = 3_s;
inline constexpr units::meter_t kDriveDistance = 2_m;
inline constexpr double kDriveSpeed = 0.5;
} // namespace autonomous

namespace oi {
inline constexpr int kDriverControllerPort = 0;
} // namespace oi
} // namespace constants
