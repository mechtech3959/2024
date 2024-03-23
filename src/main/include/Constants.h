#pragma once
#include <numbers>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

namespace constants {
// Set to 'CAN1' for CANivore, 'rio' for roboRIO
constexpr char canBus[] = "rio";
namespace drive {
constexpr int leftFrontMotorID = 2;
constexpr int leftRearMotorID = 3;
constexpr int rightFrontMotorID = 4;
constexpr int rightRearMotorID = 5;
constexpr int PigeonID = 32;
} // namespace drive

namespace shooter {
constexpr int motorOneID = 9;
constexpr int motorTwoID = 10;

constexpr double speakerShootSpeed = 1;
constexpr double ampShootSpeed = 0.29;
} // namespace shooter

namespace intake {
constexpr int pickupMotorID = 7;
constexpr int feedMotorID = 8;

constexpr double pickupMotorSpeed = 1;
constexpr double feedMotorSpeed = 1;
} // namespace intake
} // namespace constants

namespace ArmConstants {
inline constexpr int kMotorPort = 4;

inline constexpr double kP = 1;

// These are fake gains; in actuality these must be determined individually for
// each robot
inline constexpr auto kS = 1_V;
inline constexpr auto kG = 1_V;
inline constexpr auto kV = 0.5_V * 1_s / 1_rad;
inline constexpr auto kA = 0.1_V * 1_s * 1_s / 1_rad;

inline constexpr auto kMaxVelocity = 3_rad_per_s;
inline constexpr auto kMaxAcceleration = 10_rad / (1_s * 1_s);

inline constexpr int kEncoderPorts[]{4, 5};
inline constexpr int kEncoderPPR = 256;
inline constexpr auto kEncoderDistancePerPulse =
    2.0_rad * std::numbers::pi / kEncoderPPR;

// The offset of the arm from the horizontal in its neutral position,
// measured from the horizontal
inline constexpr auto kArmOffset = 0.5_rad;
} // namespace ArmConstants

namespace AutoConstants {
inline constexpr auto kAutoTimeoutSeconds = 12_s;
inline constexpr auto kAutoShootTimeSeconds = 7_s;
} // namespace AutoConstants

namespace OIConstants {
inline constexpr int kDriverControllerPort = 0;
} // namespace OIConstants
