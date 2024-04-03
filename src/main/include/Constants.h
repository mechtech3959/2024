#pragma once
#include <units/length.h>

namespace constants {
// Set to 'CAN1' for CANivore, 'rio' for roboRIO
constexpr char canBus[] = "rio";
namespace drive {
constexpr int leftFrontMotorID = 2;
constexpr int leftRearMotorID = 3;
constexpr int rightFrontMotorID = 4;
constexpr int rightRearMotorID = 5;
constexpr int PigeonID = 32;
constexpr int m_leftEncoderID = 31;  // set once attached
constexpr int m_rightEncoderID = 30; // set once attached
constexpr units::inch_t kWidth =
    26.935_in; // remeasure needs to be wheel to wheel width
constexpr double rotperIn = 0.0740288628; // rotation per inch
constexpr double rotperM = 1.91370942525; // rotation per inch

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
