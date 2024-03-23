#pragma once

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
