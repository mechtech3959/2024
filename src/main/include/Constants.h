#pragma once

namespace constants {
constexpr bool competitionMode = false;
// Set to 'CAN1' for CANivore, 'rio' for roboRIO
constexpr char canBus[] = "rio";
namespace drive {
constexpr int leftFrontMotorID = 2;
constexpr int leftRearMotorID = 3;
constexpr int rightFrontMotorID = 4;
constexpr int rightRearMotorID = 5;
constexpr int PigeonID = 32;
constexpr double defaultCurrentLimit = 30;
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
constexpr double defaultFeedMotorCurrentLimit = 20;
constexpr double defaultPickupMotorCurrentLimit = 20;

constexpr double pickupMotorSpeed = 1;
constexpr double feedMotorSpeed = 1;
} // namespace intake
namespace climber {
constexpr int motorID = 33;

constexpr double speed = .25;
} // namespace climber
} // namespace constants
