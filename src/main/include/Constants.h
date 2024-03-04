#pragma once

namespace constants {
namespace drive {
constexpr int leftFrontMotorID = 1;
constexpr int leftRearMotorID = 2;
constexpr int rightFrontMotorID = 3;
constexpr int rightRearMotorID = 4;
} // namespace drive

namespace shooter {
constexpr int motorOneID = 9;
constexpr int motorTwoID = 10;

constexpr double speakerShootSpeed = 1;
constexpr double ampShootSpeed = 0.5;
} // namespace shooter

namespace intake {
constexpr int pickupMotorID = 7;
constexpr int feedMotorID = 8;

constexpr double pickupMotorSpeed = 1;
constexpr double feedMotorSpeed = 1;
} // namespace intake
} // namespace constants
