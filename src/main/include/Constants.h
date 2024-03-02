#pragma once

namespace constants {
namespace drive {
constexpr int leftFrontMotorID = 1;
constexpr int leftRearMotorID = 2;
constexpr int rightFrontMotorID = 3;
constexpr int rightRearMotorID = 4;
} // namespace drive

namespace shooter {
constexpr int shooterMotorOneID = 9;
constexpr int shooterMotorTwoID = 10;

// Change the variables below to adjust shoot speeds
// speakershootSpeed changes the speed of pressing the right trigger and
// pressing the left bumper
// ampShootSpeed changes the speed of pressing the left trigger
constexpr double speakerShootSpeed = 0.3;
constexpr double ampShootSpeed = 0.1;
} // namespace shooter

namespace intake {
constexpr int intakeFrontMotorID = 7;
constexpr int intakeRearMotorID = 8;
constexpr double frontIntakeSpeed = 1;
constexpr double rearIntakeSpeed = 1;
} // namespace intake
} // namespace constants
