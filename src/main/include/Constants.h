#pragma once

namespace constants {
  namespace drive {
    constexpr int leftFrontMotorID = 1;
    constexpr int leftRearMotorID = 2;
    constexpr int rightFrontMotorID = 3;
    constexpr int rightRearMotorID = 4;
  }

  namespace shooter {
    constexpr int shooterMotorOneID = 5;
    constexpr int shooterMotorTwoID = 6;
    // constexpr int shooterMotorThreeID;
    // constexpr int shooterMotorFourID;
    constexpr double speakerShootSpeed = 1;
    constexpr double ampShootSpeed = 0.5;
    } // namespace shooter

  namespace intake {
    constexpr int intakeFrontMotorID = 7;
    constexpr int intakeRearMotorID = 8;
    constexpr double frontIntakeSpeed = 1;
    constexpr double rearIntakeSpeed = 1;
  }
} // namespace constants
