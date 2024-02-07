// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace constants {
  constexpr double NominalVoltage = 12.0;

  namespace DriveConstants {
    constexpr int _driveRightFront = 1;
    constexpr int _driveRightFollower = 2;
    constexpr int _driveLeftFront = 3;
    constexpr int _driveLeftFollower = 4;
    constexpr int _launcherLeftFront = 5;
    constexpr int _launcherLeftFollower = 6;
    constexpr int _launcherRightFront = 7;
    constexpr int _launcherRightFollower = 8;
    constexpr int _intake = 9;
    constexpr int _climber = 10;
  }
  namespace OperatorConstants {

    inline constexpr int kDriverControllerPort = 0;

  } // namespace OperatorConstants
} // namespace constants
