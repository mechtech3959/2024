// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

Robot::Robot() {}

void Robot::RobotInit() {

  m_autoChooser.SetDefaultOption(a_Left, AutoRoutine::kLeft);
  m_autoChooser.AddOption(a_Middle, AutoRoutine::kMiddle);
  m_autoChooser.AddOption(a_Right, AutoRoutine::kRight);
  m_autoChooser.AddOption(a_Test, AutoRoutine::kTest);
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

  m_swerve.SetHeading(0_deg);
  headingControl = true;
  driveMode = DriveMode::HeadingControl;

  autoState = 0;
  autoTimer.Reset();

  m_swerve.ResetDriveEncoders();

  // m_arm.Init();
  // m_slide.Init();
  // m_elevator.Init();
  // m_claw.ClawClose();
  // m_claw.SetIntakeSpeed(constants::clawConstants::HoldSpeed);

  // m_compressor.EnableAnalog(115.0_psi, 120.0_psi);
}

void Robot::UpdatePose() { m_swerve.UpdateOdometry(); }

void Robot::RobotPeriodic() {

  UpdatePose();

  frc::Pose2d bp = m_swerve.GetPose();
  frc::SmartDashboard::PutNumber("Bot Pose X", units::inch_t(bp.X()).value());
  frc::SmartDashboard::PutNumber("Bot Pose Y", units::inch_t(bp.Y()).value());
  frc::SmartDashboard::PutNumber("Bot Pose Heading",
                                 bp.Rotation().Degrees().value());

  m_swerve.SendData();
}

// void Robot::GenLeft() {

//   if (frc::DriverStation::GetAlliance() ==
//       frc::DriverStation::Alliance::kBlue) {
//     m_swerve.SetPose(
//         {waypoints::Blue6Right.Translation(), frc::Rotation2d(180_deg)});
//   } else {
//     m_swerve.SetPose(
//         {waypoints::Red3Left.Translation(), frc::Rotation2d(0_deg)});
//   }

//   // VERY IMPORTANT!!!  Make sure the initial angle of the
//   // start pose has the heading you wish to drive.  Failure
//   // to do so will result in a small movement in the heading
//   // of the initial pose to allow for a differential robot
//   // to turn.  This is not needed for a swerve bot.

//   units::degree_t heading;
//   if (frc::DriverStation::GetAlliance() ==
//       frc::DriverStation::Alliance::kBlue) {
//     heading = 0_deg;
//     frc::Pose2d x = {m_swerve.GetPose().Translation(), heading};

//     // create config for traj generation
//     waypoints::WaypointPoses c{};

//     // create waypoint list from scoring position to piece pickup position

//   } else {
//     heading = 179.9_deg;
//     frc::Pose2d x = {m_swerve.GetPose().Translation(), heading};

//     // create config for traj generation
//     waypoints::WaypointPoses c{};

//     // create waypoint list from scoring position to piece pickup position
//   }
// }

// void Robot::RunLeft() {
//   frc::Pose2d p; // goal pose
//   // frc::Pose2d rp = m_swerve.GetPose(); //Current Pose
//   units::degree_t heading; // Goal heading

//   // init Heading just incase we forget.
//   if (frc::DriverStation::GetAlliance() ==
//       frc::DriverStation::Alliance::kBlue) {
//     heading = 179.9_deg;
//   } else {
//     heading = 0_deg;
//   }

//   switch (autoState) {
//   case 0:
//     autoState++;       // Start timer for indexing into trajectory
//     autoTimer.Reset(); // reset timer
//     break;
//   case 1:
//     if (autoTimer.Get() < 1.8_s) {
//       break;
//     } else if (autoTimer.Get() < 1.9_s) {
//       // m_claw.ClawOpen();
//       break;
//     } else if (autoTimer.Get() < 2_s) {
//       // UpTuckPos();
//       break;
//     } else if (autoTimer.Get() < 2.5_s) {
//       break;
//     } else {
//       autoState++;       // Start timer for indexing into trajectory
//       autoTimer.Reset(); // reset timer
//       m_swerve.SetRotationGain(.5);
//       break;
//     }
//   case 2:
//     p = traj2Piece1.Sample(autoTimer.Get()).pose;
//     if (m_swerve.GetPose().X() > waypoints::BlueCorridorNear.X() &&
//         m_swerve.GetPose().X() < waypoints::RedCorridorNear.X()) {
//       if (frc::DriverStation::GetAlliance() ==
//           frc::DriverStation::Alliance::kBlue) {
//         heading = -30_deg;
//       } else {
//         heading = -150_deg;
//       }
//       // If within 5 degrees of finishing turn, go to ground pos for pickup
//       if (fabs(m_swerve.GetTargetHeading().value() -
//                m_swerve.GetHeading().Degrees().value()) < 5.0) {
//         // GroundPos();
//       }
//     } else {
//       if (frc::DriverStation::GetAlliance() ==
//           frc::DriverStation::Alliance::kBlue) {
//         heading = 179.9_deg;
//       } else {
//         heading = 0_deg;
//       }
//     }
//     m_swerve.DrivePos(p.X(), p.Y(), heading);

//     if (autoTimer.Get() > (traj2Piece1.TotalTime())) {
//       autoState++;
//       autoTimer.Reset();
//       m_swerve.SetRotationGain(constants::swerveConstants::RotGain); // Set back
//     }
//     break;

//   default:
//     m_swerve.DriveXY(0_mps, 0_mps);
//   }
// }

// void Robot::RunMiddle() {}

// void Robot::GenMiddle() {}

// void Robot::RunRight() {}

// void Robot::GenRight() {}

// void Robot::GenSimpleSwitch() {
//   // if (!FuseLL()) {
//   if (frc::DriverStation::GetAlliance() ==
//       frc::DriverStation::Alliance::kBlue) {
//     m_swerve.SetPose(
//         {waypoints::Blue7Right.Translation(), frc::Rotation2d(179.9_deg)});
//   } else {
//     m_swerve.SetPose(
//         {waypoints::Red2Left.Translation(), frc::Rotation2d(0_deg)});
//     // }
//   }

  // VERY IMPORTANT!!!  Make sure the initial angle of the
  // start pose has the heading you wish to drive.  Failure
  // to do so will result in a small movement in the heading
  // of the initial pose to allow for a differential robot
  // to turn.  This is not needed for a swerve bot.

//   units::degree_t heading;
//   if (frc::DriverStation::GetAlliance() ==
//       frc::DriverStation::Alliance::kBlue) {
//     heading = 0_deg;
//     frc::Pose2d x = {m_swerve.GetPose().Translation(), heading};

//     // create config for traj generation
//     waypoints::WaypointPoses c{};

//     // create waypoint list from scoring position to piece pickup position
//     std::vector<frc::Translation2d> wp2Piece1{waypoints::BlueSwitchNear,
//                                               waypoints::BlueSwitchFar};

//     // create traj from score position 1 to pickup position 1
//     frc::Pose2d p1 = frc::Pose2d(waypoints::BlueSwitchFar.X() + 24_in,
//                                  waypoints::BlueSwitchFar.Y() - 0_in,
//                                  frc::Rotation2d(179.9_deg));
//     traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(x, wp2Piece1, p1,
//                                                                c.config);

//     // create waypoint list from scoring position to piece pickup position
//     std::vector<frc::Translation2d> wp2Switch{waypoints::BlueSwitchFar};

//     // create traj from score position 1 to pickup position 1
//     traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(
//         p1, wp2Switch,
//         frc::Pose2d(waypoints::BlueSwitch.X() + 0_in, waypoints::BlueSwitch.Y(),
//                     frc::Rotation2d(0_deg)),
//         c.config);

//   } else {
//     heading = 179.9_deg;
//     frc::Pose2d x = {m_swerve.GetPose().Translation(), heading};

//     // create config for traj generation
//     waypoints::WaypointPoses c{};

//     // create waypoint list from scoring position to piece pickup position
//     std::vector<frc::Translation2d> wp2Piece1{waypoints::RedSwitchNear,
//                                               waypoints::RedSwitchFar};

//     // create traj from score position 1 to pickup position 1
//     frc::Pose2d p1 =
//         frc::Pose2d(waypoints::RedSwitchFar.X() - 24_in,
//                     waypoints::RedSwitchFar.Y() - 0_in, frc::Rotation2d(0_deg));
//     traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(x, wp2Piece1, p1,
//                                                                c.config);

//     // create waypoint list from scoring position to piece pickup position
//     std::vector<frc::Translation2d> wp2Switch{waypoints::RedSwitchFar};

//     // create traj from score position 1 to pickup position 1
//     traj2Score2 = frc::TrajectoryGenerator::GenerateTrajectory(
//         p1, wp2Switch,
//         frc::Pose2d(waypoints::RedSwitch.X() - 0_in, waypoints::RedSwitch.Y(),
//                     frc::Rotation2d(0_deg)),
//         c.config);
//   }
// }

// void Robot::RunSimpleSwitch() {
//   frc::Pose2d p; // goal pose
//   // frc::Pose2d rp = m_swerve.GetPose(); //Current Pose
//   units::degree_t heading; // Goal heading

//   // init Heading just incase we forget.
//   if (frc::DriverStation::GetAlliance() ==
//       frc::DriverStation::Alliance::kBlue) {
//     heading = 179.9_deg;
//   } else {
//     heading = 0_deg;
//   }

//   // FuseLLNoHeading();

//   switch (autoState) {
//   case 0:
//     autoState++;       // Start timer for indexing into trajectory
//     autoTimer.Reset(); // reset timer
//     // m_slide.SetPosition(0_in);
//     // MidPos();
//     break;
//   case 1:
//     if (autoTimer.Get() < 1.8_s) {
//       break;
//     } else if (autoTimer.Get() < 1.9_s) {
//       // m_claw.ClawOpen();
//       break;
//     } else if (autoTimer.Get() < 2_s) {
//       // UpTuckPos();
//       break;
//     } else if (autoTimer.Get() < 2.5_s) {
//       break;
//     } else {
//       autoState++; // Start timer for indexing into trajectory
//       autoTimer.Reset();
//       break; // reset timer
//       // m_claw.ClawClose();
//     }
//   case 2:
//     p = traj2Piece1.Sample(autoTimer.Get()).pose;
//     if (frc::DriverStation::GetAlliance() ==
//         frc::DriverStation::Alliance::kBlue) {
//       heading = 179.9_deg;
//     } else {
//       heading = 0_deg;
//     }
//     m_swerve.DrivePos(p.X(), p.Y(), heading);

//     // PUT ARM OUT COMING OFF THE SWITCH TO AVOID TIPPING
//     if (m_swerve.GetPose().X() > (waypoints::BlueSwitch.X()) &&
//         m_swerve.GetPose().X() < (waypoints::RedSwitch.X())) {
//       // m_arm.SetAngle(-10_deg);
//     }

//     if (autoTimer.Get() > (traj2Piece1.TotalTime())) {
//       autoState++;
//       autoTimer.Reset();
//       m_swerve.DriveXY(0_mps, 0_mps);
//       // m_arm.SetAngle(-10_deg);
//     }
//     break;
//   case 3:
//     m_swerve.DriveXY(0_mps, 0_mps);
//     // if(autoTimer.Get()>.5_s){
//     autoState++;
//     autoTimer.Reset();
//     //}
//     break;
//   case 4:
//     p = traj2Score2.Sample(autoTimer.Get()).pose;
//     if (frc::DriverStation::GetAlliance() ==
//         frc::DriverStation::Alliance::kBlue) {
//       heading = 179.9_deg;
//     } else {
//       heading = 0_deg;
//     }
//     m_swerve.DrivePos(p.X(), p.Y(), heading);

//     if (autoTimer.Get() > (traj2Score2.TotalTime() + .25_s)) {
//       autoState++;
//       autoTimer.Reset();
//       m_swerve.DriveXY(0_mps, 0_mps);
//     }
//     break;
//   case 5:
//     m_swerve.DriveXY(0_mps, 0_mps);
//     if (autoTimer.Get() > .25_s) {
//       autoState++;
//       autoTimer.Reset();
//     }
//     break;
//   case 6: // do balance stuff
//     if (m_swerve.GetPitch().value() < -0.5) {
//       // m_arm.SetAngle(-70_deg);
//       // m_slide.SetPosition(m_slide.GetPosition() + 1_in);
//     } else {
//       if (m_swerve.GetPitch().value() > 0.5) {
//         // UpTuckPos();
//         autoState = 5;
//       }
//     }
//     m_swerve.DriveXY(0_mps, 0_mps);
//     break;
//   default:
//     m_swerve.DriveXY(0_mps, 0_mps);
//     break;
//   }
// }

// void Robot::GenSpeedBump() {
//   // if (!FuseLL()) {
//   if (frc::DriverStation::GetAlliance() ==
//       frc::DriverStation::Alliance::kBlue) {
//     m_swerve.SetPose(
//         {waypoints::Blue8Left.Translation(), frc::Rotation2d(180_deg)});
//   } else {
//     m_swerve.SetPose(
//         {waypoints::Red1Right.Translation(), frc::Rotation2d(0_deg)});
//   }
//   // }

//   // VERY IMPORTANT!!!  Make sure the initial angle of the
//   // start pose has the heading you wish to drive.  Failure
//   // to do so will result in a small movement in the heading
//   // of the initial pose to allow for a differential robot
//   // to turn.  This is not needed for a swerve bot.

//   units::degree_t heading;
//   if (frc::DriverStation::GetAlliance() ==
//       frc::DriverStation::Alliance::kBlue) {
//     heading = 0_deg;
//     frc::Pose2d x = {m_swerve.GetPose().Translation(), heading};

//     // create config for traj generation
//     waypoints::WaypointPoses c{};

//     // create waypoint list from scoring position to piece pickup position
//     std::vector<frc::Translation2d> wp2Piece1{waypoints::BlueBumpNear,
//                                               waypoints::BlueBumpFar};

//     // create traj from score position 1 to pickup position 1
//     traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(
//         x, wp2Piece1,
//         frc::Pose2d(waypoints::BluePiece4.X() - 40_in,
//                     waypoints::BluePiece4.Y() - 8_in, frc::Rotation2d(0_deg)),
//         c.config);

//   } else {
//     heading = 179.9_deg;
//     frc::Pose2d x = {m_swerve.GetPose().Translation(), heading};

//     // create config for traj generation
//     waypoints::WaypointPoses c{};

//     // create waypoint list from scoring position to piece pickup position
//     std::vector<frc::Translation2d> wp2Piece1{waypoints::RedBumpNear,
//                                               waypoints::RedBumpFar};

//     // create traj from score position 1 to pickup position 1
//     traj2Piece1 = frc::TrajectoryGenerator::GenerateTrajectory(
//         x, wp2Piece1,
//         frc::Pose2d(waypoints::RedPiece4.X() + 40_in,
//                     waypoints::RedPiece4.Y() - 8_in, frc::Rotation2d(0_deg)),
//         c.config);
//   }
// }

// void Robot::RunSpeedBump() {
//   frc::Pose2d p; // goal pose
//   // frc::Pose2d rp = m_swerve.GetPose(); //Current Pose
//   units::degree_t heading; // Goal heading

//   // init Heading just incase we forget.
//   if (frc::DriverStation::GetAlliance() ==
//       frc::DriverStation::Alliance::kBlue) {
//     heading = 179.9_deg;
//   } else {
//     heading = 0_deg;
//   }

//   // FuseLLNoHeading();

//   switch (autoState) {
//   case 0:
//     autoState++;       // Start timer for indexing into trajectory
//     autoTimer.Reset(); // reset timer
//     // m_slide.SetPosition(0_in);
//     // MidPos();
//     break;
//   case 1:
//     if (autoTimer.Get() < 1.8_s) {
//       break;
//     } else if (autoTimer.Get() < 1.9_s) {
//       // m_claw.ClawOpen();
//       break;
//     } else if (autoTimer.Get() < 2_s) {
//       // UpTuckPos();
//       break;
//     } else if (autoTimer.Get() < 2.5_s) {
//       break;
//     } else {
//       autoState++;       // Start timer for indexing into trajectory
//       autoTimer.Reset(); // reset timer
//       m_swerve.SetRotationGain(.5);
//       break;
//     }
//   case 2:
//     p = traj2Piece1.Sample(autoTimer.Get()).pose;
//     if (m_swerve.GetPose().X() > (waypoints::BlueSwitch.X() + 18_in) &&
//         m_swerve.GetPose().X() < (waypoints::RedSwitch.X() - 18_in)) {
//       if (frc::DriverStation::GetAlliance() ==
//           frc::DriverStation::Alliance::kBlue) {
//         heading = 10_deg;
//       } else {
//         heading = 170_deg;
//       }
//       // If within 5 degrees of finishing turn, go to ground pos for pickup
//       if (fabs(m_swerve.GetTargetHeading().value() -
//                m_swerve.GetHeading().Degrees().value()) < 5.0) {
//         // GroundPos();
//       }
//     } else {
//       if (frc::DriverStation::GetAlliance() ==
//           frc::DriverStation::Alliance::kBlue) {
//         heading = 179.9_deg;
//       } else {
//         heading = 0_deg;
//       }
//     }
//     m_swerve.DrivePos(p.X(), p.Y(), heading);

//     if (autoTimer.Get() > (traj2Piece1.TotalTime())) {
//       autoState++;
//       autoTimer.Reset();
//       m_swerve.SetRotationGain(constants::swerveConstants::RotGain); // Set back
//     }
//     break;

//   default:
//     m_swerve.DriveXY(0_mps, 0_mps);
//   }
// }

// void Robot::GenTest() {}

// void Robot::RunTest() {}

// void Robot::GenTraj() {
//   m_autoSelected = m_autoChooser.GetSelected();

//   switch (m_autoSelected) {
//   case AutoRoutine::kLeft:
//     GenLeft();
//     break;
//   case AutoRoutine::kMiddle:
//     GenMiddle();
//     break;
//   case AutoRoutine::kRight:
//     GenRight();
//     break;
//   case AutoRoutine::kTest:
//     GenTest();
//     break;
//   }
// }

// void Robot::TrackToGoal(frc::Pose2d goal) {
//   /*if (m_leftLL.IsTargetVisible()) {
//     if ((units::math::abs(m_swerve.GetChassisSpeeds().vx) +
//          units::math::abs(m_swerve.GetChassisSpeeds().vy)) < .05_mps) {
//       m_swerve.SetPose(m_leftLL.GetRobotPose());
//     }

//     m_swerve.DrivePos(goal.X(), goal.Y(), goal.Rotation().Degrees());
//   }*/
// }

void Robot::AutonomousInit() {
  autoTimer.Reset();
  autoTimer.Start();
  autoState = 0;
  // m_claw.ClawClose();
  m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees());
  units::degree_t heading;
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    heading = 179.9_deg;
  } else {
    heading = 0_deg;
  }
  m_swerve.SetHeading(heading);
  m_swerve.SetTargetHeading(heading);
  // GenTraj();

  // remove before flight!!!
  autoState = 5;
  // TuckPos();
}

void Robot::AutonomousPeriodic() {

  switch (m_autoSelected) {
  case AutoRoutine::kLeft:
    // RunLeft();
    ;
    break;
  case AutoRoutine::kMiddle:
    // RunMiddle();
    break;
  case AutoRoutine::kRight:
    // RunRight();
    break;
  case AutoRoutine::kTest:
    // RunTest();
    break;
  }

  frc::SmartDashboard::PutNumber("Auto State", autoState);
  frc::SmartDashboard::PutNumber("Auto Timer", autoTimer.Get().value());
}

void Robot::TeleopInit() {

  // Set current heading as target heading to avoid unwanted motion
  m_swerve.SetTargetHeading(m_swerve.GetHeading().Degrees());

  m_swerve.SetRotationGain(constants::swerveConstants::RotGain);
  // m_claw.ClawClose();
}

/*
  Driver Back Button Resets Drive Pose
*/
void Robot::TeleopPeriodic() {

  m_swerve.SendData();

  // Reset Drive Pose
  if (driver.GetBackButtonPressed()) {
    m_swerve.SetPose(frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)));
  }

  if (codriver.GetAButton()) {
    // m_claw.ClawOpen();
  } else {
    // m_claw.ClawClose();
  }

  if (codriver.GetBButton()) {
    // m_claw.ClawClose();
  }

  if (codriver.GetRightTriggerAxis() > 0.0) {
    // m_claw.SetIntakeSpeed(codriver.GetRightTriggerAxis() * .2); // shoot
  } else if (codriver.GetLeftTriggerAxis() > 0.0) {
    // m_claw.SetIntakeSpeed(-codriver.GetLeftTriggerAxis() * .25); // feed
  } /*else {
      }*/

  Drive();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

/*
Driver Control Function

Left Driver Joystick is heading/rotation
Right Driver Joystick is translation in x/y

Driver A Button is Heading Control, points in direction left joystick is
pointing Driver B Button is Velocity Control, just turns based on Left X axis

Driver Right Trigger Scales drive speed

*/
void Robot::Drive() {
  double drivex = -driver.GetLeftY();
  double drivey = -driver.GetLeftX();
  // deadband
  if ((fabs(drivex) + fabs(drivey)) / 2.0 < .1) {
    drivex = 0.0;
    drivey = 0.0;
  }

  // Get Right joystick for heading and velocity control
  double hy = -driver.GetRightX(); // hy is w for vel control
  double hx = -driver.GetRightY();

  // Scale Speed
  units::scalar_t scale = 1.0 - driver.GetRightTriggerAxis() * .9;
  scale = scale * .8;

  // Set control mode
  if (driver.GetAButtonPressed()) {
    driveMode = DriveMode::HeadingControl;
  }

  if (driver.GetBButtonPressed()) {
    driveMode = DriveMode::VelocityMode;
  }

  switch (driveMode) {
  case DriveMode::VelocityMode:
    if (fabs(hy) < .1) { // deadband rot vel, translations were done
      hy = 0.0;
    }
    m_swerve.Drive(m_xspeedLimiter.Calculate(pow(drivex, 3)) *
                       constants::swerveConstants::MaxSpeed * scale,
                   m_yspeedLimiter.Calculate(pow(drivey, 3)) *
                       constants::swerveConstants::MaxSpeed * scale,
                   m_rotLimiter.Calculate(pow(hy, 3)) *
                       constants::swerveConstants::MaxAngularVelocity * scale,
                   true);
    break;
  case DriveMode::HeadingControl:
    if (sqrt(hx * hx + hy * hy) >
        0.9) // make sure the joystick is begin used by calculating magnitude
    {
      m_swerve.SetTargetHeading(frc::Rotation2d(hx, hy).Degrees());
      frc::SmartDashboard::PutNumber("Heading Stick Value",
                                     sqrt(hx * hx + hy * hy));
    }
    m_swerve.DriveXY(m_xspeedLimiter.Calculate(pow(drivex, 1)) *
                         constants::swerveConstants::MaxSpeed * scale,
                     m_yspeedLimiter.Calculate(pow(drivey, 1)) *
                         constants::swerveConstants::MaxSpeed * scale);
    break;
  // case DriveMode::TargetTracking:
  //   if (driver.GetAButton()) {
  //     if (frc::DriverStation::GetAlliance() ==
  //         frc::DriverStation::Alliance::kBlue) {
  //       TrackToGoal(
  //           frc::Pose2d(waypoints::BlueSwitch, frc::Rotation2d(180_deg)));
  //     } else {
  //       TrackToGoal(frc::Pose2d(waypoints::RedSwitch, frc::Rotation2d(0_deg)));
  //     }
  //   }
  //   break;
  default:
    break;
  }
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
