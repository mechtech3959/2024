#include "Robot.h"

// Set the CANdle LEDs to white
void Robot::White() { candle.SetLEDs(255, 255, 255); }
void Robot::Green() { candle.SetLEDs(0, 255, 0); }
void Robot::Red() { candle.SetLEDs(255, 0, 0); }
void Robot::Yellow() { candle.SetLEDs(255, 255, 0); }
void Robot::Blue() { candle.SetLEDs(0, 0, 255); }
void Robot::Rainbow() {
  // For every pixel
  m_toAnimate = new ctre::phoenix::led::RainbowAnimation(1, 0.1, 150);
  candle.Animate(*m_toAnimate);
}

void Robot::ampAuto() { // zac did this :)
  limelight.updateTracking();
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 3.0_s) {
      diffDrive.ArcadeDrive(
          0.6, limelight.m_LimelightTurnCmd); // drive towards apriltag
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 6.0_s) {
      ShootAmp(); // shoot
    } else if (autoTimer.Get() > 6.0_s && autoTimer.Get() < 8_s) {
      diffDrive.ArcadeDrive(-0.75, -0.5); // turn to note and intake it
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 8.0_s && autoTimer.Get() < 10.5_s) {
      diffDrive.ArcadeDrive(0.65, 0.48); // turn to amp
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 10.5_s && autoTimer.Get() < 15_s) {
      ShootAmp(); // score
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) { // red
    if (autoTimer.Get() < 3.0_s) {
      diffDrive.ArcadeDrive(0.6, limelight.m_LimelightTurnCmd);
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 6.0_s) {
      ShootAmp();
    } else if (autoTimer.Get() > 6.0_s && autoTimer.Get() < 8_s) {
      diffDrive.ArcadeDrive(-0.65, 0.48);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 8.0_s && autoTimer.Get() < 10.5_s) {
      diffDrive.ArcadeDrive(0.65, -0.48);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 10.5_s && autoTimer.Get() < 15_s) {
      ShootAmp();
    }
  } else {
    diffDrive.ArcadeDrive(0.0, 0.0);
  }
}

void Robot::middleAuto() { // callie did this :D
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 4.5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 4.5_s && autoTimer.Get() < 7.0_s) {
      shooterMotorLeader.Set(0);
      diffDrive.ArcadeDrive(-0.6, 0.0);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 7.0_s && autoTimer.Get() < 9.5_s) {
      diffDrive.ArcadeDrive(0.6, 0.0);
    } else if (autoTimer.Get() > 9.5_s && autoTimer.Get() < 12.5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 12.5_s && autoTimer.Get() < 15.0_s) {
      shooterMotorLeader.Set(0);
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 4.5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 4.5_s && autoTimer.Get() < 7.0_s) {
      shooterMotorLeader.Set(0);
      diffDrive.ArcadeDrive(-0.6, 0.0);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 7.0_s && autoTimer.Get() < 9.5_s) {
      diffDrive.ArcadeDrive(0.6, 0.0);
    } else if (autoTimer.Get() > 9.5_s && autoTimer.Get() < 12.5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 13.0_s && autoTimer.Get() < 15.0_s) {
      shooterMotorLeader.Set(0);
    }
  } else {
    diffDrive.ArcadeDrive(0, 0);
  }
}

void Robot::middle3PcAuto() { // callie did this :D
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 2.0_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 2.0_s && autoTimer.Get() < 4.0_s) {
      diffDrive.ArcadeDrive(-0.6, 0.0);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 4.0_s && autoTimer.Get() < 6.5_s) {
      diffDrive.ArcadeDrive(0.6, 0.0);
    } else if (autoTimer.Get() > 6.5_s && autoTimer.Get() < 8.5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 8.5_s && autoTimer.Get() < 11_s) {
      diffDrive.ArcadeDrive(-0.65, 0.3);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 11_s && autoTimer.Get() < 13.5_s) {
      diffDrive.ArcadeDrive(0.65, -0.3);
    } else if (autoTimer.Get() > 13.5_s && autoTimer.Get() < 15_s) {
      ShootSpeaker();
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 2.0_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 2.0_s && autoTimer.Get() < 4.0_s) {
      diffDrive.ArcadeDrive(-0.6, 0.0);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 4.0_s && autoTimer.Get() < 6.5_s) {
      diffDrive.ArcadeDrive(0.6, 0.0);
    } else if (autoTimer.Get() > 6.5_s && autoTimer.Get() < 8.5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 8.5_s && autoTimer.Get() < 11_s) {
      diffDrive.ArcadeDrive(-0.65, -0.3);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 11_s && autoTimer.Get() < 13.5_s) {
      diffDrive.ArcadeDrive(0.65, 0.3);
    } else if (autoTimer.Get() > 13.5_s && autoTimer.Get() < 15_s) {
      ShootSpeaker();
    }
  } else {
    diffDrive.ArcadeDrive(0, 0);
  }
}

void Robot::sideAuto() { // Jacob did this ;/
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 3.0_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 5.1_s) {
      diffDrive.ArcadeDrive(-0.8, -0.42);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 5.1_s && autoTimer.Get() < 5.6_s) {
      diffDrive.ArcadeDrive(0, 0);
    } else if (autoTimer.Get() > 5.6_s && autoTimer.Get() < 7.7_s) {
      diffDrive.ArcadeDrive(0.8, 0.42);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 8.7_s && autoTimer.Get() < 11.0_s) {
      ShootSpeaker();
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 3.0_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 5.1_s) {
      diffDrive.ArcadeDrive(-0.8, 0.42);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 5.1_s && autoTimer.Get() < 5.6_s) {
      diffDrive.ArcadeDrive(0.0, 0.0);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 5.6_s && autoTimer.Get() < 7.7_s) {
      diffDrive.ArcadeDrive(0.8, -0.42);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 8.7_s && autoTimer.Get() < 11.0_s) {
      ShootSpeaker();
    }
  }
}

void Robot::driveoutAuto() { // ZAC
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 3.0_s) {
      ShootSpeaker();
    } /* else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 5.0_s) {
       diffDrive.ArcadeDrive(-0.7, 0.00);
       intake.SetSpeed(1);
     } else if (autoTimer.Get() > 5.0_s && autoTimer.Get() < 15.0_s) {
       diffDrive.ArcadeDrive(0.00, 0.00);
       intake.Stop();
     }*/
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 3.0_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 6.0_s) {
      diffDrive.ArcadeDrive(-0.5, 0.00);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 6.0_s && autoTimer.Get() < 15.0_s) {
      diffDrive.ArcadeDrive(0.00, 0.00);
      intake.Stop();
    }
  }
}

void Robot::wallOnlySideAuto() { // drive out only
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 5_s) {
      diffDrive.ArcadeDrive(-0.7, -0.6);
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 5_s) {
      diffDrive.ArcadeDrive(-0.7, 0.6);
    }
  }
}

void Robot::wallShootSideAuto() { // shoot and drive out only
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 5.0_s && autoTimer.Get() < 8.0_s) {
      diffDrive.ArcadeDrive(-0.7, -0.6);
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 5.0_s && autoTimer.Get() < 8.0_s) {
      diffDrive.ArcadeDrive(-0.7, 0.6);
    }
  }
}

void Robot::wallOnlyAmpAuto() {
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 5_s) {
      diffDrive.ArcadeDrive(-0.7, 0.6);
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 5_s) {
      diffDrive.ArcadeDrive(-0.7, -0.6);
    }
  }
}

void Robot::wallShootAmpAuto() {
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 5.0_s && autoTimer.Get() < 8.0_s) {
      diffDrive.ArcadeDrive(-0.7, 0.6);
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 5_s) {
      ShootSpeaker();
    } else if (autoTimer.Get() > 5.0_s && autoTimer.Get() < 8.0_s) {
      diffDrive.ArcadeDrive(-0.7, -0.6);
    }
  }
}

void Robot::testAuto() {
  double Yaw = Pigeon.GetYaw().GetValueAsDouble();
  double ta = LimelightHelpers::getTA("limelight-greenie");
  limelight.updateTracking();
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 2.0_s) {
      diffDrive.ArcadeDrive(0.6, limelight.m_LimelightTurnCmd);
    } else if (autoTimer.Get() > 2.0_s && autoTimer.Get() < 2.5_s) {
      ShootAmp();
      Pigeon.SetYaw(0_deg);
    } else if (autoTimer.Get() > 2.5_s && autoTimer.Get() < 3.5_s) {
      shooterMotorLeader.Set(0);
      if ((ta < 1) or (ta = 0)) {
        diffDrive.ArcadeDrive(-0.5, 0);
      }
    } else if (autoTimer.Get() > 3.5_s && autoTimer.Get() < 6.5_s) {
      if (Yaw < 90) {
        diffDrive.ArcadeDrive(0, -0.8);
        diffDrive.ArcadeDrive(0, -0.1);
        diffDrive.ArcadeDrive(0, -0.8);
      }
    } else if (autoTimer.Get() > 6.0_s && autoTimer.Get() < 7.5_s) {
      intake.SetSpeed(1);
      diffDrive.ArcadeDrive(-0.6, 0);
    } else if (autoTimer.Get() > 7.5_s && autoTimer.Get() < 9.0_s) {
      diffDrive.ArcadeDrive(0.5, -0.1);
    } else if (autoTimer.Get() > 9_s && autoTimer.Get() < 12_s) {
      if (Yaw > 0) {
        diffDrive.ArcadeDrive(0, 0.8);
        diffDrive.ArcadeDrive(0, 0);
        diffDrive.ArcadeDrive(0, 0.7);
      }
    } else if (autoTimer.Get() > 12_s && autoTimer.Get() < 13_s) {
      diffDrive.ArcadeDrive(0.6, limelight.m_LimelightTurnCmd);
    } else if (autoTimer.Get() > 13_s && autoTimer.Get() < 14.5_s) {
      ShootAmp();
    } else if (autoTimer.Get() > 14.5_s && autoTimer.Get() < 15.0_s) {
      shooterMotorLeader.Set(0);
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 2.0_s) {
      diffDrive.ArcadeDrive(0.6, limelight.m_LimelightTurnCmd);
    } else if (autoTimer.Get() > 2.0_s && autoTimer.Get() < 2.5_s) {
      ShootAmp();
      Pigeon.SetYaw(0_deg);
    } else if (autoTimer.Get() > 2.5_s && autoTimer.Get() < 3.5_s) {
      shooterMotorLeader.Set(0);
      if ((ta < 1) or (ta = 0)) {
        diffDrive.ArcadeDrive(-0.5, 0);
      }
    } else if (autoTimer.Get() > 3.5_s && autoTimer.Get() < 6.5_s) {
      if (Yaw < 90) {
        diffDrive.ArcadeDrive(0, 0.8);
        diffDrive.ArcadeDrive(0, 0.1);
        diffDrive.ArcadeDrive(0, 0.8);
      }
    } else if (autoTimer.Get() > 6.0_s && autoTimer.Get() < 7.5_s) {
      intake.SetSpeed(1);
      diffDrive.ArcadeDrive(-0.6, 0);
    } else if (autoTimer.Get() > 7.5_s && autoTimer.Get() < 9.0_s) {
      diffDrive.ArcadeDrive(0.5, 0.1);
    } else if (autoTimer.Get() > 9_s && autoTimer.Get() < 12_s) {
      if (Yaw > 0) {
        diffDrive.ArcadeDrive(0, -0.8);
        diffDrive.ArcadeDrive(0, 0);
        diffDrive.ArcadeDrive(0, -0.7);
      }
    } else if (autoTimer.Get() > 12_s && autoTimer.Get() < 13_s) {
      diffDrive.ArcadeDrive(0.6, limelight.m_LimelightTurnCmd);
    } else if (autoTimer.Get() > 13_s && autoTimer.Get() < 14.5_s) {
      ShootAmp();
    } else if (autoTimer.Get() > 14.5_s && autoTimer.Get() < 15.0_s) {
      shooterMotorLeader.Set(0);
    }
  }
}

void Robot::ShootSpeaker() {
  shooterMotorLeader.Set(constants::shooter::speakerShootSpeed);
  intake.feedMotor.Set(constants::intake::feedMotorSpeed);
}

void Robot::ShootAmp() {
  shooterMotorLeader.Set(constants::shooter::ampShootSpeed);
  intake.feedMotor.Set(constants::intake::feedMotorSpeed);
}

void Robot::RobotInit() {
  leftFrontMotor.RestoreFactoryDefaults();
  leftRearMotor.RestoreFactoryDefaults();
  rightFrontMotor.RestoreFactoryDefaults();
  rightRearMotor.RestoreFactoryDefaults();
  leftFrontMotor.SetSmartCurrentLimit(30);
  leftRearMotor.SetSmartCurrentLimit(30);
  rightFrontMotor.SetSmartCurrentLimit(30);
  rightRearMotor.SetSmartCurrentLimit(30);
  leftFrontMotor.Follow(leftRearMotor);
  rightRearMotor.Follow(rightFrontMotor);

  // Set motors to go the correct direction
  rightFrontMotor.SetInverted(false);
  rightRearMotor.SetInverted(false);
  leftFrontMotor.SetInverted(true);
  leftRearMotor.SetInverted(true);

  Pigeon.SetYaw(0_deg);

  shooterMotorLeader.GetConfigurator().Apply(
      ctre::phoenix6::configs::TalonFXConfiguration{});
  shooterMotorFollower.GetConfigurator().Apply(
      ctre::phoenix6::configs::TalonFXConfiguration{});
  shooterMotorLeader.SetInverted(true);
  shooterMotorFollower.SetControl(ctre::phoenix6::controls::Follower{
      shooterMotorLeader.GetDeviceID(), true});
  config.stripType = ctre::phoenix::led::LEDStripType::RGB;
  config.brightnessScalar = 1; // LEDs full brightness
  // ctre::phoenix::ErrorCode faultsError = candle.GetFaults(faults); //fills
  // current candle fault; returns

  // in init function
  ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};

  // set slot 0 gains
  auto &slot0Configs = talonFXConfigs.Slot0;
  slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
  slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP =
      4.8; // A position error of 2.5 rotations results in 12 V output
  slot0Configs.kI = 0;   // no output for integrated error
  slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

  // set Motion Magic settings
  auto &motionMagicConfigs = talonFXConfigs.MotionMagic;
  motionMagicConfigs.MotionMagicCruiseVelocity =
      80; // Target cruise velocity of 80 rps
  motionMagicConfigs.MotionMagicAcceleration =
      160; // Target acceleration of 160 rps/s (0.5 seconds)
  motionMagicConfigs.MotionMagicJerk =
      1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
  climberMotor.GetConfigurator().Apply(talonFXConfigs);
  // Default to a length of 60, start empty output
  // Length is expensive to set, so only set it once, then just update data
  m_autoChooser.SetDefaultOption(a_AmpAuto, AutoRoutine::kAmpAuto);
  m_autoChooser.AddOption(a_MiddleAuto, AutoRoutine::kMiddleAuto);
  m_autoChooser.AddOption(a_Middle3PcAuto, AutoRoutine::kMiddle3PcAuto);
  m_autoChooser.AddOption(a_SideAuto, AutoRoutine::kSideAuto);
  m_autoChooser.AddOption(a_driveoutAuto, AutoRoutine::kdriveoutAuto);
  m_autoChooser.AddOption(a_WallOnlySideAuto, AutoRoutine::kWallOnlySideAuto);
  m_autoChooser.AddOption(a_WallShootSideAuto, AutoRoutine::kWallShootSideAuto);
  m_autoChooser.AddOption(a_WallOnlyAmpAuto, AutoRoutine::kWallOnlyAmpAuto);
  m_autoChooser.AddOption(a_WallShootAmpAuto, AutoRoutine::kWallShootAmpAuto);
  m_autoChooser.AddOption(a_TestAuto, AutoRoutine::kTestAuto);

  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);
}

void Robot::AutonomousInit() { autoTimer.Start(); }
void Robot::AutonomousPeriodic() {
  m_autoSelected = m_autoChooser.GetSelected();

  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    Blue();
  } else {
    Red();
  }

  switch (m_autoSelected) {
  case AutoRoutine::kAmpAuto:
    ampAuto();
    break;
  case AutoRoutine::kMiddleAuto:
    middleAuto();
    break;
  case AutoRoutine::kMiddle3PcAuto:
    middle3PcAuto();
    break;
  case AutoRoutine::kSideAuto:
    sideAuto();
    break;
  case AutoRoutine::kdriveoutAuto:
    driveoutAuto();
    break;
  case AutoRoutine::kWallOnlySideAuto:
    wallOnlySideAuto();
    break;
  case AutoRoutine::kWallShootSideAuto:
    wallShootSideAuto();
    break;
  case AutoRoutine::kWallOnlyAmpAuto:
    wallOnlyAmpAuto();
    break;
  case AutoRoutine::kWallShootAmpAuto:
    wallShootAmpAuto();
    break;
  case AutoRoutine::kTestAuto:
    testAuto();
    break;
  }

  frc::SmartDashboard::PutNumber("Auto Timer", autoTimer.Get().value());
}

void Robot::TeleopPeriodic() {
  double Roll = Pigeon.GetRoll().GetValueAsDouble();
  if ((Roll < -2.3) or (Roll > 1)) {
    Rainbow();
  } else {
    if (frc::DriverStation::GetAlliance() ==
        frc::DriverStation::Alliance::kBlue) {
      Blue();
    } else {
      Red();
    }
    Green();
  }

  // Get inputs/controller mapping
  double forw = -_controller.GetLeftY();
  double spin = _controller.GetRightX();
  double intakeSpeed = _controller.GetLeftTriggerAxis();
  double shooterSpeed = _controller.GetRightTriggerAxis();
  bool reverse = _controller.GetAButton();
  bool shootAmp = _controller.GetLeftBumper();
  bool shootSpeaker = _controller.GetRightBumper();
  int pov = _controller.GetPOV();

  // Deadzone the joysticks
  if (fabs(forw) < 0.10)
    forw = 0;
  if (fabs(spin) < 0.10)
    spin = 0;
  // Set the differential drive to the commanded speed
  diffDrive.ArcadeDrive(forw, spin, true);

  // create a Motion Magic request, voltage output
  ctre::phoenix6::controls::MotionMagicVoltage m_request{0_tr};

  if (pov == 0 || pov == 45 || pov == 315)
    climberMotor.SetControl(m_request.WithPosition(400_tr));
  else if (pov == 135 || pov == 180 || pov == 225)
    climberMotor.SetControl(m_request.WithPosition(0_tr));
  // else
  // climberMotor.Set(0);
  // Reverse everything if the button is pressed
  if (reverse) {
    intakeSpeed = -intakeSpeed;
    shooterSpeed = -shooterSpeed;
  }
  if (shootAmp) {
    shooterMotorLeader.Set(constants::shooter::ampShootSpeed);
    intake.feedMotor.Set(constants::intake::feedMotorSpeed);
  }

  if (shootSpeaker) {
    shooterMotorLeader.Set(constants::shooter::speakerShootSpeed);
    intake.feedMotor.Set(constants::intake::feedMotorSpeed);
  }

  if (!shootAmp && !shootSpeaker) {
    shooterMotorLeader.Set(shooterSpeed);
    intake.SetSpeed(intakeSpeed);
  }
}

void Robot::DisabledInit() {
  autoTimer.Stop();
  autoTimer.Reset();
}

void Robot::RobotPeriodic() {
  /*if (frc::DriverStation::Alliance::kBlue and
          m_autoSelected == AutoRoutine::kMiddleAuto or
      AutoRoutine::kMiddle3PcAuto) {
    Pigeon.AddYaw(180);
  } else if (frc::DriverStation::Alliance::kBlue and
                 m_autoSelected == AutoRoutine::kSideAuto or
             AutoRoutine::kdriveoutAuto) {
    Pigeon.AddYaw(0); // test later
  } else if (frc::DriverStation::Alliance::kBlue and
             m_autoSelected == AutoRoutine::kAmpAuto) {
    Pigeon.AddYaw(90); // maybe be right, double check
  };
  if (frc::DriverStation::Alliance::kRed and
          m_autoSelected == AutoRoutine::kMiddleAuto or
      AutoRoutine::kMiddle3PcAuto) {
    Pigeon.AddYaw(0);
  } else if (frc::DriverStation::Alliance::kRed and
                 m_autoSelected == AutoRoutine::kSideAuto or
             AutoRoutine::kdriveoutAuto) {
    Pigeon.AddYaw(0); // test later
  } else if (frc::DriverStation::Alliance::kRed and
             m_autoSelected == AutoRoutine::kAmpAuto) {
    Pigeon.AddYaw(-90); // maybe be right, double check
  }; */

  // The PDP returns the voltage in increments of 0.05 Volts.
  double voltage = pdh.GetVoltage();
  frc::SmartDashboard::PutNumber("Voltage", voltage);
  // Retrieves the temperature of the PDP, in degrees Celsius.

  double temperatureCelsius = pdh.GetTemperature();

  frc::SmartDashboard::PutNumber("Temperature", temperatureCelsius);
  // Get the total current of all channels.

  double totalCurrent = pdh.GetTotalCurrent();

  frc::SmartDashboard::PutNumber("Total Current", totalCurrent);

  // Get the total power of all channels.

  // Power is the bus voltage multiplied by the current with the auto Watts.

  double totalPower = pdh.GetTotalPower();

  frc::SmartDashboard::PutNumber("Total Power", totalPower);

  // Get the total energy of all channels.

  // Energy is the power summed over time with auto Joules.

  double totalEnergy = pdh.GetTotalEnergy();

  frc::SmartDashboard::PutNumber("Total Energy", totalEnergy);
  frc::SmartDashboard::PutNumber("Pitch", Pigeon.GetPitch().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Roll", Pigeon.GetRoll().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Yaw", Pigeon.GetYaw().GetValueAsDouble());

  frc::SmartDashboard::PutNumber("Shooter TX",
                                 LimelightHelpers::getTY("limelight-greenie"));
  frc::SmartDashboard::PutNumber("Shooter TY",
                                 LimelightHelpers::getTX("limelight-greenie"));
  frc::SmartDashboard::PutNumber("Shooter TA",
                                 LimelightHelpers::getTA("limelight-greenie"));
}
void Robot::TeleopInit() {}
void Robot::DisabledPeriodic() {
  double Roll = Pigeon.GetRoll().GetValueAsDouble();
  if ((Roll < -2.3) || (Roll > 1)) {
    Rainbow();
  } else {
    Green();
    Red();
  }
}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
void Robot::TestPeriodic() {}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
