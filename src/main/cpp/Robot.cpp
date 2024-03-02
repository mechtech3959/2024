#include "Robot.h"
#include "LimeLight.h"
#include <ctre/phoenix6/Orchestra.hpp>
#include <frc/AddressableLED.h>
#include <unistd.h>
TalonFX MUSIC{12};

static constexpr int kLength = 300;

frc::AddressableLED m_led{0};

std::array<frc::AddressableLED::LEDData, kLength>
    m_ledBuffer; // Reuse the buffer

// Store what the last hue of the first pixel is

int firstPixelHue = 0;
void Robot::Green() {
  for (int i = 0; i < kLength; i++) {
    m_ledBuffer[i].SetRGB(0, 255, 0);
  }
}
void Robot::Red() {
  for (int i = 0; i < kLength; i++) {
    m_ledBuffer[i].SetRGB(255, 0, 0);
  }
}
void Robot::Yellow() {
  for (int i = 0; i < kLength; i++) {
    m_ledBuffer[i].SetRGB(255, 255, 0);
  }
}
void Robot::Blue() {
  for (int i = 0; i < kLength; i++) {
    m_ledBuffer[i].SetRGB(0, 0, 255);
  }
}
void Robot::Rainbow() {

  // For every pixel

  for (int i = 0; i < kLength; i++) {

    // Calculate the hue - hue is easier for rainbows because the color

    // shape is a circle so only one value needs to precess

    const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;

    // Set the value

    m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
  }

  // Increase by to make the rainbow "move"

  firstPixelHue += 3;

  // Check bounds

  firstPixelHue %= 180;
}

void Robot::ampAuto() { // zac did this :)
  limelight.updateTracking();
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 3.0_s) {
      drive.diffDrive.ArcadeDrive(
          0.6, limelight.m_LimelightTurnCmd); // drive towards apriltag
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 6.0_s) {
      shooter.ShootAmp(); // shoot
    } else if (autoTimer.Get() > 6.0_s && autoTimer.Get() < 8_s) {
      drive.diffDrive.ArcadeDrive(-0.65, -0.48); // turn to note and intake it
      intake.Forward();
    } else if (autoTimer.Get() > 8.0_s && autoTimer.Get() < 10.5_s) {
      drive.diffDrive.ArcadeDrive(0.65, 0.48); // turn to amp
      intake.Forward();
    } else if (autoTimer.Get() > 10.5_s && autoTimer.Get() < 15_s) {
      shooter.ShootAmp(); // score
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) { // red
    if (autoTimer.Get() < 3.0_s) {
      drive.diffDrive.ArcadeDrive(0.6, limelight.m_LimelightTurnCmd);
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 6.0_s) {
      shooter.ShootAmp();
    } else if (autoTimer.Get() > 6.0_s && autoTimer.Get() < 8_s) {
      drive.diffDrive.ArcadeDrive(-0.65, 0.48);
      intake.Forward();
    } else if (autoTimer.Get() > 8.0_s && autoTimer.Get() < 10.5_s) {
      drive.diffDrive.ArcadeDrive(0.65, -0.48);
      intake.Forward();
    } else if (autoTimer.Get() > 10.5_s && autoTimer.Get() < 15_s) {
      shooter.ShootAmp();
    }
  } else {
    // cout << "no" << endl;
    drive.diffDrive.ArcadeDrive(0.0, 0.0);
  }
}

void Robot::middleAuto() { // callie did this :D
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 3.0_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 5.0_s) {
      drive.diffDrive.ArcadeDrive(-0.6, 0.0);
      intake.Forward();
    } else if (autoTimer.Get() > 5.0_s && autoTimer.Get() < 7.5_s) {
      drive.diffDrive.ArcadeDrive(0.6, 0.0);
    } else if (autoTimer.Get() > 7.5_s && autoTimer.Get() < 10.0_s) {
      shooter.ShootSpeaker();
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 3.0_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 5.0_s) {
      drive.diffDrive.ArcadeDrive(-0.6, 0.0);
      intake.Forward();
    } else if (autoTimer.Get() > 5.0_s && autoTimer.Get() < 7.5_s) {
      drive.diffDrive.ArcadeDrive(0.6, 0.0);
    } else if (autoTimer.Get() > 7.5_s && autoTimer.Get() < 10.0_s) {
      shooter.ShootSpeaker();
    }
  } else {
    // cout << "no" << endl;
    drive.diffDrive.ArcadeDrive(0, 0);
  }
}

void Robot::middle3PcAuto() { // callie did this :D
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 2.0_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 2.0_s && autoTimer.Get() < 4.0_s) {
      drive.diffDrive.ArcadeDrive(-0.6, 0.0);
      intake.Forward();
    } else if (autoTimer.Get() > 4.0_s && autoTimer.Get() < 6.5_s) {
      drive.diffDrive.ArcadeDrive(0.6, 0.0);
    } else if (autoTimer.Get() > 6.5_s && autoTimer.Get() < 8.5_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 8.5_s && autoTimer.Get() < 11_s) {
      drive.diffDrive.ArcadeDrive(-0.65, 0.3);
      intake.Forward();
    } else if (autoTimer.Get() > 11_s && autoTimer.Get() < 13.5_s) {
      drive.diffDrive.ArcadeDrive(0.65, -0.3);
    } else if (autoTimer.Get() > 13.5_s && autoTimer.Get() < 15_s) {
      shooter.ShootSpeaker();
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 2.0_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 2.0_s && autoTimer.Get() < 4.0_s) {
      drive.diffDrive.ArcadeDrive(-0.6, 0.0);
      intake.Forward();
    } else if (autoTimer.Get() > 4.0_s && autoTimer.Get() < 6.5_s) {
      drive.diffDrive.ArcadeDrive(0.6, 0.0);
    } else if (autoTimer.Get() > 6.5_s && autoTimer.Get() < 8.5_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 8.5_s && autoTimer.Get() < 11_s) {
      drive.diffDrive.ArcadeDrive(-0.65, -0.3);
      intake.Forward();
    } else if (autoTimer.Get() > 11_s && autoTimer.Get() < 13.5_s) {
      drive.diffDrive.ArcadeDrive(0.65, 0.3);
    } else if (autoTimer.Get() > 13.5_s && autoTimer.Get() < 15_s) {
      shooter.ShootSpeaker();
    }
  } else {
    // cout << "no" << endl;
    drive.diffDrive.ArcadeDrive(0, 0);
  }
}

void Robot::sideAuto() { // Jacob did this ;/
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 3.0_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 5.1_s) {
      drive.diffDrive.ArcadeDrive(-0.8, -0.42);
      intake.Forward();
    } else if (autoTimer.Get() > 5.1_s && autoTimer.Get() < 5.6_s) {
      drive.diffDrive.ArcadeDrive(0, 0);
    } else if (autoTimer.Get() > 5.6_s && autoTimer.Get() < 7.7_s) {
      drive.diffDrive.ArcadeDrive(0.8, 0.42);
      intake.Forward();
    } else if (autoTimer.Get() > 8.7_s && autoTimer.Get() < 11.0_s) {
      shooter.ShootSpeaker();
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 3.0_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 5.1_s) {
      drive.diffDrive.ArcadeDrive(-0.8, 0.42);
      intake.Forward();
    } else if (autoTimer.Get() > 5.1_s && autoTimer.Get() < 5.6_s) {
      drive.diffDrive.ArcadeDrive(0.0, 0.0);
      intake.Forward();
    } else if (autoTimer.Get() > 5.6_s && autoTimer.Get() < 7.7_s) {
      drive.diffDrive.ArcadeDrive(0.8, -0.42);
      intake.Forward();
    } else if (autoTimer.Get() > 8.7_s && autoTimer.Get() < 11.0_s) {
      shooter.ShootSpeaker();
    }
  }
}
void Robot::driveoutAuto() { // ZAC
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    if (autoTimer.Get() < 3.0_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 5.0_s) {
      drive.diffDrive.ArcadeDrive(-0.8, -0.35);
    } else if (autoTimer.Get() > 5.0_s && autoTimer.Get() < 15.0_s) {
      drive.diffDrive.ArcadeDrive(-0.50, 0.00);
    }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) {
    if (autoTimer.Get() < 3.0_s) {
      shooter.ShootSpeaker();
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 5.0_s) {
      drive.diffDrive.ArcadeDrive(-0.8, 0.35);
    } else if (autoTimer.Get() > 5.0_s && autoTimer.Get() < 15.0_s) {
      drive.diffDrive.ArcadeDrive(0.00, 0.00);
    }
  }
}
// auto selector
void Robot::RobotInit() {
  // Default to a length of 60, start empty output

  // Length is expensive to set, so only set it once, then just update data

  m_led.SetLength(kLength);
  m_led.SetData(m_ledBuffer);
  m_led.Start();

  m_autoChooser.SetDefaultOption(a_AmpAuto, AutoRoutine::kAmpAuto);
  m_autoChooser.AddOption(a_MiddleAuto, AutoRoutine::kMiddleAuto);
  m_autoChooser.AddOption(a_Middle3PcAuto, AutoRoutine::kMiddle3PcAuto);
  m_autoChooser.AddOption(a_SideAuto, AutoRoutine::kSideAuto);
  m_autoChooser.AddOption(a_driveoutAuto, AutoRoutine::kdriveoutAuto);

  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);
}

void Robot::AutonomousInit() { autoTimer.Start(); }
void Robot::AutonomousPeriodic() {
  m_autoSelected = m_autoChooser.GetSelected();

  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    Blue();
    m_led.SetData(m_ledBuffer);
    m_led.Start();
  } else {
    Red();
    m_led.SetData(m_ledBuffer);
    m_led.Start();
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
  }

  frc::SmartDashboard::PutNumber("Auto Timer", autoTimer.Get().value());

  /* if (autoTimer.Get() < 1.0_s) {
     drive.diffDrive.ArcadeDrive(-0.6, 0.0);
   } else if (autoTimer.Get() < 1.7_s) {
     drive.diffDrive.ArcadeDrive(0.0, 0.6);
   } else if (autoTimer.Get() > 1.7_s && autoTimer.Get() < 2.7_s) {
     limelight.updateTracking();
     drive.diffDrive.ArcadeDrive(1, limelight.m_LimelightTurnCmd);
   } else if (autoTimer.Get() > 5_s && autoTimer.Get() < 9_s) {
     shooter.ShootAmp();
   }
  */ /*
  while (limelight.autoTimer.Get() < 15.0_s) {
    limelight.updateTracking();
    drive.diffDrive.ArcadeDrive(0.4, limelight.m_LimelightTurnCmd);
  }
  */
  /*
    limelight.updateTracking();
    long long ID = llround(limelight.aprilTagID);
    switch (ID) {
    case 6:
      limelight.ampAuto();
      break;
    default:
      drive.diffDrive.ArcadeDrive(0.0, 0.0);
      break;
    }
  */
  /*
  limelight.autoTimer.Start();
  if (limelight.autoTimer.Get() < 0.5_s) {
    drive.diffDrive.ArcadeDrive(0.0, 0.0);
  }
  (limelight.m_LimelightHasTarget)
      ? drive.diffDrive.ArcadeDrive(limelight.m_LimelightDriveCmd,
                                    limelight.m_LimelightTurnCmd, true)
      : drive.diffDrive.ArcadeDrive(0.0, 0.0);
  */
}

void Robot::TeleopPeriodic() {
  int i = -1;
  if (i == -1) {
    if (frc::DriverStation::GetAlliance() ==
        frc::DriverStation::Alliance::kBlue) {
      Blue();
    } else {
      Red();
    }

    m_led.SetData(m_ledBuffer);
  }
  Green();
  i = 0;
  m_led.SetData(m_ledBuffer);
  m_led.Start();

  m_led.SetData(m_ledBuffer);
  m_led.Start();

  // Get inputs/controller mapping
  double forw = -_controller.GetLeftY();
  double spin = _controller.GetRightX();
  bool loadFromIntake = _controller.GetRightBumper();
  bool loadFromFront = _controller.GetLeftBumper();
  bool forward = _controller.GetAButton();
  bool reverse = _controller.GetBButton();
  // Once we actually have a limit switch this will need to be fixed
  bool noteDetected = false;
  // Once we have a climber this will need to be fixed
  // bool climbUp = _controller.GetXButton();
  // bool climbDown = _controller.GetYButton();

  // Deadzone the joysticks
  if (fabs(forw) < 0.10)
    forw = 0;
  if (fabs(spin) < 0.10)
    spin = 0;

  // Run the shooter at 50% for the amp and feed the note in
  if (_controller.GetLeftTriggerAxis() > 0) {
    shooter.ShootAmp();
    intake.Forward();
  }
  // Run the shooter at 100% for the speaker and feed the note in
  if (_controller.GetRightTriggerAxis() > 0) {
    shooter.ShootSpeaker();
    intake.Forward();
  }

  // Set the differential drive to the commanded speed
  drive.diffDrive.ArcadeDrive(forw, spin, true);

  // Load the note in from the front
  if (loadFromFront) {
    if (!noteDetected) {
      shooter.Reverse();
      intake.Reverse();
    } else {
      shooter.Stop();
      intake.Stop();
    }
  }
  // Load the note in from the intake
  if (loadFromIntake) {
    if (!noteDetected) {
      shooter.Reverse();
      intake.Reverse();
    } else {
      shooter.Stop();
      intake.Stop();
    }
  }

  // Run everything forward if the button is pressed
  if (forward) {
    shooter.Forward();
    intake.Forward();
  }

  // Reverse everything if the button is pressed
  if (reverse) {
    shooter.Reverse();
    intake.Reverse();
  }

  // Set the climber motor speed
  // Commented out until we have a climber
  /*
  if (climbUp)
    _climber.Set(1);
  if (climbDown)
    _climber.Set(1);
  */

  // If nothing is commanded stop the motors
  if (_controller.GetXButton()) {
    shooter.Stop();
    intake.Stop();
  }
}

void Robot::DisabledInit() {
  autoTimer.Stop();
  autoTimer.Reset();
  m_led.SetLength(kLength);

  m_led.SetData(m_ledBuffer);
}

void Robot::RobotPeriodic() {}
void Robot::TeleopInit() {}
void Robot::DisabledPeriodic() {
  int i = -1;
  if (i == -1) {
    Green();
    m_led.SetData(m_ledBuffer);
  }
  Red();
  i = 0;
  m_led.SetData(m_ledBuffer);
  m_led.Start();
}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}
void Robot::TestPeriodic() {}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
