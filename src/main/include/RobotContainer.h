// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "pathplanner/lib/auto/AutoBuilder.h"
#include <frc/PowerDistribution.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "LimeLight.h"
#include "LimelightHelpers.h"
#include "frc/DataLogManager.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LEDSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  frc2::CommandPtr PutDashboardCommand();

  void TeleopInit();
private:
  // The driver's controller
  frc2::CommandXboxController m_driverController{
      OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LEDSubsystem m_led;
  ClimberSubsystem m_climber;

  frc::PowerDistribution pdh{};

  // Set drive to half speed
  frc2::InstantCommand m_driveHalfSpeed{[this] { m_drive.SetMaxOutput(0.5); },
                                        {}};

  // Set drive to full speed
  frc2::InstantCommand m_driveFullSpeed{[this] { m_drive.SetMaxOutput(1); },
                                        {}};

  // Run the intake
  frc2::InstantCommand m_pickupStart{[this] { m_intake.Pickup(); },
                                     {&m_intake}};

  // Stop the intake
  frc2::InstantCommand m_pickupStop{[this] { m_intake.Stop(); }, {&m_intake}};

  // Extend the climber
  frc2::InstantCommand m_extendClimber{[this] { m_climber.Extend(); },
                                       {&m_climber}};

  // Retract the climber
  frc2::InstantCommand m_retractClimber{[this] { m_climber.Retract(); },
                                        {&m_climber}};

  // Run the shooter with inverts
  frc2::InstantCommand m_shootSpeaker{[this] {
                                        m_driverController.GetAButton()
                                            ? m_shooter.Reverse()
                                            : m_shooter.ShootSpeaker();
                                      },
                                      {&m_shooter}};

  // Stop the shooter
  frc2::InstantCommand m_shootStop{[this] { m_shooter.Stop(); }, {&m_shooter}};

  // Start the intake with inverts
  frc2::InstantCommand m_startIntake{[this] {
                                       m_driverController.GetAButton()
                                           ? m_intake.Reverse()
                                           : m_intake.Pickup();
                                     },
                                     {&m_intake}};

  // Stop the intake
  frc2::InstantCommand m_stopIntake{[this] { m_intake.Stop(); }, {&m_intake}};

  void ConfigureButtonBindings();
  void GetAutonomousPos();

  frc::SendableChooser<frc2::Command *> chooser;
};
