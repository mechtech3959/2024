// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <memory>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "Constants.h"
 
using namespace pathplanner;

RobotContainer::RobotContainer() {
  NamedCommands::registerCommand(
      "Shoot Speaker", frc2::cmd::Run(
                           [this] {
                             (m_driverController.GetRightTriggerAxis() > 0.1)
                                 ? m_driverController.GetAButton()
                                       ? m_shooter.Reverse()
                                       : m_shooter.ShootSpeaker()
                                 : m_shooter.Stop();
                           },
                           {&m_shooter}));
  NamedCommands::registerCommand(
      "shootspeaker", frc2::cmd::Run([this] { m_shooter.ShootSpeaker(); }));
  NamedCommands::registerCommand(
      "shootstop", frc2::cmd::Run([this] { m_shooter.Stop(); }));
  NamedCommands::registerCommand("intake",
                                 frc2::cmd::Run([this] { m_intake.Feed(); }));
  NamedCommands::registerCommand("intakestop",
                                 frc2::cmd::Run([this] { m_intake.Stop(); }));
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  m_drive.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        m_drive.ArcadeDrive(-m_driverController.GetLeftY(),
                            -m_driverController.GetRightX());
      },
      {&m_drive}));

  m_shooter.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        (m_driverController.GetRightTriggerAxis() > 0.1)
            ? m_driverController.GetAButton() ? m_shooter.Reverse()
                                              : m_shooter.ShootSpeaker()
            : m_shooter.Stop();
      },
      {&m_shooter}));

  m_intake.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        (m_driverController.GetLeftTriggerAxis() > 0.1)
            ? m_driverController.GetAButton() ? m_intake.Reverse()
                                              : m_intake.Pickup()
            : m_intake.Stop();
      },
      {&m_intake}));

  m_led.SetDefaultCommand(
      frc2::cmd::RunOnce([this] { m_led.Rainbow(); }, {&m_led}));

  m_climber.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        int pov = m_driverController.GetPOV();
        if (pov == 0 || pov == 45 || pov == 315)
          m_climber.Extend();
        if (pov == 135 || pov == 180 || pov == 225)
          m_climber.Retract();
      },
      {&m_climber}));
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // While holding the shoulder button, drive at half speed
  frc2::JoystickButton{&m_driverController, 6}
      .OnTrue(&m_driveHalfSpeed)
      .OnFalse(&m_driveFullSpeed);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Load the path you want to follow using its name in the GUI
  auto path = PathPlannerPath::fromPathFile("ampNgo");
  auto path2 = PathPlannerPath::fromPathFile("ampNreturn");

  // Create a path following command using AutoBuilder. This will also trigger
  // event markers.
  return AutoBuilder::followPath(path);
}
