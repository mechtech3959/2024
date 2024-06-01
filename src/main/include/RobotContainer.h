#pragma once

#include "Constants.h"
#include "Drivetrain.h"

#include "pathplanner/lib/auto/AutoBuilder.h"
#include <frc/PowerDistribution.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/CommandXboxController.h>

class RobotContainer {

 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  frc2::CommandPtr PutDashboardCommand();

  void TeleopInit();
  void TeleopInit();
  private:
  void GetAutonomousPos(){};
  frc::SendableChooser<frc2::Command *> chooser;
  constants::swerveConstants::SwerveConfig config;
  Drivetrain m_swerve{config};
};