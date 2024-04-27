#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <memory>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

using namespace pathplanner;

RobotContainer::RobotContainer() {
  m_drive.Periodic();
  NamedCommands::registerCommand(
      "shootspeaker", frc2::cmd::RunOnce([this] { m_shooter.ShootSpeaker(); }));
  NamedCommands::registerCommand(
      "shootstop", frc2::cmd::RunOnce([this] { m_shooter.Stop(); }));
  NamedCommands::registerCommand(
      "intake", frc2::cmd::RunOnce([this] { m_intake.Feed(); }));
  NamedCommands::registerCommand(
      "intakestop", frc2::cmd::RunOnce([this] { m_intake.Stop(); }));
  chooser = AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &chooser);

  // Configure the button bindings
  ConfigureButtonBindings();

  m_led.SetDefaultCommand(
      frc2::cmd::RunOnce([this] { m_led.Rainbow(); }, {&m_led}));
}

void RobotContainer::ConfigureButtonBindings() {
  // While holding the shoulder button, drive at half speed
  m_driverController.RightBumper()
      .OnTrue(&m_driveHalfSpeed)
      .OnFalse(&m_driveFullSpeed);

  // Extend the climber if DPad Up is pushed
  frc2::POVButton{&m_driverController, 0}.OnTrue(&m_extendClimber);

  // Retract the climber if DPad Down is pushed
  frc2::POVButton{&m_driverController, 180}.OnFalse(&m_retractClimber);

  // If the right trigger is pushed run the shooter, else stop it
  m_driverController.RightTrigger()
      .OnTrue(&m_shootSpeaker)
      .OnFalse(&m_shootStop);

  // If the left trigger is pushed run the intake, else stop it
  m_driverController.LeftTrigger()
      .OnTrue(&m_startIntake)
      .OnFalse(&m_stopIntake);
}

/// Returns a CommandPtr to the selected autonomous routine
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Load the path you want to follow using its name in the GUI
  return frc2::CommandPtr{
      std::unique_ptr<frc2::Command>{chooser.GetSelected()}};
}

void RobotContainer::GetAutonomousPos() {
  frc::Pose2d pathstartpose = PathPlannerAuto::getStartingPoseFromAutoFile(
      chooser.GetSelected()->GetName());

  m_drive.ResetOdometry(pathstartpose);
}
frc2::CommandPtr RobotContainer::PutDashboardCommand() {
  return frc2::cmd::Run([this] {
    frc::SmartDashboard::PutNumber("X", m_drive.poseXY().X().value());
    frc::SmartDashboard::PutNumber("Y", m_drive.poseXY().Y().value());
    frc::SmartDashboard::PutNumber("Voltage", pdh.GetVoltage());
    frc::SmartDashboard::PutNumber("Temperature", pdh.GetTemperature());
    frc::SmartDashboard::PutNumber("Total Current", pdh.GetTotalCurrent());
    frc::SmartDashboard::PutNumber("Total Power", pdh.GetTotalPower());
    frc::SmartDashboard::PutNumber("Total Energy", pdh.GetTotalEnergy());
    frc::SmartDashboard::PutNumber(
        "Pitch", m_drive.GetPigeon().GetPitch().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(
        "Roll", m_drive.GetPigeon().GetRoll().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(
        "Yaw", m_drive.GetPigeon().GetYaw().GetValueAsDouble());
    frc::SmartDashboard::PutNumber(
        "Shooter TX", LimelightHelpers::getTY("limelight-greenie"));
    frc::SmartDashboard::PutNumber(
        "Shooter TY", LimelightHelpers::getTX("limelight-greenie"));
    frc::SmartDashboard::PutNumber(
        "Shooter TA", LimelightHelpers::getTA("limelight-greenie"));
  });
}
void RobotContainer::TeleopInit() {
  // Set up default drive command
  m_drive.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        m_drive.ArcadeDrive(-m_driverController.GetLeftY(),
                            -m_driverController.GetRightX());
      },
      {&m_drive}));
  m_shootStop.Schedule();
  m_stopIntake.Schedule();
}
