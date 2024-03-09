#pragma once

#include "Constants.h"
#include "Intake.h"
#include "LimeLight.h"
#include "LimelightHelpers.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableValue.h"

#include "Shooter.h"
// #include "TankDrive.h"
#include <ctre/Phoenix.h>
#include <frc/DriverStation.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <rev/CANSparkMax.h>
#include <units/pressure.h>
#include <units/time.h>

class Robot : public frc::TimedRobot {
private:
  // Initialize the controller
  frc::XboxController _controller{0};
  frc::Timer autoTimer;
  Shooter shooter{};
  Intake intake{};
  LimeLight limelight{"limelight-greenie"};
  rev::CANSparkMax leftFrontMotor{constants::drive::leftFrontMotorID,
                                  rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax leftRearMotor{constants::drive::leftRearMotorID,
                                 rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax rightFrontMotor{constants::drive::rightFrontMotorID,
                                   rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax rightRearMotor{constants::drive::rightRearMotorID,
                                  rev::CANSparkMax::MotorType::kBrushed};
  frc::DifferentialDrive diffDrive{rightFrontMotor, leftRearMotor};

  // Auto selection
  enum AutoRoutine {
    kAmpAuto,
    kMiddleAuto,
    kMiddle3PcAuto,
    kSideAuto,
    kdriveoutAuto
  } m_autoSelected;

  frc::SendableChooser<AutoRoutine> m_autoChooser;
  const std::string a_AmpAuto = "2 Piece Amp";
  const std::string a_MiddleAuto = "2 Piece Middle";
  const std::string a_Middle3PcAuto = "3 Piece Middle";
  const std::string a_SideAuto = "2 Piece Side";
  const std::string a_driveoutAuto = "1 Piece Side";

  std::shared_ptr<nt::NetworkTable> table =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-greenie");

public:
  Pigeon2 Pigeon{constants::drive::PigeonID};
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  // auto routines
  void ampAuto();
  void middleAuto();
  void middle3PcAuto();
  void sideAuto();
  void driveoutAuto();
  void Rainbow();
  void Green();
  void Red();
  void Yellow();
  void Blue();

  void ShootSpeaker();
  void ShootAmp();
};
