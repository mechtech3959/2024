#pragma once

#include "Constants.h"
#include "Intake.h"
#include "LimeLight.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/AddressableLED.h>
#include <frc/DriverStation.h>
#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/math.h>
#include <units/pressure.h>
#include <units/time.h>
#include <units/velocity.h>

class Robot : public frc::TimedRobot {
private:
  // Initialize stuff
  frc::XboxController _controller{0};
  frc::Timer autoTimer;
  frc::PowerDistribution pdh{};
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
  ctre::phoenix6::hardware::TalonFX shooterMotorLeader{
      constants::shooter::motorOneID, constants::canBus};
  ctre::phoenix6::hardware::TalonFX shooterMotorFollower{
      constants::shooter::motorTwoID, constants::canBus};
  // Initialize the Pigeon
  ctre::phoenix6::hardware::Pigeon2 Pigeon{constants::drive::PigeonID,
                                           constants::canBus};
  // Initialize the Encoders
  ctre::phoenix6::hardware::CANcoder m_leftEncoder{
      constants::drive::m_leftEncoderID, constants::canBus};
  ctre::phoenix6::hardware::CANcoder m_rightEncoder{
      constants::drive::m_rightEncoderID, constants::canBus};
  // Initialize the field
  frc::Field2d Field;
  // Auto selection
  enum AutoRoutine {
    kAmpAuto,
    kMiddleAuto,
    kMiddle3PcAuto,
    kSideAuto,
    kdriveoutAuto,
    kWallOnlySideAuto,
    kWallShootSideAuto,
    kWallOnlyAmpAuto,
    kWallShootAmpAuto,
    kTestAuto
  } m_autoSelected;

  // Auto chooser
  frc::SendableChooser<AutoRoutine> m_autoChooser;
  const std::string a_AmpAuto = "2 Piece Amp";
  const std::string a_MiddleAuto = "2 Piece Middle";
  const std::string a_Middle3PcAuto = "3 Piece Middle";
  const std::string a_SideAuto = "2 Piece Side";
  const std::string a_driveoutAuto = "1 Piece Side";
  const std::string a_WallOnlySideAuto = "Wall Only Side";
  const std::string a_WallShootSideAuto = "Wall Shoot Side";
  const std::string a_WallOnlyAmpAuto = "Wall Only Amp";
  const std::string a_WallShootAmpAuto = "Wall Shoot Amp";
  const std::string a_TestAuto = "secret auto";

  // Intitialize Limelight NetworkTables connection
  std::shared_ptr<nt::NetworkTable> table =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-greenie");

public:
 
   
    // Override standard functions
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

    // Auto routines
    void ampAuto();
    void middleAuto();
    void middle3PcAuto();
    void sideAuto();
    void driveoutAuto();
    void wallOnlySideAuto();
    void wallShootSideAuto();
    void wallOnlyAmpAuto();
    void wallShootAmpAuto();
    void testAuto();

    // LED functions
    void Rainbow();
    void Green();
    void Red();
    void Yellow();
    void Blue();

    // Shoot functions
    void ShootSpeaker();
    void ShootAmp();
};
