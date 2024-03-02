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
#include "TankDrive.h"
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
// using namespace std;
//  Unneeded headers
//  Leaving them here for future use
//  #include <frc2/command/CommandPtr.h>
//  #include <frc/MathUtil.h>
//  #include <frc/filter/SlewRateLimiter.h>
//  #include "ctre/phoenix/motorcontrol/ControlMode.h"
//  #include <frc/Compressor.h>
//  #include <frc/trajectory/Trajectory.h>
//  #include <frc/trajectory/TrajectoryGenerator.h>
//  #include <frc/MathUtil.h>
//  #include <units/math.h>
//  #include <frc/DriverStation.h>
//  #include "LimeLight.h"
//  #include <units/pressure.h>

class Robot : public frc::TimedRobot {
private:
  // Initialize the controller
  frc::XboxController _controller{0};
  frc::Timer autoTimer;
  Shooter shooter{};
  Intake intake{};
  TankDrive drive{};
  LimeLight limelight{"limelight-greenie"};

  /* static constexpr int kLength = 60;

   // PWM port 9
   // Must be a PWM header, not MXP or DIO
   frc::AddressableLED m_led{0};
   std::array<frc::AddressableLED::LEDData, kLength>
       m_ledBuffer; // Reuse the buffer
   // Store what the last hue of the first pixel is
   int firstPixelHue = 0;
 */
  // Everything from here until the public block is auto stuff
  // I have no idea how it works, ask Zac
  /*  frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto"; */
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
};
