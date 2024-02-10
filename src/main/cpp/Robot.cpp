// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>

#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
// #include "LimelightHelpers.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
/*#include <wpinet/PortForwarder.h>
#include "wpi/json.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>*/

class Robot : public frc::TimedRobot {
public:
  WPI_TalonSRX MR{1};
  WPI_TalonSRX CR{2};
  WPI_TalonSRX ML{3};
  WPI_TalonSRX CL{4};
  frc::XboxController drive{0};
  frc::Field2d m_field;
  // nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  // std::shared_ptr<nt::NetworkTable> table = inst.GetTable("datatable");
  std::shared_ptr<nt::NetworkTable> table =nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
  double targetArea = table->GetNumber("ta", 0.0);
  double targetSkew = table->GetNumber("ts", 0.0);
  // nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("<variablename>",0.0);
  // nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("<variablename>",std::vector<double>(6));

  frc::DifferentialDrive DD{ML, MR};

  Robot() {}

  void RobotInit() override {
    ML.SetInverted(true);
    CL.SetInverted(true);

    CL.Follow(ML);
    CR.Follow(MR);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use
   * this for items like diagnostics that you want to run during disabled,
   * autonomous, teleoperated and test.
   *
   * <p> This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  void RobotPeriodic() override {
    frc::SmartDashboard::PutData("Field",
                                 &m_field); // IDRK WHAT THIS DOES TBH
    frc2::CommandScheduler::GetInstance().Run();
  }

  /**
   * This function is called periodically during operator control.
   */
  void TeleopPeriodic() override {

    double forw = drive.GetLeftY();
    double spin = drive.GetLeftX();

    float Kp = -0.1f;
    float min_command = 0.05f;

   nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    float tx = table->GetNumber("tx", 0.0);

    std::cout << "tx is" << tx;

    if (drive.GetAButton()) {
      float heading_error = -targetOffsetAngle_Horizontal;
      std::cout << "heading error is" << heading_error;
      float steering_adjust = 0.0f;
          steering_adjust = Kp * targetOffsetAngle_Horizontal;
      if (abs(heading_error) > 1.0) {
        if (heading_error < 0) {
          steering_adjust = Kp * heading_error + min_command;
                    std::cout << steering_adjust;

        } else {
          steering_adjust = Kp * heading_error - min_command;
          std::cout << steering_adjust;
        }
      }
     // forw += steering_adjust;
    //spin -= steering_adjust;
     //DD.ArcadeDrive(forw += steering_adjust,spin -= steering_adjust);
    }
    DD.ArcadeDrive(forw, spin, true);
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
