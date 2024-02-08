// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/CommandScheduler.h>
#include <ctre/Phoenix.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/MotorController.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
// #include "wpi/span.h"
#include "wpi/SpanExtras.h"
// #include "Shooter.h"
// #include "LimelightHelpers.h"
class Robot : public frc::TimedRobot
{
public:
  // TalonSRX ML{1};
  // TalonSRX CL{2};
  // TalonSRX MR{3};
  // TalonSRX CR{4};
  frc::PWMTalonSRX MR{1};
  frc::PWMTalonSRX CR{2};
  frc::PWMTalonSRX ML{3};
  frc::PWMTalonSRX CL{4};
  frc::XboxController drive{0};
  frc::Field2d m_field;
 // nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  //std::shared_ptr<nt::NetworkTable> table = inst.GetTable("datatable");
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx", 0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty", 0.0);
  double targetArea = table->GetNumber("ta", 0.0);
  double targetSkew = table->GetNumber("ts", 0.0);
   //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("<variablename>",0.0);
 // nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("<variablename>",std::vector<double>(6));





  frc::DifferentialDrive DD{ML, MR};
  // Do this in either robot periodic or subsystem periodic

  Robot() {}

  void RobotInit() override
  {
    ML.SetInverted(true);
    CL.SetInverted(true);
    // SFollow();
     ML.AddFollower(CL);
    MR.AddFollower(CR);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use
   * this for items like diagnostics that you want to run during disabled,
   * autonomous, teleoperated and test.
   *
   * <p> This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  void RobotPeriodic() override
  {
    frc::SmartDashboard::PutData("Field", &m_field); // IDRK WHAT THIS DOES TBH
    frc2::CommandScheduler::GetInstance().Run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  void DisabledInit() override {}

  void DisabledPeriodic() override {}

  /**
   * This autonomous runs the autonomous command selected by your {@link
   * RobotContainer} class.
   */
  void AutonomousInit() override
  {
    // m_autonomousCommand = m_container.GetAutonomousCommand();

    /*  if (m_autonomousCommand) {
        m_autonomousCommand->Schedule();
      }*/
  }

  void AutonomousPeriodic() override {}

  void TeleopInit() override{
      // This makes sure that the autonomous stops running when
      // teleop starts running. If you want the autonomous to
      // continue until interrupted by another command, remove
      // this line or comment it out.      Robot(){}

      // if (m_autonomousCommand) {
      // m_autonomousCommand->Cancel();
      //  }
  };

  /**
   * This function is called periodically during operator control.
   */
  void TeleopPeriodic() override
  {
     double forw = drive.GetLeftY();
    double spin = drive.GetLeftX();
    float Kp = -0.1f;
float min_command = 0.05f;
  // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
 float tx = table->GetNumber("tx",0.0);

if (drive.GetRawButton(9))
{
    float heading_error = -tx;
    float steering_adjust = 0.0f;
    if (abs(heading_error) > 1.0) //    if (Math.abs(heading_error) > 1.0) 

    {
        if (heading_error < 0) 
        {
            steering_adjust = Kp*heading_error + min_command;
        } 
        else 
        {
            steering_adjust = Kp*heading_error - min_command;
        }
    }
    forw += steering_adjust;
    spin -= steering_adjust;
}

    // ML.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput,drive.GetLeftY());
    // MR.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput,drive.GetRightX());
    /* if(drive.GetBButtonPressed()){
     SpeakerS();
    }else if(drive.GetXButtonPressed()){
     AmpS();
    };*/
    /*if(ML.GetMotorOutputPercent()>0){
     drive.SetRumble(drive.kLeftRumble,1);
    }else if(MR.GetMotorOutputPercent()>0){
         drive.SetRumble(drive.kRightRumble,1);
         };
         */

    DD.ArcadeDrive(forw, spin, true);
  }

  /**
   * This function is called periodically during test mode.
   */
  void TestPeriodic() override {}

  /**
   * This function is called once when the robot is first started up.
   */
  void SimulationInit() override {}

  /**
   * This function is called periodically whilst in simulation.
   */
  void SimulationPeriodic() override {}
};
#ifndef RUNNING_FRC_TESTS

int main()
{
  return frc::StartRobot<Robot>();
}
#endif
