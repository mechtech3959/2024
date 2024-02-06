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
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
// #include "Shooter.h"
       class Robot:public frc::TimedRobot {
    public:
    // TalonSRX ML{1};
    // TalonSRX CL{2};
    // TalonSRX MR{3};
    //TalonSRX CR{4};
    frc::PWMTalonSRX MR{1};
    frc::PWMTalonSRX CR{2};
    frc::PWMTalonSRX ML{3};
    frc::PWMTalonSRX CL{4};
    frc::XboxController drive{0};
    frc::Field2d m_field;
   
    frc::DifferentialDrive DD { ML,MR
    //  [&](double output) { ML.Set(output); },
    //  [&](double output) { MR.Set(output); };
   };

// Do this in either robot periodic or subsystem periodic
 
       
      Robot(){}
   
   void RobotInit()   override {
    ML.SetInverted(true);
    CL.SetInverted(true);
   // SFollow();
    frc::SmartDashboard::PutData("Field", &m_field); // IDRK WHAT THIS DOES TBH    
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
void RobotPeriodic() override {
    
  frc2::CommandScheduler::GetInstance().Run();
 }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void DisabledInit()override {}

void DisabledPeriodic() override{}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void AutonomousInit() override{
  //m_autonomousCommand = m_container.GetAutonomousCommand();

/*  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }*/
}

void AutonomousPeriodic() override{}

void TeleopInit() override {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.      Robot(){}
 
   
  // if (m_autonomousCommand) {
   // m_autonomousCommand->Cancel();
//  }
 DD.ArcadeDrive(-drive.GetLeftY(),drive.GetRightX());
};

/**
 * This function is called periodically during operator control.
 */
void TeleopPeriodic()  override{
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

   
   
  }

/**
 * This function is called periodically during test mode.
 */
void TestPeriodic()override {}

/**
 * This function is called once when the robot is first started up.
 */
void SimulationInit()override {}

/**
 * This function is called periodically whilst in simulation.
 */
void SimulationPeriodic() override{}
       };
   #ifndef RUNNING_FRC_TESTS
      
int main() {
  return frc::StartRobot<Robot>();
}
#endif
