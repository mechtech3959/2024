// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/Joystick.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/CommandScheduler.h>
#include <ctre/Phoenix.h>
class Robot : public frc::TimedRobot
{
public:
  TalonSRX ML{1};
  TalonSRX CL{2};
  TalonSRX MR{3};
  TalonSRX CR{4};
  frc::Joystick DriverR{0};
  frc::Joystick DriverL{1};

  Robot() {}

  void RobotInit() override
  {
    MR.SetInverted(true);
    CR.SetInverted(true);
    //   double ML = frc::XboxController::GetLeftY;

    // frc::MotorControllerGroup::MotorControllerGroup(ML,CL);
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

  void TeleopInit() override
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.      Robot(){}

    // if (m_autonomousCommand) {
    // m_autonomousCommand->Cancel();
    //  }
  }

  /**
   * This function is called periodically during operator control.
   */
  void TeleopPeriodic() override
  {
    ML.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, DriverL.GetY());
    CL.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, DriverL.GetY());
    MR.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, DriverR.GetY());
    CR.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, DriverR.GetY());
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
