#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/motorcontrol/MotorController.h>
#include <string>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
// #include <frc/SpeedControllerGroup.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include "LimelightHelpers.h"
//#include "LimeLight.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
class Robot : public frc::TimedRobot{
  private:
  LimeLight billy{"limelight-greenie"};
  public:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
   WPI_TalonSRX m_Left0{1};
  WPI_TalonSRX m_Left1{2};
  WPI_TalonSRX m_Right0{3};
  WPI_TalonSRX m_Right1{4};

 std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-greenie");
  // println("{}",table);
  /*frc::SmartDashboard::*/
  frc::DifferentialDrive m_Drive{m_Left0, m_Right0};
  frc::XboxController m_Controller{0};

  bool m_LimelightHasTarget;
  double m_LimelightTurnCmd;
  double m_LimelightDriveCmd;

  double clamp(double in, double minval, double maxval)
  {
    if (in > maxval)
      return maxval;
    if (in < minval)
      return minval;
    return in;
  };

  Robot() {}
  void Update_Limelight_Tracking()
  {
    // Proportional Steering Constant:
    // If your robot doesn't turn fast enough toward the target, make this number bigger
    // If your robot oscillates (swings back and forth past the target) make this smaller
    const double STEER_K = 0.05;

    // Proportional Drive constant: bigger = faster drive
    const double DRIVE_K = 0.26;

    // Area of the target when your robot has reached the goal
    const double DESIRED_TARGET_AREA = -2.0;
    const double MAX_DRIVE = 0.62;
    const double MAX_STEER = 1.0f;
    
    double tx = LimelightHelpers::getTX("limelight-greenie");
    double ty = LimelightHelpers::getTY("limelight-greenie");
    double ta = LimelightHelpers::getTA("limelight-greenie");
    double tv = LimelightHelpers::getTV("limelight-greenie");
    //double tt = table->GetNumber("tx", 0.0);
     // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-greenie");
    


   /* double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    double targetArea = table->GetNumber("ta",0.0);
    double targetSkew = table->GetNumber("ts",0.0);
      std::cout << "tx" << tx << "\n";
      std::cout << ty << "\n";
      //std::cout <<"f: "<< targetArea << "\n";
      std::cout << "hello";*/
    if (tv < 1.0)
    {
      m_LimelightHasTarget = false;
      m_LimelightDriveCmd = 0.0;
      m_LimelightTurnCmd = 0.0;
      //std::cout << targetSkew << "\n";
     }
    else
    {

      m_LimelightHasTarget = true;
 
      // Proportional steering
      m_LimelightTurnCmd = tx * STEER_K;
      m_LimelightTurnCmd = clamp(m_LimelightTurnCmd, -MAX_STEER, MAX_STEER);

      // drive forward until the target area reaches our desired area
      m_LimelightDriveCmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
      m_LimelightDriveCmd = clamp(m_LimelightDriveCmd, -MAX_DRIVE, MAX_DRIVE);
    }
  }
 

  

  void RobotInit() override
  {
    m_Right0.SetInverted(true);
    m_Right1.SetInverted(true);

    m_Left0.Follow(m_Left1);
    m_Right0.Follow(m_Right1);

    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    
  }

  void RobotPeriodic() override {}

  void AutonomousInit() override {}

  void AutonomousPeriodic() override {}

  void TeleopInit() override {}

  void TeleopPeriodic() override
  {
    double fwd = m_Controller.GetLeftY();
    double turn = m_Controller.GetLeftX();
    turn *= 0.7f;
    //m_Drive.ArcadeDrive(fwd, turn);

    Update_Limelight_Tracking();
    if (m_Controller.GetAButton())
    {
      if (m_LimelightHasTarget)
      {
         // Proportional steering
         m_Drive.ArcadeDrive(m_LimelightDriveCmd, -m_LimelightTurnCmd, true);
      }
    /*else
    {
      m_Drive.ArcadeDrive(0.0, 0.0);
    }*/}
    else
    {
      // Tank Drive
      // double left = -m_Cont roller.GetY(frc::GenericHID::JoystickHand::kLeftHand);
      // double right = -m_Controller.GetY(frc::GenericHID::JoystickHand::kRightHand);
      // m_Drive.TankDrive(left,right);

      // Arcade Drive
      double fwd = -m_Controller.GetLeftY();
      double turn = m_Controller.GetLeftX();
      turn *= 0.7f;
      m_Drive.ArcadeDrive(fwd, turn);
    }
    //m_Drive.ArcadeDrive(fwd, turn);
      
  

  };


  void TestPeriodic() override {}

};
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif