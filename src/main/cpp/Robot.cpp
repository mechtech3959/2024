#include "Robot.h"
#include <ctre/phoenix/core/GadgeteerUartClient.h>

frc::Rotation2d Robot::GetHeading() { return m_odometry.GetPose().Rotation(); }

frc::Rotation2d Robot::GetGyroHeading() { return Pigeon.GetRotation2d(); }
void Robot::setPose(frc::Pose2d pose){
  m_odometry.ResetPosition(GetGyroHeading(), diffWPos.left,
                           diffWPos.right,pose);
}
void Robot::poseupdater() {
  m_odometry.Update(Pigeon.GetRotation2d(), diffWPos.left, diffWPos.right);
  pose2d = m_odometry.GetPose();
}

void Robot::waypointtestauto() {
  limelight.updateTracking();
  units::time::second_t time = autoTimer.Get();
  frc::Pose2d p;
  if (frc::DriverStation::GetAlliance() ==
      frc::DriverStation::Alliance::kBlue) {
    // if (time < 3.0_s) {
    // ShootSpeaker();
    // } else if (time > 3.0_s and time < 4.0_s) {

    // ramseteCommand.Schedule();
    std::cout << "test";
    // p = traj.Sample(autoTimer.Get()).pose;
    // diffDrive.ArcadeDrive(p.X().value(), p.Y().value(), true);
    // std::cout << p.X().value() << " y " << p.Y().value() << std::endl;
    // } else if (time > 4.0_s) {
    // }
  } else if (frc::DriverStation::GetAlliance() ==
             frc::DriverStation::Alliance::kRed) { // red
    if (autoTimer.Get() < 3.0_s) {
      diffDrive.ArcadeDrive(0.6, limelight.m_LimelightTurnCmd);
    } else if (autoTimer.Get() > 3.0_s && autoTimer.Get() < 6.0_s) {
      ShootAmp();
    } else if (autoTimer.Get() > 6.0_s && autoTimer.Get() < 8_s) {
      diffDrive.ArcadeDrive(-0.65, 0.48);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 8.0_s && autoTimer.Get() < 10.5_s) {
      diffDrive.ArcadeDrive(0.65, -0.48);
      intake.SetSpeed(1);
    } else if (autoTimer.Get() > 10.5_s && autoTimer.Get() < 15_s) {
      ShootAmp();
    }
  } else {
    diffDrive.ArcadeDrive(0.0, 0.0);
  }
}

void Robot::ShootSpeaker() {
  shooterMotorLeader.Set(constants::shooter::speakerShootSpeed);
  intake.feedMotor.Set(constants::intake::feedMotorSpeed);
}

void Robot::ShootAmp() {
  shooterMotorLeader.Set(constants::shooter::ampShootSpeed);
  intake.feedMotor.Set(constants::intake::feedMotorSpeed);
}

void Robot::RobotInit() {

  leftFrontMotor.RestoreFactoryDefaults();
  leftRearMotor.RestoreFactoryDefaults();
  rightFrontMotor.RestoreFactoryDefaults();
  rightRearMotor.RestoreFactoryDefaults();
  m_leftEncoder.SetPosition(0_tr);
  m_rightEncoder.SetPosition(0_tr);
  leftFrontMotor.SetSmartCurrentLimit(30);
  leftRearMotor.SetSmartCurrentLimit(30);
  rightFrontMotor.SetSmartCurrentLimit(30);
  rightRearMotor.SetSmartCurrentLimit(30);
  leftFrontMotor.Follow(leftRearMotor);
  rightRearMotor.Follow(rightFrontMotor);

  // Set motors to go the correct direction
  rightFrontMotor.SetInverted(false);
  rightRearMotor.SetInverted(false);
  leftFrontMotor.SetInverted(true);
  leftRearMotor.SetInverted(true);

  Pigeon.SetYaw(0_deg);

  shooterMotorLeader.GetConfigurator().Apply(
      ctre::phoenix6::configs::TalonFXConfiguration{});
  shooterMotorFollower.GetConfigurator().Apply(
      ctre::phoenix6::configs::TalonFXConfiguration{});
  shooterMotorLeader.SetInverted(true);
  shooterMotorFollower.SetControl(ctre::phoenix6::controls::Follower{
      shooterMotorLeader.GetDeviceID(), true});

  auto [linearVelocity, vy, angularVelocity] = kinematics.ToChassisSpeeds(
      wheelSpeeds); // Converting Wheel Speeds to Chassis Speeds

  m_autoChooser.AddOption(a_TestAuto, AutoRoutine::kTestAuto);

  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);
}

void Robot::AutonomousInit() {
  autoTimer.Start();

  ramseteCommand.Schedule();
}
void Robot::AutonomousPeriodic() {
  // waypointtestauto();
  frc::SmartDashboard::PutNumber("Auto Timer", autoTimer.Get().value());
}

void Robot::TeleopPeriodic() {
  // Get inputs/controller mapping
  double forw = -_controller.GetLeftY();
  double spin = _controller.GetRightX();
  double intakeSpeed = _controller.GetLeftTriggerAxis();
  double shooterSpeed = _controller.GetRightTriggerAxis();
  bool reverse = _controller.GetAButton();
  bool shootAmp = _controller.GetLeftBumper();
  bool shootSpeaker = _controller.GetRightBumper();

  // Deadzone the joysticks
  if (fabs(forw) < 0.10)
    forw = 0;
  if (fabs(spin) < 0.10)
    spin = 0;
  // Set the differential drive to the commanded speed
  diffDrive.ArcadeDrive(forw, spin, true);

  // Reverse everything if the button is pressed
  if (reverse) {
    intakeSpeed = -intakeSpeed;
    shooterSpeed = -shooterSpeed;
  }
  if (shootAmp) {
    shooterMotorLeader.Set(constants::shooter::ampShootSpeed);
    intake.feedMotor.Set(constants::intake::feedMotorSpeed);
  }

  if (shootSpeaker) {
    shooterMotorLeader.Set(constants::shooter::speakerShootSpeed);
    intake.feedMotor.Set(constants::intake::feedMotorSpeed);
  }

  if (!shootAmp && !shootSpeaker) {
    shooterMotorLeader.Set(shooterSpeed);
    intake.SetSpeed(intakeSpeed);
  }
}

void Robot::DisabledInit() {
  autoTimer.Stop();
  autoTimer.Reset();
}

void Robot::RobotPeriodic() {
  // The PDP returns the voltage in increments of 0.05 Volts.
  double voltage = pdh.GetVoltage();
  frc::SmartDashboard::PutNumber("Voltage", voltage);
  double temperatureCelsius = pdh.GetTemperature();
  frc::SmartDashboard::PutNumber("Temperature", temperatureCelsius);
  double totalCurrent = pdh.GetTotalCurrent();
  frc::SmartDashboard::PutNumber("Total Current", totalCurrent);
  double totalPower = pdh.GetTotalPower();
  frc::SmartDashboard::PutNumber("Total Power", totalPower);
  double leftEncoderValue = m_leftEncoder.GetPosition().GetValueAsDouble();
  frc::SmartDashboard::PutNumber("Left Encoder Value", leftEncoderValue);
  double rightEncoderValue = m_rightEncoder.GetPosition().GetValueAsDouble();
  frc::SmartDashboard::PutNumber("Right Encoder Value", rightEncoderValue);
  double totalEnergy = pdh.GetTotalEnergy();
  frc::SmartDashboard::PutNumber("Total Energy", totalEnergy);
  frc::SmartDashboard::PutNumber("Pitch", Pigeon.GetPitch().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Roll", Pigeon.GetRoll().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Yaw", Pigeon.GetYaw().GetValueAsDouble());
  frc::SmartDashboard::PutNumber("Shooter TX",
                                 LimelightHelpers::getTY("limelight-greenie"));
  frc::SmartDashboard::PutNumber("Shooter TY",
                                 LimelightHelpers::getTX("limelight-greenie"));
  frc::SmartDashboard::PutNumber("Shooter TA",
                                 LimelightHelpers::getTA("limelight-greenie"));
  frc::Rotation2d gyroAngle = Pigeon.GetRotation2d();
  poseupdater();
  limelight.updateTracking();
  if (limelight.m_LimelightHasTarget)  setPose(Greenie.GetRobotPose());
  
  frc2::CommandScheduler::GetInstance().Run();
   
  // limelight.GetRobotPose();

  // this gets our calculated X and Y pose through our odometry update above
  // then puts it on drivers station
  auto posX = pose2d.X().value();
  auto posY = pose2d.Y().value();
  frc::SmartDashboard::PutNumber("Pos X", posX);
  frc::SmartDashboard::PutNumber("Pos Y", posY);
}
void Robot::TeleopInit() {}
void Robot::DisabledPeriodic() {}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() { //poseupdater();
 }
void Robot::TestPeriodic() {}
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
