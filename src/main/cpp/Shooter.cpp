#include "Shooter.h"

Shooter::Shooter() { Init(); }

void Shooter::Init() {
  shooterMotorOne.ConfigFactoryDefault();
  shooterMotorTwo.ConfigFactoryDefault();

  shooterMotorTwo.SetInverted(true);
  // shooterMotorThree.Follow(shooterMotorOne);
  // shooterMotorFour.Follow(shooterMotorThree);
}

void Shooter::Forward() {
  shooterMotorOne.Set(ControlMode::PercentOutput, 1);
  shooterMotorTwo.Set(ControlMode::PercentOutput, 1);
}

void Shooter::Reverse() {
  shooterMotorOne.Set(ControlMode::PercentOutput, -1);
  shooterMotorTwo.Set(ControlMode::PercentOutput, -1);
}

void Shooter::ShootSpeaker() {
  shooterMotorOne.Set(ControlMode::PercentOutput, 1);
  shooterMotorTwo.Set(ControlMode::PercentOutput, 1);
}

void Shooter::ShootAmp() {
  shooterMotorOne.Set(ControlMode::PercentOutput, 0.5);
  shooterMotorTwo.Set(ControlMode::PercentOutput, 0.5);
}

void Shooter::Stop() {
  shooterMotorOne.Set(ControlMode::PercentOutput, 0);
  shooterMotorTwo.Set(ControlMode::PercentOutput, 0);
}
