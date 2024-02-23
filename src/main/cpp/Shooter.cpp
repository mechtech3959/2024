#include "Shooter.h"

Shooter::Shooter() { Init(); }

void Shooter::Init() {
  shooterMotorOne.ConfigFactoryDefault();
  shooterMotorTwo.ConfigFactoryDefault();
  shooterMotorThree.ConfigFactoryDefault();
  shooterMotorFour.ConfigFactoryDefault();

  shooterMotorTwo.Follow(shooterMotorOne);
  shooterMotorThree.Follow(shooterMotorOne);
  shooterMotorFour.Follow(shooterMotorOne);
}

void Shooter::Forward() { shooterMotorOne.Set(ControlMode::PercentOutput, 1); }

void Shooter::Reverse() { shooterMotorOne.Set(ControlMode::PercentOutput, -1); }

void Shooter::ShootSpeaker() { Shooter::Forward(); }

void Shooter::ShootAmp() { shooterMotorOne.Set(ControlMode::PercentOutput, 0.5); }

void Shooter::Stop() { shooterMotorOne.Set(ControlMode::PercentOutput, 0); }
