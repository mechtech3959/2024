#include "Shooter.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"

Shooter::Shooter() { Init(); }

void Shooter::Init() {
  shooterMotorOne.ConfigFactoryDefault();
  shooterMotorTwo.ConfigFactoryDefault();
  shooterMotorTwo.Follow(shooterMotorOne);
}

void Shooter::Forward() { shooterMotorOne.Set(ControlMode::PercentOutput, 1); }

void Shooter::Reverse() { shooterMotorOne.Set(ControlMode::PercentOutput, -1); }

void Shooter::ShootSpeaker() { Shooter::Forward(); }

void Shooter::ShootAmp() {
  shooterMotorOne.Set(ControlMode::PercentOutput, 0.5);
}

void Shooter::Stop() { shooterMotorOne.Set(ControlMode::PercentOutput, 0); }
