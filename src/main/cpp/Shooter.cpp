#include "Shooter.h"
#include "Constants.h"

Shooter::Shooter() { Init(); }

void Shooter::Init() {
  shooterMotorOne.ConfigFactoryDefault();
  shooterMotorTwo.ConfigFactoryDefault();

  shooterMotorOne.SetInverted(true);
}

void Shooter::SetSpeed(double speed) {
  shooterMotorOne.Set(ControlMode::PercentOutput, speed);
  shooterMotorTwo.Set(ControlMode::PercentOutput, speed);
};

void Shooter::Stop() {
  shooterMotorOne.Set(ControlMode::PercentOutput, 0);
  shooterMotorTwo.Set(ControlMode::PercentOutput, 0);
}
