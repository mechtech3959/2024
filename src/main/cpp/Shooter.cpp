#include "Shooter.h"
#include "Constants.h"

Shooter::Shooter() { Init(); }

void Shooter::Init() {
  shooterMotorOne.ConfigFactoryDefault();
  shooterMotorTwo.ConfigFactoryDefault();

  shooterMotorOne.SetInverted(true);
}

void Shooter::Forward() {
  shooterMotorOne.Set(ControlMode::PercentOutput, 1);
  shooterMotorTwo.Set(ControlMode::PercentOutput, 1);
  ;
};
void Shooter::Reverse() {
  shooterMotorOne.Set(ControlMode::PercentOutput, -1);
  shooterMotorTwo.Set(ControlMode::PercentOutput, -1);
}

void Shooter::ShootSpeaker() {
  shooterMotorOne.Set(ControlMode::PercentOutput,
                      constants::shooter::speakerShootSpeed);
  shooterMotorTwo.Set(ControlMode::PercentOutput,
                      constants::shooter::speakerShootSpeed);
}

void Shooter::ShootAmp() {
  shooterMotorOne.Set(ControlMode::PercentOutput,
                      constants::shooter::ampShootSpeed);
  shooterMotorTwo.Set(ControlMode::PercentOutput,
                      constants::shooter::ampShootSpeed);
}

void Shooter::Stop() {
  shooterMotorOne.Set(ControlMode::PercentOutput, 0);
  shooterMotorTwo.Set(ControlMode::PercentOutput, 0);
}
