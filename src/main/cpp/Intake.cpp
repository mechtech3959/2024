#include "Intake.h"
#include "Constants.h"

Intake::Intake() { Init(); }

void Intake::Init() {
  pickupMotor.RestoreFactoryDefaults();
  feedMotor.RestoreFactoryDefaults();
  pickupMotor.SetSmartCurrentLimit(
      constants::intake::defaultPickupMotorCurrentLimit);
  feedMotor.SetSmartCurrentLimit(
      constants::intake::defaultFeedMotorCurrentLimit);
  feedMotor.SetInverted(true);
}

void Intake::SetSpeed(double speed) {
  pickupMotor.Set(speed);
  feedMotor.Set(speed);
}

void Intake::Stop() {
  pickupMotor.Set(0);
  feedMotor.Set(0);
}
/*
void Intake::Stuck() {
  // UNTESTED 3/6/24
  frc::Timer stuckTimer;
  stuckTimer.Start();
  if (stuckTimer.Get() < 0.5_s) {
    pickupMotor.Set(-0.3);
    feedMotor.Set(-0.3);
  } else if (stuckTimer.Get() > 0.5_s and stuckTimer.Get() < 1.0_s) {
    pickupMotor.Set(0.3);
    feedMotor.Set(0.3);
  }
  stuckTimer.Stop();
  stuckTimer.Reset();
}
*/
