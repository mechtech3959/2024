#include "Intake.h"

Intake::Intake() { Init(); }

void Intake::Init() {
  pickupMotor.RestoreFactoryDefaults();
  feedMotor.RestoreFactoryDefaults();
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
