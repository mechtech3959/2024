#include "Intake.h"

Intake::Intake() { Init(); }

void Intake::Init() {
  intakeFrontMotor.RestoreFactoryDefaults();
  intakeRearMotor.RestoreFactoryDefaults();
  intakeRearMotor.SetInverted(true);
}

void Intake::SetSpeed(double speed) {
  intakeFrontMotor.Set(speed);
  intakeRearMotor.Set(speed);
}
