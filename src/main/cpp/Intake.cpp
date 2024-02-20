#include "Intake.h"

Intake::Intake() { Init(); }

void Intake::Init() {
  intakeFrontMotor.RestoreFactoryDefaults();
  intakeRearMotor.RestoreFactoryDefaults();
  intakeRearMotor.SetInverted(true);
}

void Intake::Forward() {
  intakeFrontMotor.Set(1);
  intakeRearMotor.Set(1);
}

void Intake::Reverse() {
  intakeFrontMotor.Set(-1);
  intakeRearMotor.Set(-1);
}

void Intake::Stop() {
  intakeFrontMotor.Set(0);
  intakeRearMotor.Set(0);
}
