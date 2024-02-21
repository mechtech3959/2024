#pragma once

#include "Constants.h"
#include "rev/CANSparkMax.h"

#include <ctre/Phoenix.h>

class Intake {
public:
  Intake();
  void Init();
  void Forward();
  void Reverse();
  void Stop();

private:
  rev::CANSparkMax intakeFrontMotor{constants::intake::intakeFrontMotorID,
                                    rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax intakeRearMotor{constants::intake::intakeRearMotorID,
                                   rev::CANSparkMax::MotorType::kBrushless};
};
