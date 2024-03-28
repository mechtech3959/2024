#pragma once

#include "Constants.h"
#include <rev/CANSparkMax.h>

class Intake {
public:
  Intake();
  void Init();
  void SetSpeed(double speed);
  void Stop();
  void Stuck();

  rev::CANSparkMax feedMotor{constants::intake::feedMotorID,
                             rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax pickupMotor{constants::intake::pickupMotorID,
                               rev::CANSparkMax::MotorType::kBrushless};
};
