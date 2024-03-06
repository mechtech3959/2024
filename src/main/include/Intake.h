#pragma once

#include "Constants.h"
#include "rev/CANSparkMax.h"

#include <ctre/Phoenix.h>

class Intake {
public:
  Intake();
  void Init();
  void SetSpeed(double speed);
  void Stop();
  void Stuck();

  rev::CANSparkMax feedMotor{constants::intake::feedMotorID,
                             rev::CANSparkMax::MotorType::kBrushless};

private:
  rev::CANSparkMax pickupMotor{constants::intake::pickupMotorID,
                               rev::CANSparkMax::MotorType::kBrushless};
};
