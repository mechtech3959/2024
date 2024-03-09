#pragma once

#include "Constants.h"

#include <ctre/Phoenix.h>

class Shooter {
public:
  Shooter();
  void SetSpeed(double speed);
  void Stop();

private:
  void Init();

  WPI_TalonFX shooterMotorOne{constants::shooter::motorOneID};
  WPI_TalonFX shooterMotorTwo{constants::shooter::motorTwoID};
};
