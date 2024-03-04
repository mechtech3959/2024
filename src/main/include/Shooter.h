#pragma once

#include "Constants.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

#include <ctre/Phoenix.h>

class Shooter {
public:
  Shooter();
  void SetSpeed(double speed);
  void Stop();

private:
  void Init();

  WPI_TalonSRX shooterMotorOne{constants::shooter::motorOneID};
  WPI_TalonSRX shooterMotorTwo{constants::shooter::motorTwoID};
};
