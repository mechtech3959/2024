#pragma once

#include "Constants.h"

#include <ctre/Phoenix.h>

class Shooter {
public:
  Shooter();
  void Init();
  void Forward();
  void Reverse();
  void ShootSpeaker();
  void ShootAmp();
  void Stop();

private:
  WPI_TalonSRX shooterMotorOne{constants::shooter::shooterMotorOneID};
  WPI_TalonSRX shooterMotorTwo{constants::shooter::shooterMotorTwoID};
};
