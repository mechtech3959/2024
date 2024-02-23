#pragma once

#include "Constants.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

#include <ctre/Phoenix.h>

class Shooter {
public:
  Shooter();
  void Forward();
  void Reverse();
  void ShootSpeaker();
  void ShootAmp();
  void Stop();

private:
  void Init();

  WPI_TalonSRX shooterMotorOne{constants::shooter::shooterMotorOneID};
  WPI_TalonSRX shooterMotorTwo{constants::shooter::shooterMotorTwoID};
  WPI_TalonFX shooterMotorThree{constants::shooter::shooterMotorThreeID};
  WPI_TalonFX shooterMotorFour{constants::shooter::shooterMotorFourID};
};
