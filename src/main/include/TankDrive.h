#pragma once

#include "Constants.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h"

#include <ctre/Phoenix.h>
#include <frc/drive/DifferentialDrive.h>

class TankDrive {
public:
  TankDrive() { Init(); }

  frc::DifferentialDrive diffDrive{rightFrontMotor, leftFrontMotor};
 
private:
  void Init() {
    leftFrontMotor.ConfigFactoryDefault();
    leftRearMotor.ConfigFactoryDefault();
    rightFrontMotor.ConfigFactoryDefault();
    rightRearMotor.ConfigFactoryDefault();

    leftRearMotor.Follow(leftFrontMotor);
    rightRearMotor.Follow(rightFrontMotor);

    // Set motors to go the correct direction
    rightFrontMotor.SetInverted(true);
    rightRearMotor.SetInverted(true);
    leftFrontMotor.SetInverted(false);
    leftRearMotor.SetInverted(false);

    // Set TalonSRX LEDs to be the correct colors
    rightFrontMotor.SetSensorPhase(true);
    leftFrontMotor.SetSensorPhase(true);
  }

  WPI_TalonSRX leftFrontMotor{constants::drive::leftFrontMotorID};
  WPI_TalonSRX leftRearMotor{constants::drive::leftRearMotorID};
  WPI_TalonSRX rightFrontMotor{constants::drive::rightFrontMotorID};
  WPI_TalonSRX rightRearMotor{constants::drive::rightRearMotorID};
};
