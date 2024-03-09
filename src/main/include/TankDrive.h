#pragma once

#include "Constants.h"
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>

class TankDrive {
public:
  TankDrive() { Init(); }

  frc::DifferentialDrive diffDrive{rightFrontMotor, leftFrontMotor};

private:
  void Init() {
    leftFrontMotor.RestoreFactoryDefaults();
    leftRearMotor.RestoreFactoryDefaults();
    rightFrontMotor.RestoreFactoryDefaults();
    rightRearMotor.RestoreFactoryDefaults();

    leftRearMotor.Follow(leftFrontMotor);
    rightRearMotor.Follow(rightFrontMotor);

    // Set motors to go the correct direction
    rightFrontMotor.SetInverted(true);
    rightRearMotor.SetInverted(true);
    leftFrontMotor.SetInverted(false);
    leftRearMotor.SetInverted(false);
  }

  rev::CANSparkMax leftFrontMotor{constants::drive::leftFrontMotorID,
                                  rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax leftRearMotor{constants::drive::leftRearMotorID,
                                 rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax rightFrontMotor{constants::drive::rightFrontMotorID,
                                   rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax rightRearMotor{constants::drive::rightRearMotorID,
                                  rev::CANSparkMax::MotorType::kBrushed};
};
