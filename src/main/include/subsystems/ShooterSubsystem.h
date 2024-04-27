#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
public:
  ShooterSubsystem();

  // Will be called periodically whenever the CommandScheduler runs.
  // void Periodic() override;

  // Shoots at the predefined amp shoot speed
  void ShootAmp();

  // Shoots at the predefined speaker shoot speed
  void ShootSpeaker();

  // Reverses the shooter
  void Reverse();

  // Stops the shooter
  void Stop();

   /**
   * Shoots at the commanded speed
   *
   * @param speed the commanded shoot speed
   */
  // void Shoot(double speed);

private:
  // The motor controllers
  ctre::phoenix6::hardware::TalonFX m_leftShooterMotor;
  ctre::phoenix6::hardware::TalonFX m_rightShooterMotor;
};
