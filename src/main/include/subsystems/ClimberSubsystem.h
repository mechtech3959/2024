#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/SubsystemBase.h>

class ClimberSubsystem : public frc2::SubsystemBase {
public:
  ClimberSubsystem();

  void Extend();
  void Retract();

private:
  ctre::phoenix6::hardware::TalonFX m_motor;
  ctre::phoenix6::controls::MotionMagicVoltage m_motmag;
  ctre::phoenix6::configs::TalonFXConfiguration m_config;
};
