#include "subsystems/ClimberSubsystem.h"

using namespace ClimberConstants;

ClimberSubsystem::ClimberSubsystem()
    : m_motor{kClimberMotorID, GeneralConstants::kCanBus}, m_motmag{0_tr},
      m_config{} {
  // set slot 0 gains
  auto &slot0Configs = m_config.Slot0;
  slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
  slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
  slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
  slot0Configs.kP =
      4.8; // A position error of 2.5 rotations results in 12 V output
  slot0Configs.kI = 0;   // no output for integrated error
  slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

  // set Motion Magic settings
  auto &motionMagicConfigs = m_config.MotionMagic;
  motionMagicConfigs.MotionMagicCruiseVelocity =
      80; // Target cruise velocity of 80 rps
  motionMagicConfigs.MotionMagicAcceleration =
      160; // Target acceleration of 160 rps/s (0.5 seconds)
  motionMagicConfigs.MotionMagicJerk =
      1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
  m_motor.GetConfigurator().Apply(m_config);
}

void ClimberSubsystem::Extend() {
  m_motor.SetControl(m_motmag.WithPosition(kMotorRotations));
}

void ClimberSubsystem::Retract() {
  m_motor.SetControl(m_motmag.WithPosition(0_tr));
}
