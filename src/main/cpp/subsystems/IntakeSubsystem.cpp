#include "subsystems/IntakeSubsystem.h"

using namespace IntakeConstants;

IntakeSubsystem::IntakeSubsystem()
    : m_pickupMotor{kPickupMotorID, rev::CANSparkMax::MotorType::kBrushless},
      m_feedMotor{kFeedMotorID, rev::CANSparkMax::MotorType::kBrushless} {
  m_pickupMotor.RestoreFactoryDefaults();
  m_feedMotor.RestoreFactoryDefaults();
  m_pickupMotor.SetSmartCurrentLimit(kPickupMotorCurrentLimit);
  m_feedMotor.SetSmartCurrentLimit(kFeedMotorCurrentLimit);
  m_feedMotor.SetInverted(true);
}

void IntakeSubsystem::Pickup() {
  m_pickupMotor.Set(kPickupMotorSpeed);
  m_feedMotor.Set(kFeedMotorSpeed);
}

void IntakeSubsystem::Feed() { m_feedMotor.Set(kFeedMotorSpeed); }

void IntakeSubsystem::Reverse() {
  m_pickupMotor.Set(-kPickupMotorSpeed);
  m_feedMotor.Set(-kFeedMotorSpeed);
}

void IntakeSubsystem::Stop() {
  m_pickupMotor.Set(0);
  m_feedMotor.Set(0);
}
