#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  // void Periodic() override;

  /**
   * Pick up a note
   */
  void Pickup();

  /**
   * Feed a note for shooting
   */
  void Feed();

  /**
   * Reverse the intake
   */
  void Reverse();

  /**
   * Stop the intake
   */
  void Stop();

private:
  // The motor controllers
  rev::CANSparkMax m_pickupMotor;
  rev::CANSparkMax m_feedMotor;
};
