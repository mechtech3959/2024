#pragma once

#include <ctre/phoenix/led/CANdle.h>
#include <ctre/phoenix/led/RainbowAnimation.h>
#include <frc2/command/SubsystemBase.h>

class LEDSubsystem : public frc2::SubsystemBase {
public:
  LEDSubsystem();

  // LED functions
  void Rainbow();
  void White();
  void Green();
  void Red();
  void Yellow();
  void Blue();

private:
  // Initialize CTRE CANdle
  ctre::phoenix::led::CANdle m_candle; // creates a new candle with ID 40

  // Configure the candle
  // Set parameters like strip type, RGB, brightness, and other settings
  ctre::phoenix::led::CANdleConfiguration config;
  // Additional Configurations
  // Configure behavior when losing communication
  ctre::phoenix::ErrorCode losConfigResult = m_candle.ConfigLOSBehavior(true);

  ctre::phoenix::led::RainbowAnimation m_rainbow;
};
