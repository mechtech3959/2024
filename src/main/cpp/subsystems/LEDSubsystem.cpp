#include "subsystems/LEDSubsystem.h"

using namespace LEDConstants;

LEDSubsystem::LEDSubsystem()
    : m_candle{kCANdleID},
      m_rainbow{kCompetitionMode ? 1 : 0.5, kCompetitionMode ? 1 : 0.5, 89} {}

void LEDSubsystem::Rainbow() { m_candle.Animate(m_rainbow); }

void LEDSubsystem::White() {
  m_candle.ClearAnimation(0);
  m_candle.SetLEDs(255, 255, 255);
}

void LEDSubsystem::Green() {
  m_candle.ClearAnimation(0);
  m_candle.SetLEDs(0, 255, 0);
}

void LEDSubsystem::Red() {
  m_candle.ClearAnimation(0);
  m_candle.SetLEDs(255, 0, 0);
}

void LEDSubsystem::Yellow() {
  m_candle.ClearAnimation(0);
  m_candle.SetLEDs(255, 255, 0);
}

void LEDSubsystem::Blue() {
  m_candle.ClearAnimation(0);
  m_candle.SetLEDs(0, 0, 255);
}
