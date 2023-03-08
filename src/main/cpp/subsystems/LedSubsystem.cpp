// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LedSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>

LedSubsystem::LedSubsystem() {
  m_isNewPattern = false;

  m_led.SetLength(LEDConstants::kLength);
  LedSubsystem::SetRGB(255, 255, 255);
  LedSubsystem::SetPattern(LEDPattern::SOLID, 0_ms);
  m_led.Start();
};

// This method will be called once per scheduler run
void LedSubsystem::Periodic() {
  // If there is a new pattern since last cycle, copy the colour buffer to the pattern buffer.
  if (m_isNewPattern) {
    m_isNewPattern = false;
    m_periodStart = frc::Timer::GetFPGATimestamp();
    memcpy(m_ledPatternBuffer.data(), m_ledBuffer.data(), m_ledBuffer.size());
  }

  double ledBrightness = 0;
  // Perform the next animation step on the pattern buffer.
  switch (m_pattern) {
    case LEDPattern::SOLID:
      break;
    case LEDPattern::FLASHING:
      if (frc::Timer::GetFPGATimestamp() > m_periodStart + (m_period / 2)) {
        for (int i = 0; i < LEDConstants::kLength; i++) {
          m_ledPatternBuffer[i].SetRGB(0, 0, 0);
        }
        if (frc::Timer::GetFPGATimestamp() > m_periodStart + m_period) {
          memcpy(m_ledPatternBuffer.data(), m_ledBuffer.data(), m_ledBuffer.size());
          m_periodStart = frc::Timer::GetFPGATimestamp();
        }
      }
      break;
    case LEDPattern::BREATH:
      ledBrightness = ((frc::Timer::GetFPGATimestamp() - m_periodStart) / m_period).value() * 2;
      if (ledBrightness > 1.0) {
        ledBrightness = 1.0 - (ledBrightness - 1.0);
      }

      for (int i = 0; i < LEDConstants::kLength; i++) {
        m_ledPatternBuffer[i].SetRGB(
          m_ledBuffer[i].r * ledBrightness,
          m_ledBuffer[i].g * ledBrightness,
          m_ledBuffer[i].b * ledBrightness);
      }

      if (frc::Timer::GetFPGATimestamp() > m_periodStart + m_period) {
        m_periodStart = frc::Timer::GetFPGATimestamp();
      }
      break;
    case LEDPattern::WAVE:
      if (frc::Timer::GetFPGATimestamp() > m_periodStart + m_period) {
        m_periodStart = frc::Timer::GetFPGATimestamp();
      }
      break;
    case LEDPattern::PONG:
      if (frc::Timer::GetFPGATimestamp() > m_periodStart + m_period) {
        m_periodStart = frc::Timer::GetFPGATimestamp();
      }
      break;
  }

  // Write the pattern buffer to the LED.
  m_led.SetData(m_ledPatternBuffer);
}

void LedSubsystem::SetRGB(int r, int g, int b) {
  frc::DataLogManager::Log(fmt::format("Setting LEDs to RGB values: ({}, {}, {})", r, g, b));
  for (int i = 0; i < LEDConstants::kLength; i++) {
    m_ledBuffer[i].SetRGB(r, g, b);
  }
}

void LedSubsystem::SetPattern(LEDPattern pattern, units::millisecond_t period) {
  m_pattern = pattern;
  m_period = period;
  m_isNewPattern = true;
  frc::DataLogManager::Log(
    fmt::format("Set LEDs pattern to: {} with period: {}", TranslatePattern(m_pattern), m_period));
}