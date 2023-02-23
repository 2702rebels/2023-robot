// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HomeAxis1Command.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>

HomeAxis1Command::HomeAxis1Command(ExtendingArmSubsystem* arm) : m_arm(arm) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void HomeAxis1Command::Initialize() {
  frc::DataLogManager::Log(fmt::format("HomeAxis1Command: Initialize"));
  m_isTimedOut = false;
  m_isLimitHit = false;
  m_isWorkingHardProtection = false;
  m_homingStartTime = frc::Timer::GetFPGATimestamp();

  if (!m_arm->GetAxis1LimitSwitch()) {
    m_arm->SetAxis1DutyCycle(-0.2);

  } else {
    m_isLimitHit = true;
    m_arm->SetAxis1DutyCycle(0.2);
  }
}

// Called repeatedly when this Command is scheduled to run
void HomeAxis1Command::Execute() {
  if (!m_isLimitHit && m_arm->GetAxis1LimitSwitch()) {
    m_isLimitHit = true;
    m_arm->SetAxis1DutyCycle(0.2);
  }

  if (m_isLimitHit && m_arm->GetAxis1LimitSwitch()) {
    m_arm->ZeroAxis1();
    this->End(false);
  }

  if (m_arm->GetAxis1Current() >= ExtendingArmConstants::kAxis1HomeCurrentThreshold) {
    m_isWorkingHardProtection = true;
    this->End(true);
  }

  if (
    m_homingStartTime + units::second_t(ExtendingArmConstants::kAxis1HomeTimeoutInSeconds) <=
    frc::Timer::GetFPGATimestamp()) {
    m_isTimedOut = true;
    this->End(true);
  }
}

// Called once the command ends or is interrupted.
void HomeAxis1Command::End(bool interrupted) {
  frc::DataLogManager::Log(fmt::format("HomeAxis1Command: End"));
  if (interrupted) {
    m_arm->SetAxis1Off();

    if (m_isTimedOut) {
      frc::DataLogManager::Log(fmt::format("HomeAxis1Command: Timedout"));
    }

    if (m_isWorkingHardProtection) {
      frc::DataLogManager::Log(fmt::format("HomeAxis1Command: Working too hard during homing. "));
    }
  }
}

// Returns true when the command should end.
bool HomeAxis1Command::IsFinished() {
  return false;
}
