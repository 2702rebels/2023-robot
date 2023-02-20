// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HomeArmCommand.h"

#include <frc/Timer.h>

HomeArmCommand::HomeArmCommand(ExtendingArmSubsystem* arm) : m_arm(arm) {
  AddRequirements(arm);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void HomeArmCommand::Initialize() {
  m_isTimedOut = false;
  m_homingStartTime = frc::Timer::GetFPGATimestamp();
  m_arm->SetAxis2DutyCycle(-0.2);
}

// Called repeatedly when this Command is scheduled to run
void HomeArmCommand::Execute() {
  if (m_arm->GetAxis2Current() > 30) {
    m_arm->ZeroAxis2();
    HomeArmCommand::End(false);
  }

  if (m_homingStartTime + units::second_t(3) <= frc::Timer::GetFPGATimestamp()) {
    m_arm->SetAxis2Off();
    m_isTimedOut = true;
    HomeArmCommand::End(true);
  }
}

// Called once the command ends or is interrupted.
void HomeArmCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool HomeArmCommand::IsFinished() {
  return false;
}
