// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmScoreCubeCommand.h"

#include <frc/Timer.h>

ArmScoreCubeCommand::ArmScoreCubeCommand(ArmSubsystem* arm, IntakeSubsystem* intake) :
  m_arm(arm),
  m_intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void ArmScoreCubeCommand::Initialize() {
  m_intake->Outtake();
  m_arm->Move(m_arm->GetAxis1Angle(), 0.0_in);
  m_startTime = frc::Timer::GetFPGATimestamp();
}

// Called repeatedly when this Command is scheduled to run
void ArmScoreCubeCommand::Execute() {}

// Called once the command ends or is interrupted.
void ArmScoreCubeCommand::End(bool interrupted) {
  m_intake->Stop();
}

// Returns true when the command should end.
bool ArmScoreCubeCommand::IsFinished() {
  if (frc::Timer::GetFPGATimestamp() > m_startTime + IntakeConstants::kOuttakingTime) {
    return true;
  }
  return false;
}
