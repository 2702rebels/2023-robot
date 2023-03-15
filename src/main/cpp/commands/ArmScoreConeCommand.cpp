// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmScoreConeCommand.h"

ArmScoreConeCommand::ArmScoreConeCommand(ArmSubsystem* arm, IntakeSubsystem* intake) :
  m_arm(arm),
  m_intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void ArmScoreConeCommand::Initialize() {
  m_isPlaced = false;
  m_isRetracting = false;
  m_arm->Move(
    m_arm->GetAxis1Angle() - ArmConstants::kConePlacementDistance, m_arm->GetAxis2Distance());
}

// Called repeatedly when this Command is scheduled to run
void ArmScoreConeCommand::Execute() {
  if (!m_isPlaced && m_arm->IsAtCommandedPosition()) {
    m_isPlaced = true;
    m_intake->OuttakeCone();
  }

  if (!m_isRetracting && m_isPlaced) {
    m_isRetracting = true;
    m_arm->Move(m_arm->GetAxis1Angle(), 0.0_in);
    m_outtakingStartTime = frc::Timer::GetFPGATimestamp();
  }
}

// Called once the command ends or is interrupted.
void ArmScoreConeCommand::End(bool interrupted) {
  m_intake->Stop();
}

// Returns true when the command should end.
bool ArmScoreConeCommand::IsFinished() {
  if (
    m_isRetracting &&
    frc::Timer::GetFPGATimestamp() > m_outtakingStartTime + IntakeConstants::kOuttakingTime) {
    return true;
  }
  return false;
}