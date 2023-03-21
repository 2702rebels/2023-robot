// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmMoveCommand.h"

ArmMoveCommand::ArmMoveCommand(ArmSubsystem* arm, units::degree_t angle, units::inch_t distance) :
  m_arm(arm),
  m_axis1Angle(angle),
  m_axis2Distance(distance) {
  SetName("ArmMoveCommand");
  AddRequirements(arm);
  m_isUsePosition = false;
}

ArmMoveCommand::ArmMoveCommand(ArmSubsystem* arm, ArmPosition position) :
  m_arm(arm),
  m_position(position) {
  SetName("ArmMoveCommand");
  AddRequirements(arm);
  m_isUsePosition = true;
}

// Called when the command is initially scheduled.
void ArmMoveCommand::Initialize() {
  if (m_isUsePosition) {
    m_arm->Move(m_position);
  } else {
    m_arm->Move(m_axis1Angle, m_axis2Distance);
  }
}

// Called repeatedly when this Command is scheduled to run
void ArmMoveCommand::Execute() {}

// Called once the command ends or is interrupted.
void ArmMoveCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ArmMoveCommand::IsFinished() {
  if (m_arm->IsAtCommandedPosition()) {
    return true;
  }
  return false;
}
