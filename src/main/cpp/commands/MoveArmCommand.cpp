// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveArmCommand.h"

#include <frc/DataLogManager.h>

MoveArmCommand::MoveArmCommand(
  ExtendingArmSubsystem* arm, double axis1Angle, units::meter_t axis2Distance) :
  m_arm(arm),
  m_axis1Goal(axis1Angle),
  m_axis2Goal(axis2Distance) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void MoveArmCommand::Initialize() {
  frc::DataLogManager::Log(fmt::format(
    "ArmMoveCommand: Initialize | Goal:{}, CurrentPosition:{}",
    m_axis2Goal,
    m_arm->GetAxis2Distance()));

  m_profileTime = 0_ms;
  m_profile = new frc::TrapezoidProfile<units::meters>(
    frc::TrapezoidProfile<units::meters>::Constraints { 0.25_mps, 0.1_mps_sq },
    frc::TrapezoidProfile<units::meters>::State { m_axis2Goal, 0_mps },
    frc::TrapezoidProfile<units::meters>::State {
      units::make_unit<units::inch_t, double>(m_arm->GetAxis2Distance()), 0_mps });
}

// Called repeatedly when this Command is scheduled to run
void MoveArmCommand::Execute() {
  units::inch_t positionInInches = m_profile->Calculate(m_profileTime).position;
  // frc::DataLogManager::Log(fmt::format(
  //   "ArmMoveCommand: Execute | profileTime:{}, positionInInches:{}",
  //   m_profileTime,
  //   positionInInches));
  m_arm->SetAxis2NextSetpoint(positionInInches.value());
  m_profileTime += 20_ms;
}

// Called once the command ends or is interrupted.
void MoveArmCommand::End(bool interrupted) {
  frc::DataLogManager::Log(fmt::format("ArmMoveCommand: End"));
}

// Returns true when the command should end.
bool MoveArmCommand::IsFinished() {
  if (m_profile->IsFinished(m_profileTime)) {
    return true;
  }

  return false;
}
