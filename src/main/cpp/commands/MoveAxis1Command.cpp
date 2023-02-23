// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveAxis1Command.h"

#include <frc/DataLogManager.h>

MoveAxis1Command::MoveAxis1Command(ExtendingArmSubsystem* arm, units::degree_t axis1Angle) :
  m_arm(arm),
  m_axisGoal(axis1Angle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void MoveAxis1Command::Initialize() {
  frc::DataLogManager::Log(fmt::format(
    "ArmAxis1Command: Initialize | Goal:{}, CurrentPosition:{}",
    m_axisGoal,
    m_arm->GetAxis1Angle()));

  m_profileTime = 0_ms;
  m_profile = new frc::TrapezoidProfile<units::degree>(
    frc::TrapezoidProfile<units::degree>::Constraints { units::degrees_per_second_t(15.0),
                                                        units::degrees_per_second_squared_t(10.0) },
    frc::TrapezoidProfile<units::degree>::State { m_axisGoal, units::degrees_per_second_t(0) },
    frc::TrapezoidProfile<units::degree>::State { m_arm->GetAxis1Angle(),
                                                  units::degrees_per_second_t(0) });
}

// Called repeatedly when this Command is scheduled to run
void MoveAxis1Command::Execute() {
  m_arm->SetAxis1NextSetpoint(m_profile->Calculate(m_profileTime).position);
  m_profileTime += 20_ms;
}

// Called once the command ends or is interrupted.
void MoveAxis1Command::End(bool interrupted) {
  frc::DataLogManager::Log(fmt::format("ArmAxis1Command: End"));
}

// Returns true when the command should end.
bool MoveAxis1Command::IsFinished() {
  if (m_profile->IsFinished(m_profileTime)) {
    return true;
  }

  return false;
}
