// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem() {
  m_followerMotorController.Follow(m_leaderMotorController);
  m_followerMotorController.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("IntakeCurrent", m_leaderMotorController.GetStatorCurrent());
  frc::SmartDashboard::PutBoolean("HasGamePiece", m_isHasGamePiece);
}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  frc::DataLogManager::Log(fmt::format("Setting intake speed to: {}", speed));
  m_leaderMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::Intake() {
  IntakeSubsystem::SetIntakeSpeed(IntakeConstants::kIntakingSpeed);
}

// void IntakeSubsystem::StopIntaking() {
//   if (m_leaderMotorController.GetStatorCurrent() >= IntakeConstants::kHasGamePieceCurrent) {
//     m_isHasGamePiece = true;
//     IntakeSubsystem::SetIntakeSpeed(IntakeConstants::kIntakeHoldSpeed);
//   } else {
//     IntakeSubsystem::SetIntakeSpeed(0.0);
//     m_isHasGamePiece = false;
//   }
// }

void IntakeSubsystem::OuttakeCone() {
  IntakeSubsystem::SetIntakeSpeed(IntakeConstants::kOuttakingSpeedCone);
}

void IntakeSubsystem::OuttakeCube() {
  IntakeSubsystem::SetIntakeSpeed(IntakeConstants::kOuttakingSpeedCube);
}

void IntakeSubsystem::Stop() {
  IntakeSubsystem::SetIntakeSpeed(0.0);
}

void IntakeSubsystem::OpenCone() {
  frc::DataLogManager::Log("Not Implemented: OpenCone");
}

void IntakeSubsystem::CloseCone() {
  frc::DataLogManager::Log("Not Implemented: CloseCone");
}

void IntakeSubsystem::OpenCube() {
  frc::DataLogManager::Log("Not Implemented: OpenCube");
}

void IntakeSubsystem::CloseCube() {
  frc::DataLogManager::Log("Not Implemented: CloseCube");
}

bool IntakeSubsystem::IsAtCommandedPosition() {
  // TODO: implement this when new gripper is available.
  return true;
}