// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem() {
  m_leaderMotorController.SetInverted(ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput);
  m_followerMotorController.Follow(m_leaderMotorController);
  m_followerMotorController.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("IntakeCurrent", m_leaderMotorController.GetStatorCurrent());
}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  m_leaderMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}
