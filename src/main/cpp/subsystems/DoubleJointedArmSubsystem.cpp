// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DoubleJointedArmSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

DoubleJointedArmSubsystem::DoubleJointedArmSubsystem() {
  frc::DataLogManager::Log(fmt::format("Initializing DoubleJointedArm."));

  // Axis 1.
  frc::DataLogManager::Log(fmt::format(
    "Initializing Axis1 with parameters: P={}, I={}, D={}, FF={},",
    kAxis1P,
    kAxis1I,
    kAxis1D,
    kAxis1FF));
  m_axis1MotorController.RestoreFactoryDefaults();
  m_axis1MotorController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis1MotorController.SetSmartCurrentLimit(40);
  m_axis1PIDController.SetP(kAxis1P);
  m_axis1PIDController.SetI(kAxis1I);
  m_axis1PIDController.SetD(kAxis1D);
  m_axis1PIDController.SetFF(kAxis1FF);
  m_axis1PIDController.SetOutputRange(kAxis1MinOutput, kAxis1MaxOutput);
  m_axis1MotorController.BurnFlash();

  // Axis 2.
  frc::DataLogManager::Log(fmt::format(
    "Initializing Axis2 with parameters: P={}, I={}, D={}, FF={},",
    kAxis2P,
    kAxis2I,
    kAxis2D,
    kAxis2FF));
  m_axis2MotorController.RestoreFactoryDefaults();
  m_axis2MotorController.SetInverted(true);
  m_axis2MotorController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis2MotorController.SetSmartCurrentLimit(40);
  m_axis2PIDController.SetP(kAxis2P);
  m_axis2PIDController.SetI(kAxis2I);
  m_axis2PIDController.SetD(kAxis2D);
  m_axis2PIDController.SetFF(kAxis2FF);
  m_axis2PIDController.SetOutputRange(kAxis2MinOutput, kAxis2MaxOutput);
  m_axis2MotorController.BurnFlash();

  // Axis 3
  frc::DataLogManager::Log(fmt::format(
    "Initializing Axis3 with parameters: P={}, I={}, D={}, FF={},",
    kAxis3P,
    kAxis3I,
    kAxis3D,
    kAxis3FF));
  m_axis3MotorController.RestoreFactoryDefaults();
  m_axis3MotorController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis3MotorController.SetSmartCurrentLimit(20);
  m_axis3PIDController.SetP(kAxis3P);
  m_axis3PIDController.SetI(kAxis3I);
  m_axis3PIDController.SetD(kAxis3D);
  m_axis3PIDController.SetFF(kAxis3FF);
  m_axis3PIDController.SetOutputRange(kAxis3MinOutput, kAxis3MaxOutput);
  m_axis3MotorController.BurnFlash();
}

// This method will be called once per scheduler run
void DoubleJointedArmSubsystem::Periodic() {
  // Logging.
  frc::SmartDashboard::PutNumber("DJArmAxis1Angle", DoubleJointedArmSubsystem::GetAxis1Angle());
  frc::SmartDashboard::PutNumber("DJArmAxis2Angle", DoubleJointedArmSubsystem::GetAxis2Angle());
  frc::SmartDashboard::PutNumber("DJArmAxis3Angle", DoubleJointedArmSubsystem::GetAxis3Angle());

  // m_profile.Calculate(20_ms);
}

void DoubleJointedArmSubsystem::OnDisable() {
  DoubleJointedArmSubsystem::StopMotors();
}

void DoubleJointedArmSubsystem::ZeroArm() {
  DoubleJointedArmSubsystem::ZeroAxis1();
  DoubleJointedArmSubsystem::ZeroAxis2();
  DoubleJointedArmSubsystem::ZeroAxis3();
}

void DoubleJointedArmSubsystem::ZeroAxis1() {
  m_axis1Encoder.SetPosition(0);
  m_axis1PIDController.SetReference(0, rev::ControlType::kPosition);
  frc::DataLogManager::Log(fmt::format("Axis1: Zero'd and started position pid at 0."));
}

void DoubleJointedArmSubsystem::ZeroAxis2() {
  m_axis2Encoder.SetPosition(0);
  m_axis2PIDController.SetReference(0, rev::ControlType::kPosition);
  frc::DataLogManager::Log(fmt::format("Axis2: Zero'd and started position pid at 0."));
}

void DoubleJointedArmSubsystem::ZeroAxis3() {
  m_axis3Encoder.SetPosition(0);
  m_axis3PIDController.SetReference(0, rev::ControlType::kPosition);
  frc::DataLogManager::Log(fmt::format("Axis3: Zero'd and started position pid at 0."));
}

void DoubleJointedArmSubsystem::SetAxis1Angle(double angle) {
  frc::DataLogManager::Log(fmt::format("Commanding Axis 1 pid to {}", angle));
  double rotations = (angle - kAxis1ZeroAngle) * kAxis1MotorRotationsPerDegree;
  m_axis1PIDController.SetReference(rotations, rev::ControlType::kPosition);
}

void DoubleJointedArmSubsystem::SetAxis2Angle(double angle) {
  frc::DataLogManager::Log(fmt::format("Commanding Axis 2 pid to {}", angle));
  double rotations = (angle - kAxis2ZeroAngle) * kAxis2MotorRotationsPerDegree;
  m_axis2PIDController.SetReference(rotations, rev::ControlType::kPosition);
}

void DoubleJointedArmSubsystem::SetAxis3Angle(double angle) {
  frc::DataLogManager::Log(fmt::format("Commanding Axis 3 pid to {}", angle));
  double rotations = angle * kAxis3MotorRotationsPerDegree;
  m_axis3PIDController.SetReference(rotations, rev::ControlType::kPosition);
}

double DoubleJointedArmSubsystem::GetAxis1Angle() {
  return m_axis1Encoder.GetPosition();
}

double DoubleJointedArmSubsystem::GetAxis2Angle() {
  return m_axis2Encoder.GetPosition();
}

double DoubleJointedArmSubsystem::GetAxis3Angle() {
  return m_axis3Encoder.GetPosition();
}

void DoubleJointedArmSubsystem::SetAxis1Off() {
  m_axis1MotorController.StopMotor();
}

void DoubleJointedArmSubsystem::SetAxis2Off() {
  m_axis2MotorController.StopMotor();
}

void DoubleJointedArmSubsystem::SetAxis3Off() {
  m_axis3MotorController.StopMotor();
}

void DoubleJointedArmSubsystem::StopMotors() {
  DoubleJointedArmSubsystem::SetAxis1Off();
  DoubleJointedArmSubsystem::SetAxis2Off();
  DoubleJointedArmSubsystem::SetAxis3Off();
}