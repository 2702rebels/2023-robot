// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExtendingArmSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

ExtendingArmSubsystem::ExtendingArmSubsystem() {
  // Axis 1.
  m_axis1Motor.RestoreFactoryDefaults();
  m_axis1Motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_axis1PIDController.SetP(kAxis1P);
  m_axis1PIDController.SetI(kAxis1I);
  m_axis1PIDController.SetD(kAxis1D);
  // m_axis1PIDController.SetIZone(kAxis1Iz);
  m_axis1PIDController.SetFF(kAxis1FF);
  m_axis1PIDController.SetOutputRange(kAxis1MinOutput, kAxis1MaxOutput);

  // m_axis1Encoder.SetPositionConversionFactor(2);

  m_axis1Motor.BurnFlash();

  // Axis 2
  m_axis2Motor.RestoreFactoryDefaults();

  m_axis2PIDController.SetP(kAxis2P);
  m_axis2PIDController.SetI(kAxis2I);
  m_axis2PIDController.SetD(kAxis2D);
  // m_axis2PIDController.SetIZone(kAxis2Iz);
  m_axis2PIDController.SetFF(kAxis2FF);
  m_axis2PIDController.SetOutputRange(kAxis2MinOutput, kAxis2MaxOutput);

  m_axis2Motor.BurnFlash();

  // // Axis 3
  // m_axis3Motor.RestoreFactoryDefaults();

  // m_axis3PIDController.SetP(kAxis3P);
  // m_axis3PIDController.SetI(kAxis3I);
  // m_axis3PIDController.SetD(kAxis3D);
  // //m_axis3PIDController.SetIZone(kAxis3Iz);
  // m_axis3PIDController.SetFF(kAxis3FF);
  // m_axis3PIDController.SetOutputRange(kAxis3MinOutput, kAxis3MaxOutput);

  // m_axis3Motor.BurnFlash();
}

// This method will be called once per scheduler run
void ExtendingArmSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Axis1Angle", GetAxis1Angle());
  frc::SmartDashboard::PutNumber("Axis2Distance", GetAxis2Distance());
  frc::SmartDashboard::PutNumber("Axis3Angle", GetAxis3Angle());
}

double ExtendingArmSubsystem::GetAxis1Angle() {
  return m_axis1Encoder.GetPosition();
}

double ExtendingArmSubsystem::GetAxis2Distance() {
  return m_axis2Encoder.GetPosition();
}

double ExtendingArmSubsystem::GetAxis3Angle() {
  return 0;
}

void ExtendingArmSubsystem::SetAxis1Angle(double angle) {
  double rotations = (angle - kAxis1ZeroAngle) * kAxis1MotorRotationsPerDegree;
  m_axis1PIDController.SetReference(rotations, rev::ControlType::kPosition);
}

void ExtendingArmSubsystem::SetAxis1Off() {
  m_axis1PIDController.SetReference(0, rev::ControlType::kVoltage);
}

void ExtendingArmSubsystem::SetAxis2Distance(double distance) {
  double rotations = distance * kAxis2MotorRotationsPerInchOfTravel;
  m_axis2PIDController.SetReference(rotations, rev::ControlType::kPosition);
}

void ExtendingArmSubsystem::SetAxis3Angle(double angle) {}

void ExtendingArmSubsystem::ZeroAxis1() {
  m_axis1PIDController.SetReference(0, rev::ControlType::kVoltage);
  m_axis1Encoder.SetPosition(0);
}

void ExtendingArmSubsystem::ZeroAxis2() {
  m_axis2PIDController.SetReference(0, rev::ControlType::kVoltage);
  m_axis2Encoder.SetPosition(0);
}