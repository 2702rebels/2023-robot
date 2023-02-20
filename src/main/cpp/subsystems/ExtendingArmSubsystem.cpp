// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExtendingArmSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

ExtendingArmSubsystem::ExtendingArmSubsystem() {
  frc::DataLogManager::Log(fmt::format("Initializing ExtendingArmSubsystem."));

  // Axis 1.
  frc::DataLogManager::Log(fmt::format(
    "Initializing Axis1 with parameters: P={}, I={}, D={}, FF={},",
    kAxis1P,
    kAxis1I,
    kAxis1D,
    kAxis1FF));
  m_axis1Motor.RestoreFactoryDefaults();
  m_axis1Motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis1Motor.SetInverted(false);
  m_axis1Motor.SetSmartCurrentLimit(40);
  m_axis1PIDController.SetP(kAxis1P);
  m_axis1PIDController.SetI(kAxis1I);
  m_axis1PIDController.SetD(kAxis1D);
  // m_axis1PIDController.SetIZone(kAxis1Iz);
  m_axis1PIDController.SetFF(kAxis1FF);
  m_axis1PIDController.SetOutputRange(kAxis1MinOutput, kAxis1MaxOutput);
  // m_axis1Encoder.SetPositionConversionFactor(2);
  m_axis1Motor.BurnFlash();

  // Axis 2
  frc::DataLogManager::Log(fmt::format(
    "Initializing Axis2 with parameters: P={}, I={}, D={}, FF={},",
    kAxis2P,
    kAxis2I,
    kAxis2D,
    kAxis2FF));
  m_axis2Motor.RestoreFactoryDefaults();
  m_axis2Motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis2Motor.SetSmartCurrentLimit(20);
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

void ExtendingArmSubsystem::CurrentSetupParameters() {
  frc::DataLogManager::Log(fmt::format(
    "Axis 1 values as: P={}, I={}, D={}, FF={},",
    m_axis1PIDController.GetP(),
    m_axis1PIDController.GetI(),
    m_axis1PIDController.GetD(),
    m_axis1PIDController.GetFF()));

  frc::DataLogManager::Log(fmt::format(
    "Axis 2 values as: P={}, I={}, D={}, FF={},",
    m_axis2PIDController.GetP(),
    m_axis2PIDController.GetI(),
    m_axis2PIDController.GetD(),
    m_axis2PIDController.GetFF()));
}

// This method will be called once per scheduler run
void ExtendingArmSubsystem::Periodic() {
  if (!m_setupLogged) {
    if (m_setupLoggedIters > 4) {
      ExtendingArmSubsystem::CurrentSetupParameters();
      m_setupLogged = true;
      m_setupLoggedIters = 0;
    } else {
      m_setupLoggedIters++;
    }
  }

  frc::SmartDashboard::PutNumber("Axis1Angle", GetAxis1Angle());
  frc::SmartDashboard::PutNumber("Axis2Distance", GetAxis2Distance());
  frc::SmartDashboard::PutNumber("Axis3Angle", GetAxis3Angle());
  frc::SmartDashboard::PutNumber("Axis1Current", m_axis1Motor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Axis2Current", m_axis2Motor.GetOutputCurrent());

  if (m_enablePID) {
    // frc::TrapezoidProfile<units::meters> profile { m_axis2Constraints,
    //                                                m_axis2Goal,
    //                                                m_axis2Setpoint };
    // m_axis2Setpoint = profile.Calculate(20_ms);
  }
}

void ExtendingArmSubsystem::OnDisable() {
  ExtendingArmSubsystem::SetArmOff();
}

double ExtendingArmSubsystem::GetAxis1Angle() {
  return m_axis1Encoder.GetPosition() / kAxis1MotorRotationsPerDegree;
}

double ExtendingArmSubsystem::GetAxis2Distance() {
  return m_axis2Encoder.GetPosition() / kAxis2MotorRotationsPerInchOfTravel;
}

double ExtendingArmSubsystem::GetAxis3Angle() {
  return 0;
}

void ExtendingArmSubsystem::SetAxis1Off() {
  m_axis1Motor.StopMotor();
  frc::DataLogManager::Log(fmt::format("Axis1: Disabled motor by commanding voltage to 0"));
}

void ExtendingArmSubsystem::SetAxis2Off() {
  m_axis2Motor.StopMotor();
  frc::DataLogManager::Log(fmt::format("Axis2: Disabled motor by commanding voltage to 0."));
}

void ExtendingArmSubsystem::SetAxis3Off() {}

void ExtendingArmSubsystem::SetArmOff() {
  ExtendingArmSubsystem::SetAxis1Off();
  ExtendingArmSubsystem::SetAxis2Off();
  ExtendingArmSubsystem::SetAxis3Off();
}

void ExtendingArmSubsystem::ZeroAxis1() {
  m_axis1Encoder.SetPosition(0);
  m_axis1PIDController.SetReference(0, rev::CANSparkMax::ControlType::kPosition);
  frc::DataLogManager::Log(fmt::format("Axis1: Zero'd and started position pid at 0."));
}

void ExtendingArmSubsystem::ZeroAxis2() {
  m_axis2Encoder.SetPosition(0);
  m_axis2PIDController.SetReference(0, rev::CANSparkMax::ControlType::kPosition);
  frc::DataLogManager::Log(fmt::format("Axis2: Zero'd and started position pid at 0."));
}

void ExtendingArmSubsystem::ZeroAxis3() {}

void ExtendingArmSubsystem::ZeroArm() {
  ExtendingArmSubsystem::ZeroAxis1();
  ExtendingArmSubsystem::ZeroAxis2();
  ExtendingArmSubsystem::ZeroAxis3();
}

void ExtendingArmSubsystem::SetAxis1NextSetpoint(double setPoint) {
  double rotations = setPoint * kAxis1MotorRotationsPerDegree;
#ifdef VERBOSE_LOG
  frc::DataLogManager::Log(
    fmt::format("Axis1: Setting setpoint to: {}, Setting rotations to: {}", setPoint, rotations));
#endif
  m_axis1PIDController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
}

void ExtendingArmSubsystem::SetAxis2NextSetpoint(double setPoint) {
  double rotations = setPoint * kAxis2MotorRotationsPerInchOfTravel;
#ifdef VERBOSE_LOG
  frc::DataLogManager::Log(
    fmt::format("Axis2: Setting setpoint to: {}, Setting rotations to: {}", setPoint, rotations));
#endif
  m_axis2PIDController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
}

double ExtendingArmSubsystem::GetAxis1Current() {
  return m_axis1Motor.GetOutputCurrent();
}

double ExtendingArmSubsystem::GetAxis2Current() {
  return m_axis2Motor.GetOutputCurrent();
}

void ExtendingArmSubsystem::SetAxis1DutyCycle(double dutyCycle) {
  m_axis1PIDController.SetReference(dutyCycle, rev::CANSparkMax::ControlType::kDutyCycle);
}

void ExtendingArmSubsystem::SetAxis2DutyCycle(double dutyCycle) {
  m_axis2PIDController.SetReference(dutyCycle, rev::CANSparkMax::ControlType::kDutyCycle);
}