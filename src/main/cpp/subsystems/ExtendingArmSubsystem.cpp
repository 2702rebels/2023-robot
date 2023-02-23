// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExtendingArmSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Settings.h"
#include "commands/HomeAxis1Command.h"
#include "commands/HomeAxis2Command.h"

using namespace ExtendingArmConstants;
using namespace frc2;

REGISTER_DOUBLE(Axis1P, "ExArm/TuningParameters/Axis1 PID/P", 0.05)
REGISTER_DOUBLE(Axis1I, "ExArm/TuningParameters/Axis1 PID/I", 0.0)
REGISTER_DOUBLE(Axis1D, "ExArm/TuningParameters/Axis1 PID/D", 0.0)
REGISTER_DOUBLE(Axis1kS, "ExArm/TuningParameters/Axis1 ArmFeedForward/kS", 0.0)
REGISTER_DOUBLE(Axis1kG, "ExArm/TuningParameters/Axis1 ArmFeedForward/kG", 0.0)
REGISTER_DOUBLE(Axis1kV, "ExArm/TuningParameters/Axis1 ArmFeedForward/kV", 0.0)
REGISTER_DOUBLE(Axis1kA, "ExArm/TuningParameters/Axis1 ArmFeedForward/kA", 0.0)
REGISTER_DOUBLE(Axis2P, "ExArm/TuningParameters/Axis2 PID/P", 0.03)
REGISTER_DOUBLE(Axis2I, "ExArm/TuningParameters/Axis2 PID/I", 0.0)
REGISTER_DOUBLE(Axis2D, "ExArm/TuningParameters/Axis2 PID/D", 0.0)
REGISTER_DOUBLE(Axis3P, "ExArm/TuningParameters/Axis3 PID/P", 0.0)
REGISTER_DOUBLE(Axis3I, "ExArm/TuningParameters/Axis3 PID/I", 0.0)
REGISTER_DOUBLE(Axis3D, "ExArm/TuningParameters/Axis3 PID/D", 0.0)

ExtendingArmSubsystem::ExtendingArmSubsystem() {
  frc::DataLogManager::Log(fmt::format("Initializing ExtendingArmSubsystem."));
  m_axis3Encoder.SetDistancePerPulse(360 / kAxis3EncoderPulsesPerRevolution);
  ExtendingArmSubsystem::ConfigureTuningParams();
}

void ExtendingArmSubsystem::ConfigureTuningParams() {
  // Axis 1.
  m_axis1Motor.RestoreFactoryDefaults();
  m_axis1Motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis1Motor.SetInverted(false);
  m_axis1Motor.SetSmartCurrentLimit(kAxis1CurrentLimit);
  m_axis1PIDController.SetP(GetAxis1P());
  m_axis1PIDController.SetI(GetAxis1I());
  m_axis1PIDController.SetD(GetAxis1D());
  m_axis1PIDController.SetOutputRange(kAxis1MinOutput, kAxis1MaxOutput);
  m_axis1Motor.BurnFlash();
  // m_axis1feedforward = new frc::ArmFeedforward(
  //   units::volt_t(GetAxis1kS()),
  //   units::volt_t(GetAxis1kG()),
  //   units::radians_per_second_t(GetAxis1kV()),
  //   units::radians_per_second_squared_t(GetAxis1kA()));
  frc::DataLogManager::Log(fmt::format(
    "Configured Axis1 with parameters: P={}, I={}, D={}, CurrentLimit={}",
    GetAxis1P(),
    GetAxis1I(),
    GetAxis1D(),
    kAxis1CurrentLimit));

  // Axis 2
  m_axis2Motor.RestoreFactoryDefaults();
  m_axis2Motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis2Motor.SetInverted(false);
  m_axis2Motor.SetSmartCurrentLimit(kAxis2CurrentLimit);
  m_axis2PIDController.SetP(GetAxis2P());
  m_axis2PIDController.SetI(GetAxis2I());
  m_axis2PIDController.SetD(GetAxis2D());
  m_axis2PIDController.SetOutputRange(kAxis2MinOutput, kAxis2MaxOutput);
  m_axis2Motor.BurnFlash();
  frc::DataLogManager::Log(fmt::format(
    "Configured Axis2 with parameters: P={}, I={}, D={}, CurrentLimit={}, MinOutput={}, "
    "MaxOutput={}",
    GetAxis2P(),
    GetAxis2I(),
    GetAxis2D(),
    kAxis2CurrentLimit,
    kAxis2MinOutput,
    kAxis2MaxOutput));

  // Axis 3
  m_axis3Motor.ConfigFactoryDefault(50);
  m_axis3Motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_axis3Motor.ConfigPeakCurrentLimit(kAxis3CurrentLimit);
  m_axis3Motor.ConfigPeakCurrentDuration(kAxis3CurrentLimitDuration);
  m_axis3Motor.ConfigContinuousCurrentLimit(kAxis3CurrentLimit);
  m_axis3Motor.EnableCurrentLimit(true);
  m_axis3PIDController.SetP(GetAxis3P());
  m_axis3PIDController.SetI(GetAxis3I());
  m_axis3PIDController.SetD(GetAxis3D());
  frc::DataLogManager::Log(fmt::format(
    "Configured Axis2 with parameters: P={}, I={}, D={}, CurrentLimit={}",
    GetAxis3P(),
    GetAxis3I(),
    GetAxis3D(),
    kAxis3CurrentLimit));
}

// This method will be called once per scheduler run
void ExtendingArmSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Axis1Angle", GetAxis1Angle().value());
  frc::SmartDashboard::PutNumber("Axis2Distance", GetAxis2Distance().value());
  frc::SmartDashboard::PutNumber("Axis3Angle", GetAxis3Angle().value());
  frc::SmartDashboard::PutNumber("Axis1Current", m_axis1Motor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Axis2Current", m_axis2Motor.GetOutputCurrent());
  frc::SmartDashboard::PutBoolean("Axis1HomeLimitSwitch", GetAxis1LimitSwitch());

  // Run axis 3 PID if the axis isn't disabled.
  if (m_axis3Motor.GetControlMode() != ctre::phoenix::motorcontrol::ControlMode::Disabled) {
    m_axis3Motor.Set(
      ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
      m_axis3PIDController.Calculate(ExtendingArmSubsystem::GetAxis3Angle().value()));
  }
}

void ExtendingArmSubsystem::OnDisable() {
  ExtendingArmSubsystem::SetArmOff();
}

units::degree_t ExtendingArmSubsystem::GetAxis1Angle() {
  return units::degree_t(m_axis1Encoder.GetPosition() / kAxis1MotorRotationsPerDegree);
}

units::inch_t ExtendingArmSubsystem::GetAxis2Distance() {
  return units::inch_t(m_axis2Encoder.GetPosition() / kAxis2MotorRotationsPerInchOfTravel);
}

units::degree_t ExtendingArmSubsystem::GetAxis3Angle() {
  return units::degree_t(m_axis3Encoder.GetDistance());
}

bool ExtendingArmSubsystem::GetAxis1LimitSwitch() {
  return m_axis1HomeLimitSwitch.Get();
}

double ExtendingArmSubsystem::GetAxis1Current() {
  return m_axis1Motor.GetOutputCurrent();
}

double ExtendingArmSubsystem::GetAxis2Current() {
  return m_axis2Motor.GetOutputCurrent();
}

double ExtendingArmSubsystem::GetAxis3Current() {
  return m_axis3Motor.GetStatorCurrent();
}

void ExtendingArmSubsystem::SetAxis1NextSetpoint(units::degree_t setPoint) {
  double rotations = setPoint.value() * kAxis1MotorRotationsPerDegree;
#ifdef VERBOSE_LOG
  frc::DataLogManager::Log(
    fmt::format("Axis1: Setting setpoint to: {}, Setting rotations to: {}", setPoint, rotations));
#endif

#ifdef ARMFEEDFORWARDS_ENABLED
  m_axis1PIDController.SetReference(
    rotations,
    rev::CANSparkMax::ControlType::kPosition,
    0,
    m_axis1feedforward
      ->Calculate(
        ExtendingArmSubsystem::GetAxis1Angle(),
        units::radians_per_second_t(0),
        units::radians_per_second_squared_t(0))
      .value());
#else
  m_axis1PIDController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
#endif
}

void ExtendingArmSubsystem::SetAxis2NextSetpoint(units::inch_t setPoint) {
  double rotations = setPoint.value() * kAxis2MotorRotationsPerInchOfTravel;
#ifdef VERBOSE_LOG
  frc::DataLogManager::Log(
    fmt::format("Axis2: Setting setpoint to: {}, Setting rotations to: {}", setPoint, rotations));
#endif
  m_axis2PIDController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
}

void ExtendingArmSubsystem::SetAxis3NextSetpoint(units::degree_t setPoint) {
  double rotations = setPoint.value() * kAxis2MotorRotationsPerInchOfTravel;
#ifdef VERBOSE_LOG
  frc::DataLogManager::Log(
    fmt::format("Axis3: Setting setpoint to: {}, Setting rotations to: {}", setPoint, rotations));
#endif
  m_axis3Motor.Set(
    ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
    m_axis3PIDController.Calculate(ExtendingArmSubsystem::GetAxis3Angle().value(), rotations));
}

void ExtendingArmSubsystem::SetAxis1DutyCycle(double dutyCycle) {
  m_axis1PIDController.SetReference(dutyCycle, rev::CANSparkMax::ControlType::kDutyCycle);
}

void ExtendingArmSubsystem::SetAxis2DutyCycle(double dutyCycle) {
  m_axis2PIDController.SetReference(dutyCycle, rev::CANSparkMax::ControlType::kDutyCycle);
}

void ExtendingArmSubsystem::SetAxis3DutyCycle(double dutyCycle) {
  m_axis3Motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, dutyCycle);
}

void ExtendingArmSubsystem::SetAxis1Off() {
  m_axis1Motor.StopMotor();
  frc::DataLogManager::Log(fmt::format("Axis1: Disabled motor by commanding voltage to 0"));
}

void ExtendingArmSubsystem::SetAxis2Off() {
  m_axis2Motor.StopMotor();
  frc::DataLogManager::Log(fmt::format("Axis2: Disabled motor by commanding voltage to 0."));
}

void ExtendingArmSubsystem::SetAxis3Off() {
  m_axis3Motor.Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
}

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

void ExtendingArmSubsystem::ZeroAxis3() {
  m_axis3Encoder.Reset();
  m_axis3Motor.Set(
    ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
    m_axis3PIDController.Calculate(ExtendingArmSubsystem::GetAxis3Angle().value(), 0));
  frc::DataLogManager::Log(fmt::format("Axis3: Zero'd and started position pid at 0."));
}

// frc2::CommandPtr ExtendingArmSubsystem::Home() {
//   return frc2::cmd::Sequence(
//     std::move(new HomeAxis2Command(this)), std::move(new HomeAxis1Command(this)));
// }