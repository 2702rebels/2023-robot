// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Settings.h"

REGISTER_DOUBLE(IntakeInSpeed, "Intake/InSpeed", 0.6)
REGISTER_DOUBLE(IntakeOutSpeedCone, "Intake/OutSpeed/Cone", -0.2)
REGISTER_DOUBLE(IntakeOutSpeedCube, "Intake/OutSpeed/Cube", -0.6)
REGISTER_DOUBLE(WheelsCurrentLimit, "Intake/Wheels/CurrentLimit", 20)
REGISTER_DOUBLE(WheelsCurrentLimitDuration, "Intake/Wheels/CurrentLimitDuration", 0.0)

REGISTER_DOUBLE(GripperCurrentLimit, "Intake/Gripper/CurrentLimit", 5)
REGISTER_DOUBLE(GripperCurrentLimitDuration, "Intake/Gripper/CurrentLimitDuration", 0.0)
REGISTER_DOUBLE(GripperP, "Intake/Gripper/PID/P", 0.0005)
REGISTER_DOUBLE(GripperI, "Intake/Gripper/PID/I", 0.0)
REGISTER_DOUBLE(GripperD, "Intake/Gripper/PID/D", 0.0)
REGISTER_DOUBLE(GripperConeOpen, "Intake/Gripper/Positions/Cone/Open", 0);
REGISTER_DOUBLE(GripperConeClosed, "Intake/Gripper/Positions/Cone/Closed", 0);
REGISTER_DOUBLE(GripperCubeOpen, "Intake/Gripper/Positions/Cube/Open", 0);
REGISTER_DOUBLE(GripperCubeClosed, "Intake/Gripper/Positions/Cube/Closed", 0);

IntakeSubsystem::IntakeSubsystem() {
  m_leaderMotorController.SetInverted(ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput);
  IntakeSubsystem::ConfigureTuningParams();
}

void IntakeSubsystem::OnDisable() {
  IntakeSubsystem::SetGripperOff();
  IntakeSubsystem::Stop();
}

void IntakeSubsystem::ConfigureTuningParams() {
  // Gripper
  m_gripperMotor.ConfigFactoryDefault(50);
  m_gripperMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_gripperMotor.ConfigPeakCurrentLimit(GetGripperCurrentLimit());
  m_gripperMotor.ConfigPeakCurrentDuration(GetGripperCurrentLimitDuration());
  m_gripperMotor.ConfigContinuousCurrentLimit(GetGripperCurrentLimit());
  // m_gripperMotor.EnableCurrentLimit(true);
  m_gripperMotorPIDController.SetP(GetGripperP());
  m_gripperMotorPIDController.SetI(GetGripperI());
  m_gripperMotorPIDController.SetD(GetGripperD());

  m_gripperMotorEncoder.SetDistancePerPulse(0.087890625);
  frc::DataLogManager::Log(fmt::format(
    "Configured Gripper with parameters: P={}, I={}, D={}, CurrentLimit={}, "
    "CurrentLimitDuration={}",
    GetGripperP(),
    GetGripperI(),
    GetGripperD(),
    GetGripperCurrentLimit(),
    GetGripperCurrentLimitDuration()));
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("IntakeCurrent", m_leaderMotorController.GetStatorCurrent());
  frc::SmartDashboard::PutBoolean("HasGamePiece", m_isHasGamePiece);
  frc::SmartDashboard::PutNumber("GripperAngle", GetGripperAngle().value());
  frc::SmartDashboard::PutNumber("GripperCurrent", GetGripperCurrent());

  // frc::DataLogManager::Log(fmt::format("Noodles: {}", GetGripperAngle().value()));

  // Axis 3 control loop.
  if (m_isGripperControlEnabled) {
    m_gripperMotor.Set(
      ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
      m_gripperMotorPIDController.Calculate(IntakeSubsystem::GetGripperAngle().value()));
    // frc::DataLogManager::Log(fmt::format(
    //   "PID Set: {}",
    //   m_gripperMotorPIDController.Calculate(IntakeSubsystem::GetGripperAngle().value())));
  }
}

units::degree_t IntakeSubsystem::GetGripperAngle() {
  return units::degree_t(m_gripperMotorEncoder.GetDistance());
}

double IntakeSubsystem::GetGripperCurrent() {
  return m_gripperMotor.GetStatorCurrent();
}

void IntakeSubsystem::SetGripperNextSetpoint(units::degree_t setPoint) {
  frc::DataLogManager::Log(fmt::format(
    "Gripper: Setting setpoint to: {}, PID Val: {}",
    setPoint,
    m_gripperMotorPIDController.Calculate(
      IntakeSubsystem::GetGripperAngle().value(), setPoint.value())));
  m_gripperMotor.Set(
    ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
    m_gripperMotorPIDController.Calculate(
      IntakeSubsystem::GetGripperAngle().value(), setPoint.value()));
  m_isGripperControlEnabled = true;
}

void IntakeSubsystem::SetGripperDutyCycle(double dutyCycle) {
  m_isGripperControlEnabled = false;
  m_gripperMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, dutyCycle);
}

void IntakeSubsystem::SetGripperOff() {
  m_isGripperControlEnabled = false;
  m_gripperMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
  frc::DataLogManager::Log(
    fmt::format("Gripper: Disabled motor by commanding motor control mode to 'Disabled'."));
}

void IntakeSubsystem::ZeroGripper() {
  m_gripperMotorEncoder.Reset();
  SetGripperNextSetpoint(0_deg);
  frc::DataLogManager::Log(fmt::format("Gripper: Zeroed and started position pid at 0."));
}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  frc::DataLogManager::Log(fmt::format("Setting intake speed to: {}", speed));
  m_leaderMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::IntakeCone() {
  IntakeSubsystem::SetIntakeSpeed(GetIntakeInSpeed());
}

void IntakeSubsystem::IntakeCube() {
  IntakeSubsystem::SetIntakeSpeed(GetIntakeInSpeed());
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
  IntakeSubsystem::SetIntakeSpeed(GetIntakeOutSpeedCone());
}

void IntakeSubsystem::OuttakeCube() {
  IntakeSubsystem::SetIntakeSpeed(GetIntakeOutSpeedCube());
}

void IntakeSubsystem::Stop() {
  IntakeSubsystem::SetIntakeSpeed(0.0);
}

void IntakeSubsystem::HoldGamePiece() {
  IntakeSubsystem::SetIntakeSpeed(0.2);
}

void IntakeSubsystem::OpenCone() {
  frc::DataLogManager::Log(fmt::format("OpenCone. Setting angle to: {}", GetGripperConeOpen()));
  IntakeSubsystem::SetGripperNextSetpoint(units::degree_t(GetGripperConeOpen()));
}

void IntakeSubsystem::CloseCone() {
  frc::DataLogManager::Log(fmt::format("CloseCone. Setting angle to: {}", GetGripperConeClosed()));
  IntakeSubsystem::SetGripperNextSetpoint(units::degree_t(GetGripperConeClosed()));
}

void IntakeSubsystem::OpenCube() {
  frc::DataLogManager::Log(fmt::format("OpenCube. Setting angle to: {}", GetGripperCubeOpen()));
  IntakeSubsystem::SetGripperNextSetpoint(units::degree_t(GetGripperCubeOpen()));
}

void IntakeSubsystem::CloseCube() {
  frc::DataLogManager::Log(fmt::format("CloseCube. Setting angle to: {}", GetGripperCubeClosed()));
  IntakeSubsystem::SetGripperNextSetpoint(units::degree_t(GetGripperCubeClosed()));
}

bool IntakeSubsystem::IsAtCommandedPosition() {
  return m_gripperMotorPIDController.AtSetpoint();
}