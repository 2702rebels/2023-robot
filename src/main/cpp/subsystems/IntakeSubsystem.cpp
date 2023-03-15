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

REGISTER_DOUBLE(GripperCurrentLimit, "Intake/Gripper/CurrentLimit", 10)
REGISTER_DOUBLE(GripperCurrentLimitDuration, "Intake/Gripper/CurrentLimitDuration", 0.0)
REGISTER_DOUBLE(GripperP, "Intake/Gripper/PID/P", 0.0005)
REGISTER_DOUBLE(GripperI, "Intake/Gripper/PID/I", 0.0)
REGISTER_DOUBLE(GripperD, "Intake/Gripper/PID/D", 0.0)

// TODO: We might need separate speeds for cube and cone.
constexpr double kIntakingSpeed = 0.6;
constexpr double kOuttakingSpeedCone = -0.2;
constexpr double kOuttakingSpeedCube = -0.6;

IntakeSubsystem::IntakeSubsystem() {
  m_followerMotorController.Follow(m_leaderMotorController);
  m_followerMotorController.SetInverted(ctre::phoenix::motorcontrol::InvertType::OpposeMaster);
  IntakeSubsystem::ConfigureTuningParams();
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

  m_gripperMotorEncoder.SetDistancePerPulse(
    360 / IntakeConstants::kGripperEncoderPulsesPerRevolution);
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
  frc::SmartDashboard::PutNumber("Axis3Angle", GetGripperAngle().value());
  frc::SmartDashboard::PutNumber("Axis3Current", GetGripperCurrent());

  // Axis 3 control loop.
  if (m_isGripperControlEnabled) {
    double calcVal =
      m_gripperMotorPIDController.Calculate(IntakeSubsystem::GetGripperAngle().value());
    double setpntVal = m_gripperMotorPIDController.GetSetpoint();
    double angleVal = IntakeSubsystem::GetGripperAngle().value();
    m_gripperMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, calcVal);

    // if (calcVal != debug_calcVal || setpntVal != debug_setpntVal || angleVal != debug_angleVal) {
    //   debug_calcVal = calcVal;
    //   debug_setpntVal = setpntVal;
    //   debug_angleVal = angleVal;
    //   frc::DataLogManager::Log(
    //     fmt::format("Axis3 Set Point: {} PID: {} Position: {}", setpntVal, calcVal, angleVal));
    // }
  }
}

units::degree_t IntakeSubsystem::GetGripperAngle() {
  return units::degree_t(m_gripperMotorEncoder.GetDistance());
}

double IntakeSubsystem::GetGripperCurrent() {
  return m_gripperMotor.GetStatorCurrent();
}

void IntakeSubsystem::SetGripperNextSetpoint(units::degree_t setPoint) {
  double rotations = setPoint.value();
#ifdef VERBOSE_LOG
  frc::DataLogManager::Log(
    fmt::format("Gripper: Setting setpoint to: {}, Setting rotations to: {}", setPoint, rotations));
#endif
  m_gripperMotor.Set(
    ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
    m_gripperMotorPIDController.Calculate(IntakeSubsystem::GetGripperAngle().value(), rotations));
  m_isGripperControlEnabled = true;
}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  frc::DataLogManager::Log(fmt::format("Setting intake speed to: {}", speed));
  m_leaderMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void IntakeSubsystem::SetGripperOff() {
  m_gripperMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
  frc::DataLogManager::Log(
    fmt::format("Gripper: Disabled motor by commanding motor control mode to 'Disabled'."));
}

void IntakeSubsystem::ZeroGripper() {
  m_gripperMotorEncoder.Reset();
  m_gripperMotor.Set(
    ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
    m_gripperMotorPIDController.Calculate(IntakeSubsystem::GetGripperAngle().value(), 0));
  frc::DataLogManager::Log(fmt::format("Gripper: Zeroed and started position pid at 0."));
}

void IntakeSubsystem::Intake() {
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