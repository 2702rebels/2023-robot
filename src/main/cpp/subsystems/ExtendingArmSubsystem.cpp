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
#include "commands/MoveAxis1Command.h"
#include "commands/MoveAxis2Command.h"

using namespace ExtendingArmConstants;
using namespace frc2;

REGISTER_DOUBLE(Axis1P, "ExArm/TuningParameters/Axis1 PID/P", 0.05)
REGISTER_DOUBLE(Axis1I, "ExArm/TuningParameters/Axis1 PID/I", 0.0)
REGISTER_DOUBLE(Axis1D, "ExArm/TuningParameters/Axis1 PID/D", 0.0)
REGISTER_DOUBLE(Axis1kS, "ExArm/TuningParameters/Axis1 ArmFeedForward/kS", 0.0)
REGISTER_DOUBLE(Axis1kG, "ExArm/TuningParameters/Axis1 ArmFeedForward/kG", 0.0)
REGISTER_DOUBLE(Axis1kV, "ExArm/TuningParameters/Axis1 ArmFeedForward/kV", 0.0)
REGISTER_DOUBLE(Axis1kA, "ExArm/TuningParameters/Axis1 ArmFeedForward/kA", 0.0)
REGISTER_DOUBLE(
  Axis1MaxVelocity, "ExArm/TuningParameters/Axis1 MotionProfile/MaxVelDegPerSecond", 100.0)
REGISTER_DOUBLE(
  Axis1MaxAcceletration, "ExArm/TuningParameters/Axis1 MotionProfile/MaxAccelDegPerSecond", 130.0)
REGISTER_DOUBLE(Axis1CurrentLimit, "ExArm/TuningParameters/Axis1CurrentLimit", 40.0)

REGISTER_DOUBLE(Axis2P, "ExArm/TuningParameters/Axis2 PID/P", 0.04)
REGISTER_DOUBLE(Axis2I, "ExArm/TuningParameters/Axis2 PID/I", 0.00001)
REGISTER_DOUBLE(Axis2D, "ExArm/TuningParameters/Axis2 PID/D", 0.0)
REGISTER_DOUBLE(
  Axis2MaxVelocity, "ExArm/TuningParameters/Axis2 MotionProfile/MaxVelInchPerSec", 40.0)
REGISTER_DOUBLE(
  Axis2MaxAcceletration, "ExArm/TuningParameters/Axis2 MotionProfile/MaxAccelInchPerSec", 60.0)
REGISTER_DOUBLE(Axis2CurrentLimit, "ExArm/TuningParameters/Axis2CurrentLimit", 20.0)

REGISTER_DOUBLE(Axis3P, "ExArm/TuningParameters/Axis3 PID/P", 0.0)
REGISTER_DOUBLE(Axis3I, "ExArm/TuningParameters/Axis3 PID/I", 0.0)
REGISTER_DOUBLE(Axis3D, "ExArm/TuningParameters/Axis3 PID/D", 0.0)
REGISTER_DOUBLE(Axis3CurrentLimit, "ExArm/TuningParameters/Axis3CurrentLimit", 0.0)
REGISTER_DOUBLE(Axis3CurrentLimitDuration, "ExArm/TuningParameters/Axis3CurrentLimitDuration", 0.0)

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
  m_axis1Motor.SetSmartCurrentLimit(GetAxis1CurrentLimit());
  m_axis1PIDController.SetP(GetAxis1P());
  m_axis1PIDController.SetI(GetAxis1I());
  m_axis1PIDController.SetD(GetAxis1D());
  m_axis1PIDController.SetOutputRange(kAxis1MinOutput, kAxis1MaxOutput);
  m_axis1Motor.BurnFlash();
  m_axis1Encoder.SetVelocityConversionFactor((1 / kAxis1MotorRotationsPerDegree) / 60);
  m_axis1feedforward = new frc::ArmFeedforward(
    units::volt_t(GetAxis1kS()),
    units::volt_t(GetAxis1kG()),
    units::unit_t<frc::ArmFeedforward::kv_unit>(GetAxis1kV()),
    units::unit_t<frc::ArmFeedforward::ka_unit>(GetAxis1kA()));
  m_axis1TrapezoidalProfileConstraints = new frc::TrapezoidProfile<units::degree>::Constraints(
    units::degrees_per_second_t(GetAxis1MaxVelocity()),
    units::degrees_per_second_squared_t(GetAxis1MaxAcceletration()));
  frc::DataLogManager::Log(fmt::format(
    "Configured Axis1 with parameters: P={}, I={}, D={}, CurrentLimit={}, MinOutput={}, "
    "MaxOutput={}, ArmFeedForward=[kS={}, kG={}, kV={}, kA={}], MaxVelocity={}, MaxAcceleration={}",
    GetAxis1P(),
    GetAxis1I(),
    GetAxis1D(),
    GetAxis1CurrentLimit(),
    kAxis1MinOutput,
    kAxis1MaxOutput,
    GetAxis1kS(),
    GetAxis1kG(),
    GetAxis1kV(),
    GetAxis1kA(),
    GetAxis1MaxVelocity(),
    GetAxis1MaxAcceletration()));

  // Axis 2
  m_axis2Motor.RestoreFactoryDefaults();
  m_axis2Motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_axis2Motor.SetInverted(false);
  m_axis2Motor.SetSmartCurrentLimit(GetAxis2CurrentLimit());
  m_axis2PIDController.SetP(GetAxis2P());
  m_axis2PIDController.SetI(GetAxis2I());
  m_axis2PIDController.SetD(GetAxis2D());
  m_axis2PIDController.SetOutputRange(kAxis2MinOutput, kAxis2MaxOutput);
  m_axis2Motor.BurnFlash();
  m_axis2TrapezoidalProfileConstraints = new frc::TrapezoidProfile<units::meter>::Constraints(
    units::meters_per_second_t(GetAxis2MaxVelocity() * 0.0254),
    units::meters_per_second_squared_t(GetAxis2MaxAcceletration() * 0.0254));
  frc::DataLogManager::Log(fmt::format(
    "Configured Axis2 with parameters: P={}, I={}, D={}, CurrentLimit={}, MinOutput={}, "
    "MaxOutput={},  MaxVelocity={}, MaxAcceleration={}",
    GetAxis2P(),
    GetAxis2I(),
    GetAxis2D(),
    GetAxis2CurrentLimit(),
    kAxis2MinOutput,
    kAxis2MaxOutput,
    GetAxis2MaxVelocity(),
    GetAxis2MaxAcceletration()));

  // Axis 3
  m_axis3Motor.ConfigFactoryDefault(50);
  m_axis3Motor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_axis3Motor.ConfigPeakCurrentLimit(GetAxis3CurrentLimit());
  m_axis3Motor.ConfigPeakCurrentDuration(GetAxis3CurrentLimitDuration());
  m_axis3Motor.ConfigContinuousCurrentLimit(GetAxis3CurrentLimit());
  m_axis3Motor.EnableCurrentLimit(true);
  m_axis3PIDController.SetP(GetAxis3P());
  m_axis3PIDController.SetI(GetAxis3I());
  m_axis3PIDController.SetD(GetAxis3D());
  frc::DataLogManager::Log(fmt::format(
    "Configured Axis2 with parameters: P={}, I={}, D={}, CurrentLimit={}, CurrentLimitDuration={}",
    GetAxis3P(),
    GetAxis3I(),
    GetAxis3D(),
    GetAxis3CurrentLimit(),
    GetAxis3CurrentLimitDuration()));
}

// This method will be called once per scheduler run
void ExtendingArmSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Axis1Angle", GetAxis1Angle().value());
  frc::SmartDashboard::PutNumber("Axis2Distance", GetAxis2Distance().value());
  frc::SmartDashboard::PutNumber("Axis3Angle", GetAxis3Angle().value());
  frc::SmartDashboard::PutNumber("Axis1Current", GetAxis1Current());
  frc::SmartDashboard::PutNumber("Axis2Current", GetAxis2Current());
  frc::SmartDashboard::PutNumber("Axis3Current", GetAxis3Current());
  frc::SmartDashboard::PutBoolean("Axis1HomeLimitSwitch", GetAxis1LimitSwitch());

  // Axis 1 & 2.
  units::degree_t axis1ProfileSetPoint = m_axis1SetPoint;
  units::inch_t axis2ProfileSetPoint = m_axis2SetPoint;

  if (GetAxis1Angle().value() < kPastBumperAngle) {
    axis2ProfileSetPoint = units::inch_t(0);
  }

  if (
    axis1ProfileSetPoint.value() < kPastBumperAngle &&
    GetAxis2Distance().value() > kAxis2ZeroDeadBand) {
    axis1ProfileSetPoint = units::degree_t(kPastBumperAngle);
  }

  if (m_isAxis1ControlEnabled) {
    frc::TrapezoidProfile<units::degree> axis1Profile {
      ExtendingArmSubsystem::GetAxis1Profile(),
      frc::TrapezoidProfile<units::degree>::State { axis1ProfileSetPoint,
                                                    units::degrees_per_second_t(0) },
      m_axis1TrapezoidalProfileState
    };

    m_axis1TrapezoidalProfileState = axis1Profile.Calculate(ExtendingArmConstants::kDt);
    ExtendingArmSubsystem::SetAxis1NextSetpoint(m_axis1TrapezoidalProfileState.position);

    if (m_isAxis2ControlEnabled) {
      frc::TrapezoidProfile<units::meters> axis2Profile {
        *m_axis2TrapezoidalProfileConstraints,
        frc::TrapezoidProfile<units::meters>::State { axis2ProfileSetPoint, 0_mps },
        m_axis2TrapezoidalProfileState
      };

      m_axis2TrapezoidalProfileState = axis2Profile.Calculate(ExtendingArmConstants::kDt);
      ExtendingArmSubsystem::SetAxis2NextSetpoint(m_axis2TrapezoidalProfileState.position);
    }
  }

  // Run axis 3 PID if the axis isn't disabled.
  if (m_axis3Motor.GetControlMode() != ctre::phoenix::motorcontrol::ControlMode::Disabled) {
    m_axis3Motor.Set(
      ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
      m_axis3PIDController.Calculate(ExtendingArmSubsystem::GetAxis3Angle().value()));
  }
}

void ExtendingArmSubsystem::OnDisable() {
  m_isAxis1ControlEnabled = false;
  m_isAxis2ControlEnabled = false;
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

frc::TrapezoidProfile<units::degree>::Constraints ExtendingArmSubsystem::GetAxis1Profile() {
  return *m_axis1TrapezoidalProfileConstraints;
}

frc2::CommandPtr ExtendingArmSubsystem::MoveArm(units::degree_t angle, units::inch_t distance) {
  return frc2::cmd::Sequence(
    MoveAxis1Command(this, units::degree_t(angle)).ToPtr(),
    MoveAxis2Command(this, units::inch_t(distance)).ToPtr());
  // return frc2::cmd::Either(
  //   frc2::cmd::Sequence(
  //     MoveAxis1Command(this, units::degree_t(angle)).ToPtr(),
  //     MoveAxis2Command(this, units::inch_t(distance)).ToPtr()),
  //   frc2::cmd::Sequence(
  //     MoveAxis2Command(this, units::inch_t(distance)).ToPtr(),
  //     MoveAxis1Command(this, units::degree_t(angle)).ToPtr()),
  //   [this] { return units::degree_t(1) > GetAxis1Angle(); });
}

void ExtendingArmSubsystem::MoveArmSimple(units::degree_t angle, units::inch_t distance) {
  m_axis1SetPoint = angle;
  m_axis2SetPoint = distance;
  m_isAxis1ControlEnabled = true;
  m_isAxis2ControlEnabled = true;

  // Reset our initial profile states.
  m_axis1TrapezoidalProfileState = frc::TrapezoidProfile<units::degree>::State {
    GetAxis1Angle(), units::degrees_per_second_t(m_axis1Encoder.GetVelocity())
  };
  m_axis2TrapezoidalProfileState =
    frc::TrapezoidProfile<units::meters>::State { GetAxis2Distance(), 0_mps };
}

void ExtendingArmSubsystem::DisableControlAxis2() {
  m_isAxis2ControlEnabled = false;
}