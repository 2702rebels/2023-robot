// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ExtendingArmSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "Settings.h"

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

REGISTER_DOUBLE(ConeAxis1IntakeFloor, "ExArm/Position/Cone/IntakeFloor/Axis1", 18.0)
REGISTER_DOUBLE(ConeAxis2IntakeFloor, "ExArm/Position/Cone/IntakeFloor/Axis2", 10.0)
// REGISTER_DOUBLE(ConeAxis1IntakeStation, "ExArm/Position/Cone/IntakeStation/Axis1", 65.0)
// REGISTER_DOUBLE(ConeAxis2IntakeStation, "ExArm/Position/Cone/IntakeStation/Axis2", 5.0)

// REGISTER_DOUBLE(ConeAxis1ScoreLow, "ExArm/Position/Cone/ScoreLow/Axis1", 18.0)
// REGISTER_DOUBLE(ConeAxis2ScoreLow, "ExArm/Position/Cone/ScoreLow/Axis2", 0.0)
// REGISTER_DOUBLE(ConeAxis1ScoreMid, "ExArm/Position/Cone/ScoreMid/Axis1", 65.0)
// REGISTER_DOUBLE(ConeAxis2ScoreMid, "ExArm/Position/Cone/ScoreMid/Axis2", 7.0)
// REGISTER_DOUBLE(ConeAxis1ScoreHigh, "ExArm/Position/Cone/ScoreHigh/Axis1", 80.0)
// REGISTER_DOUBLE(ConeAxis2ScoreHigh, "ExArm/Position/Cone/ScoreHigh/Axis2", 20.0)

REGISTER_DOUBLE(CubeAxis1IntakeFloor, "ExArm/Position/Cube/IntakeFloor/Axis1", 18.0)
REGISTER_DOUBLE(CubeAxis2IntakeFloor, "ExArm/Position/Cube/IntakeFloor/Axis2", 10.0)

// REGISTER_DOUBLE(CubeAxis1IntakeStation, "ExArm/Position/Cube/IntakeStation/Axis1", 65.0)
// REGISTER_DOUBLE(CubeAxis2IntakeStation, "ExArm/Position/Cube/IntakeStation/Axis2", 5.0)

// REGISTER_DOUBLE(CubeAxis1ScoreLow, "ExArm/Position/Cube/ScoreLow/Axis1", 18.0)
// REGISTER_DOUBLE(CubeAxis2ScoreLow, "ExArm/Position/Cube/ScoreLow/Axis2", 0.0)
// REGISTER_DOUBLE(CubeAxis1ScoreMid, "ExArm/Position/Cube/ScoreMid/Axis1", 52.0)
// REGISTER_DOUBLE(CubeAxis2ScoreMid, "ExArm/Position/Cube/ScoreMid/Axis2", 0.0)
// REGISTER_DOUBLE(CubeAxis1ScoreHigh, "ExArm/Position/Cube/ScoreHigh/Axis1", 70.0)
// REGISTER_DOUBLE(CubeAxis2ScoreHigh, "ExArm/Position/Cube/ScoreHigh/Axis2", 10.0)

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
  // Conver to degree's per second.
  if (ExtendingArmSubsystem::IsCompBot()) {
    m_axis1Encoder.SetVelocityConversionFactor((1 / kAxis1MotorRotationsPerDegreeComp) / 60);
  } else {
    m_axis1Encoder.SetVelocityConversionFactor((1 / kAxis1MotorRotationsPerDegree) / 60);
  }
  m_axis1Motor.BurnFlash();
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
  // Convert to meters per second.
  m_axis2Encoder.SetVelocityConversionFactor((1 / kAxis2MotorRotationsPerMeterOfTravel) / 60);
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
  // m_axis3Motor.EnableCurrentLimit(true);
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
  frc::SmartDashboard::PutNumber("Axis1SetPoint", m_axis1SetPoint.value());
  frc::SmartDashboard::PutNumber("Axis2SetPoint", m_axis2SetPoint.value());
  frc::SmartDashboard::PutBoolean("IsComp", IsCompBot());

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

  // Axis 1 control loop.
  if (m_isAxis1ControlEnabled) {
    frc::TrapezoidProfile<units::degree> axis1Profile {
      *m_axis1TrapezoidalProfileConstraints,
      frc::TrapezoidProfile<units::degree>::State { axis1ProfileSetPoint,
                                                    units::degrees_per_second_t(0) },
      m_axis1TrapezoidalProfileState
    };

    m_axis1TrapezoidalProfileState = axis1Profile.Calculate(ExtendingArmConstants::kDt);
    ExtendingArmSubsystem::SetAxis1NextSetpoint(m_axis1TrapezoidalProfileState.position);
  }

  // Axis 2 control loop.
  if (m_isAxis2ControlEnabled) {
    frc::TrapezoidProfile<units::meters> axis2Profile {
      *m_axis2TrapezoidalProfileConstraints,
      frc::TrapezoidProfile<units::meters>::State { axis2ProfileSetPoint, 0_mps },
      m_axis2TrapezoidalProfileState
    };

    m_axis2TrapezoidalProfileState = axis2Profile.Calculate(ExtendingArmConstants::kDt);
    ExtendingArmSubsystem::SetAxis2NextSetpoint(m_axis2TrapezoidalProfileState.position);
  }

  // Axis 3 control loop.
  if (m_isAxis3ControlEnabled) {
    double calcVal = m_axis3PIDController.Calculate(ExtendingArmSubsystem::GetAxis3Angle().value());
    double setpntVal = m_axis3PIDController.GetSetpoint();
    double angleVal = ExtendingArmSubsystem::GetAxis3Angle().value();
    m_axis3Motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, calcVal);

    if (calcVal != debug_calcVal || setpntVal != debug_setpntVal || angleVal != debug_angleVal) {
      debug_calcVal = calcVal;
      debug_setpntVal = setpntVal;
      debug_angleVal = angleVal;
      frc::DataLogManager::Log(
        fmt::format("Axis3 Set Point: {} PID: {} Position: {}", setpntVal, calcVal, angleVal));
    }
  }
}

void ExtendingArmSubsystem::OnDisable() {
  m_isAxis1ControlEnabled = false;
  m_isAxis2ControlEnabled = false;
  m_isAxis3ControlEnabled = false;
  ExtendingArmSubsystem::SetArmOff();
}

units::degree_t ExtendingArmSubsystem::GetAxis1Angle() {
  if (ExtendingArmSubsystem::IsCompBot()) {
    return units::degree_t(m_axis1Encoder.GetPosition() / kAxis1MotorRotationsPerDegreeComp);
  } else {
    return units::degree_t(m_axis1Encoder.GetPosition() / kAxis1MotorRotationsPerDegree);
  }
}

units::degrees_per_second_t ExtendingArmSubsystem::GetAxis1Velocity() {
  return units::degrees_per_second_t(m_axis1Encoder.GetVelocity());
}

units::inch_t ExtendingArmSubsystem::GetAxis2Distance() {
  return units::inch_t(m_axis2Encoder.GetPosition() / kAxis2MotorRotationsPerInchOfTravel);
}

units::meters_per_second_t ExtendingArmSubsystem::GetAxis2Velocity() {
  return units::meters_per_second_t(m_axis2Encoder.GetVelocity());
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
  if (ExtendingArmSubsystem::IsCompBot()) {
    rotations = setPoint.value() * kAxis1MotorRotationsPerDegreeComp;
  }
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
        ExtendingArmSubsystem::GetAxis1Angle() - kaxis1ffAngleCorrection,
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
  double rotations = setPoint.value();
#ifdef VERBOSE_LOG
  frc::DataLogManager::Log(
    fmt::format("Axis3: Setting setpoint to: {}, Setting rotations to: {}", setPoint, rotations));
#endif
  m_axis3Motor.Set(
    ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
    m_axis3PIDController.Calculate(ExtendingArmSubsystem::GetAxis3Angle().value(), rotations));
  m_isAxis3ControlEnabled = true;
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
  frc::DataLogManager::Log(
    fmt::format("Axis3: Disabled motor by commanding motor control mode to 'Disabled'."));
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

void ExtendingArmSubsystem::MoveArmSimple(units::degree_t angle, units::inch_t distance) {
  // Ensure we aren't commanding the arm outside it's limit.
  if (angle > kAxis1MaxAngle) {
    m_axis1SetPoint = kAxis1MaxAngle;
  } else if (angle < units::degree_t(0)) {
    m_axis1SetPoint = units::degree_t(0);
  } else {
    m_axis1SetPoint = angle;
  }

  if (distance > kAxis2MaxExtension) {
    m_axis2SetPoint = kAxis2MaxExtension;
  } else if (distance < units::inch_t(0)) {
    m_axis2SetPoint = units::inch_t(0);
  } else {
    m_axis2SetPoint = distance;
  }

  m_isAxis1ControlEnabled = true;
  m_isAxis2ControlEnabled = true;

  // Reset our initial profile states.
  m_axis1TrapezoidalProfileState =
    frc::TrapezoidProfile<units::degree>::State { ExtendingArmSubsystem::GetAxis1Angle(),
                                                  ExtendingArmSubsystem::GetAxis1Velocity() };
  m_axis2TrapezoidalProfileState =
    frc::TrapezoidProfile<units::meters>::State { ExtendingArmSubsystem::GetAxis2Distance(),
                                                  ExtendingArmSubsystem::GetAxis2Velocity() };
}

frc2::CommandPtr ExtendingArmSubsystem::MoveArm(ExtendingArmPositions position) {
  switch (position) {
    case STOWED:
      return frc2::FunctionalCommand(
               // Initialize
               [this] { ExtendingArmSubsystem::MoveArmSimple(0.0_deg, 0.0_in); },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case INTAKE_FLOOR_CONE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] {
                 ExtendingArmSubsystem::MoveArmSimple(
                   units::degree_t(GetConeAxis1IntakeFloor()),
                   units::inch_t(GetConeAxis2IntakeFloor()));
               },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case INTAKE_FLOOR_CUBE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] {
                 ExtendingArmSubsystem::MoveArmSimple(
                   units::degree_t(GetCubeAxis1IntakeFloor()),
                   units::inch_t(GetCubeAxis2IntakeFloor()));
               },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case INTAKE_STATION_CONE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] { ExtendingArmSubsystem::MoveArmSimple(65.0_deg, 5.0_in); },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case INTAKE_STATION_CUBE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] { ExtendingArmSubsystem::MoveArmSimple(65.0_deg, 5.0_in); },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case SCORE_LOW_CONE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] { ExtendingArmSubsystem::MoveArmSimple(15.0_deg, 0.0_in); },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case SCORE_LOW_CUBE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] { ExtendingArmSubsystem::MoveArmSimple(15.0_deg, 0.0_in); },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case SCORE_MID_CONE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] { ExtendingArmSubsystem::MoveArmSimple(65.0_deg, 7.0_in); },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case SCORE_MID_CUBE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] { ExtendingArmSubsystem::MoveArmSimple(52.0_deg, 0.0_in); },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case SCORE_HIGH_CONE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] { ExtendingArmSubsystem::MoveArmSimple(80.0_deg, 20.0_in); },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
    case SCORE_HIGH_CUBE:
      return frc2::FunctionalCommand(
               // Initialize
               [this] { ExtendingArmSubsystem::MoveArmSimple(70.0_deg, 10.0_in); },
               // Execute
               [this] {},
               // End
               [this](bool interrupted) {},
               // IsFinished
               [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
               // Require subsystem
               { this })
        .ToPtr();
  }

  return frc2::InstantCommand(
           [this] { frc::DataLogManager::Log(fmt::format("Location not implemented.")); }, {})
    .ToPtr();
}

frc2::CommandPtr ExtendingArmSubsystem::MoveArm(units::degree_t angle, units::inch_t distance) {
  return frc2::FunctionalCommand(
           // Initialize
           [&] {
             ExtendingArmSubsystem::MoveArmSimple(angle, distance);
             frc::DataLogManager::Log(fmt::format(
               "Running MoveArm command with angle:{} and distance{}",
               angle.value(),
               distance.value()));
           },
           // Execute
           [this] {},
           // End
           [this](bool interrupted) {},
           // IsFinished
           [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
           // Require subsystem
           { this })
    .ToPtr();
}

frc2::CommandPtr ExtendingArmSubsystem::ScoreCone() {
  return frc2::cmd::Sequence(
    frc2::FunctionalCommand(
      // Initialize
      [this] {
        ExtendingArmSubsystem::MoveArmSimple(GetAxis1Angle() - 12.0_deg, GetAxis2Distance());
      },
      // Execute
      [this] {},
      // End
      [this](bool interrupted) {},
      // IsFinished
      [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
      // Require subsystem
      { this })
      .ToPtr(),
    frc2::InstantCommand([this] { m_intake->StartOuttaking(); }, { m_intake }),
    frc2::FunctionalCommand(
      // Initialize
      [this] { ExtendingArmSubsystem::MoveArmSimple(GetAxis1Angle(), 0.0_in); },
      // Execute
      [this] {},
      // End
      [this](bool interrupted) {},
      // IsFinished
      [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
      // Require subsystem
      { this })
      .ToPtr(),
    frc2::InstantCommand([this] { m_intake->StopOuttaking(); }, { m_intake })/*,
    frc2::FunctionalCommand(
      // Initialize
      [this] { ExtendingArmSubsystem::MoveArmSimple(0.0_deg, 0.0_in); },
      // Execute
      [this] {},
      // End
      [this](bool interrupted) {},
      // IsFinished
      [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
      // Require subsystem
      { this })
      .ToPtr()*/);
}

frc2::CommandPtr ExtendingArmSubsystem::ScoreCube() {
  return frc2::cmd::Sequence(
    frc2::InstantCommand([this] { m_intake->StartOuttaking(); }, { m_intake }),
    frc2::cmd::Parallel(
      frc2::FunctionalCommand(
        // Initialize
        [this] { ExtendingArmSubsystem::MoveArmSimple(GetAxis1Angle(), 0.0_in); },
        // Execute
        [this] {},
        // End
        [this](bool interrupted) {},
        // IsFinished
        [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
        // Require subsystem
        { this })
        .ToPtr(),
      frc2::WaitCommand(0.2_s).ToPtr()),
    frc2::InstantCommand([this] { m_intake->StopOuttaking(); }, { m_intake })/*,
    frc2::FunctionalCommand(
      // Initialize
      [this] { ExtendingArmSubsystem::MoveArmSimple(0.0_deg, 0.0_in); },
      // Execute
      [this] {},
      // End
      [this](bool interrupted) {},
      // IsFinished
      [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
      // Require subsystem
      { this })
      .ToPtr()*/);
}

bool ExtendingArmSubsystem::IsArmAtCommandedPosition() {
  double axis1 = GetAxis1Angle().value();
  double axis2 = GetAxis2Distance().value();
  double axis1SetPoint = m_axis1SetPoint.value();
  double axis2SetPoint = m_axis2SetPoint.value();

  if (
    (axis1 < axis1SetPoint + kAxis1Deadband && axis1 > axis1SetPoint - kAxis1Deadband) &&
    (axis2 < axis2SetPoint + kAxis2Deadband && axis2 > axis2SetPoint - kAxis2Deadband)) {
    return true;
  }

  return false;
}

void ExtendingArmSubsystem::DisableControlAxis2() {
  m_isAxis2ControlEnabled = false;
}

frc2::CommandPtr ExtendingArmSubsystem::JogUp() {
  return frc2::FunctionalCommand(
           // Initialize
           [this] {
             ExtendingArmSubsystem::MoveArmSimple(
               GetAxis1Angle() + units::degree_t(10), GetAxis2Distance());
           },
           // Execute
           [this] {},
           // End
           [this](bool interrupted) {},
           // IsFinished
           [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
           // Require subsystem
           { this })
    .ToPtr();
}

frc2::CommandPtr ExtendingArmSubsystem::JogDown() {
  return frc2::FunctionalCommand(
           // Initialize
           [this] {
             ExtendingArmSubsystem::MoveArmSimple(
               GetAxis1Angle() - units::degree_t(10), GetAxis2Distance());
           },
           // Execute
           [this] {},
           // End
           [this](bool interrupted) {},
           // IsFinished
           [this] { return ExtendingArmSubsystem::IsArmAtCommandedPosition(); },
           // Require subsystem
           { this })
    .ToPtr();
}