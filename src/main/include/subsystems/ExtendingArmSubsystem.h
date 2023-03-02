// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Constants.h>
#include <ctre/Phoenix.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "subsystems/IntakeSubsystem.h"

enum ExtendingArmPositions {
  STOWED,
  INTAKE_FLOOR_CONE,
  INTAKE_FLOOR_CUBE,
  INTAKE_STATION_CONE,
  INTAKE_STATION_CUBE,
  SCORE_LOW_CONE,
  SCORE_LOW_CUBE,
  SCORE_MID_CONE,
  SCORE_MID_CUBE,
  SCORE_HIGH_CONE,
  SCORE_HIGH_CUBE,
};

class ExtendingArmSubsystem : public frc2::SubsystemBase {
 public:
  ExtendingArmSubsystem();
  void ConfigureTuningParams();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void OnDisable();

  units::degree_t GetAxis1Angle();
  units::degrees_per_second_t GetAxis1Velocity();
  units::inch_t GetAxis2Distance();
  units::meters_per_second_t GetAxis2Velocity();
  units::degree_t GetAxis3Angle();

  bool GetAxis1LimitSwitch();

  double GetAxis1Current();
  double GetAxis2Current();
  double GetAxis3Current();

  void SetAxis1Off();
  void SetAxis2Off();
  void SetAxis3Off();
  void SetArmOff();

  void ZeroAxis1();
  void ZeroAxis2();
  void ZeroAxis3();

  void SetAxis1DutyCycle(double dutyCycle);
  void SetAxis2DutyCycle(double dutyCycle);
  void SetAxis3DutyCycle(double dutyCycle);

  frc2::CommandPtr Home();
  frc2::CommandPtr MoveArm(ExtendingArmPositions position);
  frc2::CommandPtr MoveArm(units::degree_t angle, units::inch_t distance);
  frc2::CommandPtr ScoreCone();
  frc2::CommandPtr ScoreCube();

  void ReloadConfigs();

  void DisableControlAxis2();

  void SetAxis3NextSetpoint(units::degree_t setPoint);

  void SetIntakeReference(IntakeSubsystem* intake) {
    m_intake = intake;
  }

  bool IsCompBot() {
    return m_whichRobotJumper.Get();
  }

 private:
  bool m_isAxis1ControlEnabled = false;
  bool m_isAxis2ControlEnabled = false;
  bool m_isAxis3ControlEnabled = false;

  IntakeSubsystem* m_intake;

  units::degree_t m_axis1SetPoint;
  units::inch_t m_axis2SetPoint;
  units::degree_t m_axis3SetPoint;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_axis1Motor { ExtendingArmConstants::kExtendingArmAxis1MotorControllerCanId,
                                  rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax m_axis2Motor { ExtendingArmConstants::kExtendingArmAxis2MotorControllerCanId,
                                  rev::CANSparkMax::MotorType::kBrushless };
  TalonSRX m_axis3Motor { ExtendingArmConstants::kExtendingArmAxis3MotorControllerCanId };

  rev::SparkMaxPIDController m_axis1PIDController = m_axis1Motor.GetPIDController();
  rev::SparkMaxPIDController m_axis2PIDController = m_axis2Motor.GetPIDController();
  frc2::PIDController m_axis3PIDController { 0, 0, 0 };

  rev::SparkMaxRelativeEncoder m_axis1Encoder = m_axis1Motor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_axis2Encoder = m_axis2Motor.GetEncoder();
  frc::Encoder m_axis3Encoder { ExtendingArmConstants::kAxis3EncoderChannel1Id,
                                ExtendingArmConstants::kAxis3EncoderChannel2Id };

  frc::DigitalInput m_axis1HomeLimitSwitch { ExtendingArmConstants::kAxis1HomeLimitSwitchId };
  frc::DigitalInput m_whichRobotJumper { ExtendingArmConstants::kWhichRobotJumper };

  frc::ArmFeedforward* m_axis1feedforward;
  // frc::TrapezoidProfile<units::degree>* m_axis1TrapezoidalProfile;
  frc::TrapezoidProfile<units::degree>::Constraints* m_axis1TrapezoidalProfileConstraints;
  frc::TrapezoidProfile<units::degree>::State m_axis1TrapezoidalProfileState;

  // frc::TrapezoidProfile<units::meters>* m_axis2TrapezoidalProfile;
  frc::TrapezoidProfile<units::meters>::Constraints* m_axis2TrapezoidalProfileConstraints;
  frc::TrapezoidProfile<units::meters>::State m_axis2TrapezoidalProfileState;

  void SetAxis1NextSetpoint(units::degree_t setPoint);
  void SetAxis2NextSetpoint(units::inch_t setPoint);
  // void SetAxis3NextSetpoint(units::degree_t setPoint);

  void MoveArmSimple(units::degree_t angle, units::inch_t distance);
  bool IsArmAtCommandedPosition();

  double debug_calcVal = 0;
  double debug_setpntVal = 0;
  double debug_angleVal = 0;
};
