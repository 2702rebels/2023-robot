// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

using namespace DoubleJointedArmConstants;

class DoubleJointedArmSubsystem : public frc2::SubsystemBase {
 public:
  DoubleJointedArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void OnDisable();

  void ZeroArm();
  void ZeroAxis1();
  void ZeroAxis2();
  void ZeroAxis3();

  void SetAxis1Angle(double angle);
  void SetAxis2Angle(double angle);
  void SetAxis3Angle(double angle);

  double GetAxis1Angle();
  double GetAxis2Angle();
  double GetAxis3Angle();

  void StopMotors();
  void SetAxis1Off();
  void SetAxis2Off();
  void SetAxis3Off();

  void MoveToPose(double x, double y);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax m_axis1MotorController { kDoubleJointedArmAxis1MotorControllerCanId,
                                            rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax m_axis2MotorController { kDoubleJointedArmAxis2MotorControllerCanId,
                                            rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax m_axis3MotorController { kDoubleJointedArmAxis3MotorControllerCanId,
                                            rev::CANSparkMax::MotorType::kBrushless };

  rev::SparkMaxPIDController m_axis1PIDController { m_axis1MotorController.GetPIDController() };
  rev::SparkMaxPIDController m_axis2PIDController { m_axis2MotorController.GetPIDController() };
  rev::SparkMaxPIDController m_axis3PIDController { m_axis3MotorController.GetPIDController() };

  rev::SparkMaxRelativeEncoder m_axis1Encoder { m_axis1MotorController.GetEncoder() };
  rev::SparkMaxRelativeEncoder m_axis2Encoder { m_axis2MotorController.GetEncoder() };
  rev::SparkMaxRelativeEncoder m_axis3Encoder { m_axis3MotorController.GetEncoder() };
};
