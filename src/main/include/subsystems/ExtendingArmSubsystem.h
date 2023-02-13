// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Constants.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

using namespace ExtendingArmConstants;

class ExtendingArmSubsystem : public frc2::SubsystemBase {
 public:
  ExtendingArmSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  double GetAxis1Angle();
  double GetAxis2Distance();
  double GetAxis3Angle();

  /**
   * Set the angle in degrees that axis 1 should go to and maintain.
   * @param angle Angle in degrees.
   */
  void SetAxis1Angle(double angle);

  void SetAxis1Off();

  void ZeroAxis1();
  void ZeroAxis2();

  /**
   * Set the distance in inches that axis 2 should travel to and maintain.
   * @param distance Distance in inches.
   */
  void SetAxis2Distance(double distance);
  void SetAxis3Angle(double angle);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_axis1Motor { kExtendingArmAxis1MotorControllerCanId,
                                  rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax m_axis2Motor { kExtendingArmAxis2MotorControllerCanId,
                                  rev::CANSparkMax::MotorType::kBrushless };
  // rev::CANSparkMax m_axis3Motor{kExtendingArmAxis3MotorControllerCanId,
  // rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController m_axis1PIDController = m_axis1Motor.GetPIDController();
  rev::SparkMaxPIDController m_axis2PIDController = m_axis2Motor.GetPIDController();
  // rev::SparkMaxPIDController m_axis3PIDController = m_axis3Motor.GetPIDController();

  rev::SparkMaxRelativeEncoder m_axis1Encoder = m_axis1Motor.GetEncoder();
  rev::SparkMaxRelativeEncoder m_axis2Encoder = m_axis2Motor.GetEncoder();
  // rev::SparkMaxRelativeEncoder m_axis3Encoder = m_axis3Motor.GetEncoder();
};
