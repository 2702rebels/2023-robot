// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

using namespace IntakeConstants;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetIntakeSpeed(double speed);
  void StartIntaking();
  void StopIntaking();
  void StartOuttaking();
  void StopOuttaking();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  bool m_isHasGamePiece = false;

  TalonSRX m_leaderMotorController { kIntakeLeaderControllerCanId };
  VictorSPX m_followerMotorController { kIntakeFollowerControllerCanId };
};
