// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ExtendingArmSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MoveAxis2Command : public frc2::CommandHelper<frc2::CommandBase, MoveAxis2Command> {
 public:
  MoveAxis2Command(ExtendingArmSubsystem* arm, units::meter_t axis2Distance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ExtendingArmSubsystem* m_arm;
  units::meter_t m_axis2Goal;
  frc::TrapezoidProfile<units::meters>* m_profile;
  units::millisecond_t m_profileTime { 0_ms };
};
