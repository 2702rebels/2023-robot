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
class MoveAxis1Command : public frc2::CommandHelper<frc2::CommandBase, MoveAxis1Command> {
 public:
  MoveAxis1Command(ExtendingArmSubsystem* arm, units::degree_t axis1Angle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ExtendingArmSubsystem* m_arm;
  units::degree_t m_axisGoal;
  frc::TrapezoidProfile<units::degree>* m_profile;
  units::millisecond_t m_profileTime { 0_ms };
};
