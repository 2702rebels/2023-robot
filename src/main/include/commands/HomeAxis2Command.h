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
class HomeAxis2Command : public frc2::CommandHelper<frc2::CommandBase, HomeAxis2Command> {
 public:
  HomeAxis2Command(ExtendingArmSubsystem* arm);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ExtendingArmSubsystem* m_arm;
  units::second_t m_homingStartTime;
  int m_cyclesAboveCurrentLimit = 0;
  bool m_isHomed = false;
  bool m_isTimedOut = false;
};
