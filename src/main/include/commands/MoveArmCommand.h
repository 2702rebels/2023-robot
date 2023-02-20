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
class MoveArmCommand : public frc2::CommandHelper<frc2::CommandBase, MoveArmCommand> {
 public:
  MoveArmCommand(ExtendingArmSubsystem* arm, double axis1Angle, units::meter_t axis2Distance);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ExtendingArmSubsystem* m_arm;
  double m_axis1Goal;
  units::meter_t m_axis2Goal;
  frc::TrapezoidProfile<units::meters>* m_profile;
  //  {
  //   frc::TrapezoidProfile<units::meters>::Constraints { 5_mps, 10_mps_sq },
  //   frc::TrapezoidProfile<units::meters>::State { 5_m, 0_mps },
  //   frc::TrapezoidProfile<units::meters>::State { 0_m, 0_mps }
  // };
  units::millisecond_t m_profileTime { 0_ms };
};
