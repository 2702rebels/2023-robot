#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

class DriveRotateCommand : public frc2::CommandHelper<frc2::CommandBase, DriveRotateCommand> {
 public:
  DriveRotateCommand(const frc::Rotation2d& desiredRotation, DriveSubsystem* drive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  frc::Rotation2d m_desiredRotation;
  frc::ProfiledPIDController<units::radians> m_controller;
  frc::Timer m_timer;
  units::degree_t m_lastRotation;
  units::second_t m_lastRotationAt;
};
