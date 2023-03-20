#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/length.h>

#include "subsystems/DriveSubsystem.h"

class AutoBalanceCommand : public frc2::CommandHelper<frc2::CommandBase, AutoBalanceCommand> {
 public:
  AutoBalanceCommand(DriveSubsystem* drive);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  units::radian_t m_angle;
  units::radian_t m_angleThreshold;
  units::radians_per_second_t m_velocityThreshold;
  units::meters_per_second_t m_speed;
};
