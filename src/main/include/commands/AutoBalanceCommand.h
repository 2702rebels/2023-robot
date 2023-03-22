#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

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
  units::radians_per_second_t m_velocity;

  double m_factor;
  units::meters_per_second_t m_speed;
  units::radians_per_second_t m_thresholdV;
  units::radians_per_second_t m_stopV;
  units::radian_t m_stopAngle;
};
