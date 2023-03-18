#pragma once

#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"
#include "utils/CustomHolonomicDriveController.h"
#include "utils/Waypoint.h"
#include "utils/WaypointTrajectory.h"

class DriveTrajectoryCommand
  : public frc2::CommandHelper<frc2::CommandBase, DriveTrajectoryCommand> {
 public:
  DriveTrajectoryCommand(
    const std::vector<Waypoint>& waypoints,
    const frc::Translation2d& origin,
    bool reversed,
    bool resetOdometry,
    DriveSubsystem* drive);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  WaypointTrajectory m_trajectory;
  CustomHolonomicDriveController m_controller;
  const bool m_resetOdometry;
  frc::Timer m_timer;
};
