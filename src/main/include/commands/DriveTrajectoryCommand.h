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
    bool resetOdometry,
    bool stopByTimer,
    bool printTrajectory,
    DriveSubsystem* drive);

  DriveTrajectoryCommand(
    const std::vector<Waypoint>& waypoints,
    const frc::Translation2d& origin,
    bool resetOdometry,
    bool stopByTimer,
    DriveSubsystem* drive) :
    DriveTrajectoryCommand(waypoints, origin, resetOdometry, stopByTimer, false, drive) {}

  DriveTrajectoryCommand(
    const std::vector<Waypoint>& waypoints,
    const frc::Translation2d& origin,
    bool resetOdometry,
    DriveSubsystem* drive) :
    DriveTrajectoryCommand(waypoints, origin, resetOdometry, false, false, drive) {}

  DriveTrajectoryCommand(
    const std::vector<Waypoint>& waypoints,
    const frc::Translation2d& origin,
    DriveSubsystem* drive) :
    DriveTrajectoryCommand(waypoints, origin, false, false, false, drive) {}

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  bool AtTarget() const;

  DriveSubsystem* m_drive;
  WaypointTrajectory m_trajectory;
  CustomHolonomicDriveController m_controller;
  frc::Pose2d m_finalPose;
  const bool m_resetOdometry;
  const bool m_stopByTimer;
  frc::Timer m_timer;
};
