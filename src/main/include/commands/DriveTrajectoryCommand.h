#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SwerveControllerCommand.h>

#include "subsystems/DriveSubsystem.h"

class DriveTrajectoryCommand
  : public frc2::
      CommandHelper<frc2::SwerveControllerCommand<NUMBER_OF_MODULES>, DriveTrajectoryCommand> {
 public:
  DriveTrajectoryCommand(
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end,
    DriveSubsystem* drive);

  void Initialize() override;

 private:
  DriveSubsystem* m_drive;
  frc::Pose2d m_start;
};
