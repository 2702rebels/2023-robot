#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SwerveControllerCommand.h>

#include "subsystems/DriveSubsystem.h"

class DriveTrajectoryCommand
  : public frc2::
      CommandHelper<frc2::SwerveControllerCommand<NUMBER_OF_MODULES>, DriveTrajectoryCommand> {
 public:
  /**
   * Drives robot via trajectory of specified start, end and interior waypoints,
   * uses clamped cubic hermite spline trajectory generation.
   * Requires at least two interior waypoints.
   *
   * Set `reversed` if the robot facing backwards, the initial pose should also
   * be set to 180 degree rotation. Note that the trajectory is still followed
   * in the same direction.
   *
   * @param start Start pose of the robot
   * @param interiorWaypoints Interior waypoints (positions only)
   * @param end End pose of the robot
   * @param rotation Final pose rotation (heading)
   * @param reversed Indicates that robot is facing backwards
   * @param resetOdometry Indicates that odometry should be reset to the initial pose
   */
  DriveTrajectoryCommand(
    const frc::Pose2d& start,
    const std::vector<frc::Translation2d>& interiorWaypoints,
    const frc::Pose2d& end,
    const frc::Rotation2d& rotation,
    bool reversed,
    bool resetOdometry,
    DriveSubsystem* drive);

  /**
   * Drives robot via trajectory of specified waypoints,
   * uses quintic hermite spline trajectory generation.
   *
   * @param waypoints Waypoints, each should be a complete pose with orientation
   * @param rotation Final pose rotation (heading)
   * @param reversed Indicates that robot is facing backwards
   * @param resetOdometry Indicates that odometry should be reset to the initial pose
   */
  DriveTrajectoryCommand(
    const std::vector<frc::Pose2d>& waypoints,
    const frc::Rotation2d& rotation,
    bool reversed,
    bool resetOdometry,
    DriveSubsystem* drive);

  void Initialize() override;
  void End(bool interrupted) override;

 private:
  DriveSubsystem* m_drive;
  frc::Pose2d m_start;
  const bool m_resetOdometry;
};
