#include "commands/DriveTrajectoryCommand.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/SwerveControllerCommand.h>

/**
 * Generates trajectory with configuration that includes
 * swerve drive kinematics.
 */
frc::Trajectory createTrajectory(
  const frc::Pose2d& start,
  const std::vector<frc::Translation2d>& interiorWaypoints,
  const frc::Pose2d& end,
  DriveSubsystem* drive) {
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  // add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(drive->kDriveKinematics);
  return frc::TrajectoryGenerator::GenerateTrajectory(start, interiorWaypoints, end, config);
}

/**
 * Creates profiled PID controller for tracking angle.
 */
frc::ProfiledPIDController<units::radians> createThetaController() {
  frc::ProfiledPIDController<units::radians> controller {
    AutoConstants::kPThetaController, 0, 0, AutoConstants::kThetaControllerConstraints
  };

  controller.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });

  return controller;
}

DriveTrajectoryCommand::DriveTrajectoryCommand(
  const frc::Pose2d& start,
  const std::vector<frc::Translation2d>& interiorWaypoints,
  const frc::Pose2d& end,
  DriveSubsystem* drive) :
  CommandHelper {
    createTrajectory(start, interiorWaypoints, end, drive),
    [drive]() { return drive->GetPose(); },
    drive->kDriveKinematics,
    frc2::PIDController { AutoConstants::kPXController, 0, 0 },
    frc2::PIDController { AutoConstants::kPYController, 0, 0 },
    createThetaController(),
    [drive](auto moduleStates) { drive->SetModuleStates(moduleStates); },
    { drive },
  },
  m_drive { drive },
  m_start { start } {
  AddRequirements({ drive });
}

void DriveTrajectoryCommand::Initialize() {
  // reset odometry to the starting pose of the trajectory
  m_drive->ResetOdometry(m_start);

  // call base initialize
  CommandHelper<frc2::SwerveControllerCommand<NUMBER_OF_MODULES>, DriveTrajectoryCommand>::
    Initialize();
}
