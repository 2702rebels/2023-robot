#include "commands/DriveTrajectoryCommand.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/SwerveControllerCommand.h>

/**
 * Generates trajectory (cubic hermite spline) taking into account
 * drive kinematics and maximum speed and acceleration constraints.
 */
frc::Trajectory createTrajectory(
  const frc::Pose2d& start,
  const std::vector<frc::Translation2d>& interiorWaypoints,
  const frc::Pose2d& end,
  bool reversed,
  DriveSubsystem* drive) {
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  if (reversed) {
    config.SetReversed(true);
  }

  // add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(drive->kDriveKinematics);
  return frc::TrajectoryGenerator::GenerateTrajectory(start, interiorWaypoints, end, config);
}

/**
 * Generates trajectory (quintic hermite spline) taking into account
 * drive kinematics and maximum speed and acceleration constraints.
 */
frc::Trajectory createTrajectory(
  const std::vector<frc::Pose2d>& waypoints, bool reversed, DriveSubsystem* drive) {
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  if (reversed) {
    config.SetReversed(true);
  }

  // add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(drive->kDriveKinematics);
  return frc::TrajectoryGenerator::GenerateTrajectory(waypoints, config);
}

/**
 * Creates profiled PID controller for tracking angle.
 */
frc::ProfiledPIDController<units::radians> createThetaController() {
  frc::ProfiledPIDController<units::radians> controller {
    AutoConstants::kPThetaController,
    AutoConstants::kIThetaController,
    AutoConstants::kDThetaController,
    AutoConstants::kThetaControllerConstraints,
  };

  controller.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });

  return controller;
}

DriveTrajectoryCommand::DriveTrajectoryCommand(
  const frc::Pose2d& start,
  const std::vector<frc::Translation2d>& interiorWaypoints,
  const frc::Pose2d& end,
  const frc::Rotation2d& rotation,
  bool reversed,
  bool resetOdometry,
  DriveSubsystem* drive) :
  CommandHelper {
    createTrajectory(start, interiorWaypoints, end, reversed, drive),
    [drive]() { return drive->GetPose(); },
    drive->kDriveKinematics,
    frc2::PIDController {
      AutoConstants::kPXController,
      AutoConstants::kIXController,
      AutoConstants::kDXController,
    },
    frc2::PIDController {
      AutoConstants::kPYController,
      AutoConstants::kIYController,
      AutoConstants::kDYController,
    },
    createThetaController(),
    [rotation]() { return rotation; },
    [drive](auto moduleStates) { drive->SetModuleStates(moduleStates); },
    { drive },
  },
  m_drive { drive },
  m_start { start },
  m_resetOdometry { resetOdometry } {
  AddRequirements({ drive });
}

DriveTrajectoryCommand::DriveTrajectoryCommand(
  const std::vector<frc::Pose2d>& waypoints,
  const frc::Rotation2d& rotation,
  bool reversed,
  bool resetOdometry,
  DriveSubsystem* drive) :
  CommandHelper {
    createTrajectory(waypoints, reversed, drive),
    [drive]() { return drive->GetPose(); },
    drive->kDriveKinematics,
    frc2::PIDController {
      AutoConstants::kPXController,
      AutoConstants::kIXController,
      AutoConstants::kDXController,
    },
    frc2::PIDController {
      AutoConstants::kPYController,
      AutoConstants::kIYController,
      AutoConstants::kDYController,
    },
    createThetaController(),
    [rotation]() { return rotation; },
    [drive](auto moduleStates) { drive->SetModuleStates(moduleStates); },
    { drive },
  },
  m_drive { drive },
  m_start { waypoints.front() },
  m_resetOdometry { resetOdometry } {
  AddRequirements({ drive });
}

void DriveTrajectoryCommand::Initialize() {
  // reset odometry to the starting pose of the trajectory
  if (m_resetOdometry) {
    m_drive->ResetOdometry(m_start);
  }

  // call base
  CommandHelper<frc2::SwerveControllerCommand<NUMBER_OF_MODULES>, DriveTrajectoryCommand>::
    Initialize();
}

void DriveTrajectoryCommand::End(bool interrupted) {
  // call base
  CommandHelper<frc2::SwerveControllerCommand<NUMBER_OF_MODULES>, DriveTrajectoryCommand>::End(
    interrupted);

  // stop the robot at the end of the trajectory
  m_drive->Stop();
}
