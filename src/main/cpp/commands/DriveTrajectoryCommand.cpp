#include "commands/DriveTrajectoryCommand.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>
#include <units/math.h>

#include "Settings.h"

// clang-format off
REGISTER_DOUBLE(AutoDriveTrajectoryXP, "Auto/DriveTrajectory/X - P", AutoConstants::kPXController);
REGISTER_DOUBLE(AutoDriveTrajectoryXI, "Auto/DriveTrajectory/X - I", AutoConstants::kIXController);
REGISTER_DOUBLE(AutoDriveTrajectoryXD, "Auto/DriveTrajectory/X - D", AutoConstants::kDXController);
REGISTER_DOUBLE(AutoDriveTrajectoryYP, "Auto/DriveTrajectory/Y - P", AutoConstants::kPYController);
REGISTER_DOUBLE(AutoDriveTrajectoryYI, "Auto/DriveTrajectory/Y - I", AutoConstants::kIYController);
REGISTER_DOUBLE(AutoDriveTrajectoryYD, "Auto/DriveTrajectory/Y - D", AutoConstants::kDYController);
REGISTER_DOUBLE(AutoDriveTrajectoryHP, "Auto/DriveTrajectory/Theta - P", AutoConstants::kPThetaController);
REGISTER_DOUBLE(AutoDriveTrajectoryHI, "Auto/DriveTrajectory/Theta - I", AutoConstants::kIThetaController);
REGISTER_DOUBLE(AutoDriveTrajectoryHD, "Auto/DriveTrajectory/Theta - D", AutoConstants::kDThetaController);
REGISTER_DOUBLE(AutoDriveTrajectoryHV, "Auto/DriveTrajectory/Theta - v", AutoConstants::kMaxAngularSpeed.value());
REGISTER_DOUBLE(AutoDriveTrajectoryHA, "Auto/DriveTrajectory/Theta - a", AutoConstants::kMaxAngularAcceleration.value());
REGISTER_DOUBLE(AutoDriveTrajectoryXYV, "Auto/DriveTrajectory/XY - v", AutoConstants::kMaxSpeed.value());
REGISTER_DOUBLE(AutoDriveTrajectoryXYA, "Auto/DriveTrajectory/XY - a", AutoConstants::kMaxAcceleration.value());

// clang-format on

DriveTrajectoryCommand::DriveTrajectoryCommand(
  const std::vector<Waypoint>& waypoints,
  const frc::Translation2d& origin,
  bool resetOdometry,
  DriveSubsystem* drive) :
  m_drive { drive },
  m_controller {
    frc2::PIDController {
      GetAutoDriveTrajectoryXP(),
      GetAutoDriveTrajectoryXI(),
      GetAutoDriveTrajectoryXD(),
    },
    frc2::PIDController {
      GetAutoDriveTrajectoryYP(),
      GetAutoDriveTrajectoryYI(),
      GetAutoDriveTrajectoryYD(),
    },
    frc::ProfiledPIDController<units::radian> {
      GetAutoDriveTrajectoryHP(),
      GetAutoDriveTrajectoryHI(),
      GetAutoDriveTrajectoryHD(),
      frc::TrapezoidProfile<units::radian>::Constraints {
        units::degrees_per_second_t { GetAutoDriveTrajectoryHV() },
        units::degrees_per_second_squared_t { GetAutoDriveTrajectoryHA() },
      },
    },
  },
  m_resetOdometry { resetOdometry } {
  AddRequirements(drive);

  // TODO: error handler
  m_trajectory = WaypointTrajectory(
    waypoints,
    origin,
    drive->kDriveKinematics,
    units::meters_per_second_t { GetAutoDriveTrajectoryXYV() },
    units::meters_per_second_squared_t { GetAutoDriveTrajectoryXYA() },
    units::degrees_per_second_t { GetAutoDriveTrajectoryHV() },
    units::degrees_per_second_squared_t { GetAutoDriveTrajectoryHA() });

  frc::DataLogManager::Log(fmt::format("DriveTrajectory: Trajectory"));
  for (const auto& state : m_trajectory.Trajectory().States()) {
    frc::DataLogManager::Log(fmt::format(
      "at {:.3f} -> ({:.2f}, {:.2f}, {:.1f}) r = {:.2f} v = {:.2f}",
      state.t.value(),
      state.pose.X().value(),
      state.pose.Y().value(),
      state.pose.Rotation().Degrees().value(),
      state.curvature.value(),
      state.velocity.value()));
  }

  frc::DataLogManager::Log(fmt::format("DriveTrajectory: Rotation Profile"));
  for (const auto& state : m_trajectory.Rotations().States()) {
    frc::DataLogManager::Log(fmt::format(
      "at {:.3f} -> {:.1f} v = {:.2f}",
      state.time.value(),
      state.rotation.Degrees().value(),
      state.velocity.value()));
  }
}

void DriveTrajectoryCommand::Initialize() {
  // reset odometry to the starting pose of the trajectory
  if (m_resetOdometry) {
    m_drive->ResetOdometry(m_trajectory.InitialPose());
  }

  m_timer.Restart();

  m_controller.getXController().SetP(GetAutoDriveTrajectoryXP());
  m_controller.getXController().SetI(GetAutoDriveTrajectoryXI());
  m_controller.getXController().SetD(GetAutoDriveTrajectoryXD());

  m_controller.getYController().SetP(GetAutoDriveTrajectoryYP());
  m_controller.getYController().SetI(GetAutoDriveTrajectoryYI());
  m_controller.getYController().SetD(GetAutoDriveTrajectoryYD());

  m_controller.getThetaController().SetP(GetAutoDriveTrajectoryHP());
  m_controller.getThetaController().SetI(GetAutoDriveTrajectoryHI());
  m_controller.getThetaController().SetD(GetAutoDriveTrajectoryHD());

  m_controller.getXController().Reset();
  m_controller.getYController().Reset();
  m_controller.getThetaController().Reset(m_drive->GetPose().Rotation().Radians());
}

void DriveTrajectoryCommand::Execute() {
  const auto time = m_timer.Get();

  const auto desiredState = m_trajectory.Trajectory().Sample(time);
  const auto desiredRotation = m_trajectory.Rotations().Sample(time);

  const auto speeds = m_controller.Calculate(m_drive->GetPose(), desiredState, desiredRotation);
  m_drive->SetChassisSpeeds(speeds);
}

void DriveTrajectoryCommand::End(bool interrupted) {
  m_timer.Stop();
  m_drive->Stop();
}

bool DriveTrajectoryCommand::IsFinished() {
  return m_timer.HasElapsed(m_trajectory.Trajectory().TotalTime());
}
