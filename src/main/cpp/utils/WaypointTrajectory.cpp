#include "utils/WaypointTrajectory.h"

#include <frc/trajectory/TrajectoryGenerator.h>

#include <algorithm>

WaypointTrajectory::WaypointTrajectory(
  const std::vector<Waypoint>& waypoints,
  const frc::Translation2d& origin,
  const frc::TrajectoryConfig& config,
  const frc::TrapezoidProfile<units::radians>::Constraints& constraints) {
  if (waypoints.size() < 2) {
    throw std::runtime_error("Trajectory requires at least 2 waypoints");
  }

  std::vector<frc::Pose2d> poses;

  // if directional heading is not specified for a position, auto-generate one
  // as the vector towards the next position, for the last waypoint retain
  // current (previous) heading
  unsigned int index = 0;
  for (const auto& waypoint : waypoints) {
    const auto position = waypoint.position + origin;

    if (waypoint.direction.has_value()) {
      poses.push_back({ position, waypoint.direction.value() });
    } else if (index + 1 < waypoints.size()) {
      const auto direction = (waypoints.at(index + 1).position - waypoint.position).Angle();
      poses.push_back({ position, direction });
    } else {
      const auto direction = (waypoint.position - waypoints.at(index - 1).position).Angle();
      poses.push_back({ position, direction });
    }

    ++index;
  }

  m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(poses, config);

  // now that we have constructed a drive trajectory we calculate a profile for
  // holonomic rotations based on the total time of the drive path and subject
  // to maximum velocity and acceleration constraints specified in the config
  const auto r0 = waypoints.front().rotation;
  const auto r1 = waypoints.back().rotation;

  m_rotationProfile = RotationProfile(
    r0.has_value() ? r0.value() : frc::Rotation2d(),
    r1.has_value() ? r1.value() : frc::Rotation2d(),
    m_trajectory.TotalTime(),
    constraints.maxVelocity,
    constraints.maxAcceleration);
}
