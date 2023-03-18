#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <optional>

class Waypoint {
 public:
  /**
   * Creates a waypoint from the pose and holonomic rotation.
   */
  Waypoint(const frc::Pose2d& pose, const frc::Rotation2d& rotation) :
    position(pose.Translation()),
    direction(pose.Rotation()),
    rotation(rotation) {}

  /**
   * Creates a waypoint from the pose without holonomic rotation.
   */
  Waypoint(const frc::Pose2d& pose) :
    position(pose.Translation()),
    direction(pose.Rotation()),
    rotation(std::nullopt) {}

  /**
   * Creates a waypoint from the translation and direction without holonomic rotation.
   */
  Waypoint(const frc::Translation2d& position, const frc::Rotation2d& rotation) :
    position(position),
    direction(rotation),
    rotation(std::nullopt) {}

  /**
   * Creates a waypoint from the position without heading or holonomic rotation.
   */
  Waypoint(const frc::Translation2d& position) :
    position(position),
    direction(std::nullopt),
    rotation(std::nullopt) {}

  /**
   * Translation of the robot.
   */
  const frc::Translation2d position;

  /**
   * Directional heading of the robot.
   */
  const std::optional<const frc::Rotation2d> direction;

  /**
   * Holonomic rotation.
   */
  const std::optional<const frc::Rotation2d> rotation;
};
