#include "commands/auto/Auto2.h"

#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

Auto2::Auto2(DriveSubsystem* drive) {
  AddCommands(DriveTrajectoryCommand(
    {
      Waypoint(frc::Pose2d { 0_m, 0_m, 90_deg }, frc::Rotation2d(180_deg)),
      Waypoint(frc::Pose2d { 0_m, 1_m, 90_deg }),
      Waypoint(frc::Pose2d { 0.5_m, 2_m, 0_deg }, 1_mps),
      Waypoint(frc::Pose2d { 2_m, 2_m, 0_deg }, frc::Rotation2d(90_deg)),
    },
    frc::Translation2d { 0_m, 0_m },
    true,
    drive));
}