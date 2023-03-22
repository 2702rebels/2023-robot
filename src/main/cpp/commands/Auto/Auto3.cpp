#include "commands/auto/Auto3.h"

#include "commands/AutoBalanceCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

constexpr auto origin = frc::Translation2d { 0_m, 0_m };

Auto3::Auto3(DriveSubsystem* drive, ArmSubsystem* arm) {
  AddCommands(
    DriveTrajectoryCommand(
      {
        Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
        Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }),
        Waypoint(frc::Pose2d { 2_m, 0_m, 0_deg }),
        Waypoint(frc::Pose2d { 3_m, 0_m, 0_deg }, 1.2_mps),
        Waypoint(frc::Pose2d { 4.6_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
      },
      origin,
      true,
      false,
      drive),
    DriveTrajectoryCommand(
      {
        Waypoint(frc::Pose2d { 4.6_m, 0_m, 180_deg }, frc::Rotation2d(0_deg)),
        Waypoint(frc::Pose2d { 3.6_m, 0_m, 180_deg }),
        Waypoint(frc::Pose2d { 2.6_m, 0_m, 180_deg }),
        Waypoint(frc::Pose2d { 1.6_m, 0_m, 180_deg }, 1.2_mps),
        Waypoint(frc::Pose2d { 0_m, 0_m, 180_deg }, frc::Rotation2d(180_deg)),
      },
      origin,
      false,
      false,
      drive),
    DriveTrajectoryCommand(
      {
        Waypoint(frc::Pose2d { 0_m, 0_m, 90_deg }, frc::Rotation2d(180_deg)),
        Waypoint(frc::Pose2d { 0_m, 1_m, 90_deg }),
        Waypoint(frc::Pose2d { 0.5_m, 2_m, 0_deg }, 1.2_mps),
        Waypoint(frc::Pose2d { 2_m, 2_m, 0_deg }, frc::Rotation2d(90_deg)),
      },
      origin,
      false,
      true,
      drive),
    AutoBalanceCommand(drive));
  // AddCommands(DriveTrajectoryCommand(
  //   {
  //     Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
  //     Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }),
  //     Waypoint(frc::Pose2d { 2_m, 0_m, 0_deg }),
  //     Waypoint(frc::Pose2d { 3_m, 0_m, 0_deg }, 1_mps),
  //     Waypoint(frc::Pose2d { 4.6_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
  //   },
  //   frc::Translation2d { 0_m, 0_m },
  //   true,
  //   false,
  //   drive));
}
