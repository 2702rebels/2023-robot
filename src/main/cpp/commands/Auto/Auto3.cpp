#include "commands/auto/Auto3.h"

#include "commands/AutoBalanceCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

Auto3::Auto3(DriveSubsystem* drive, ArmSubsystem* arm) {
  AddCommands(AutoBalanceCommand(drive));
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
  //   drive));
}
