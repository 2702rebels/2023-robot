#include "commands/Auto1.h"

#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

Auto1::Auto1(DriveSubsystem* drive) {
  AddCommands(
    DriveTrajectoryCommand(
      {
        frc::Pose2d { 0_m, 0_m, 0_deg },
        frc::Pose2d { 1_m, 0_m, 0_deg },
        frc::Pose2d { 2_m, 0_m, 0_deg },
        frc::Pose2d { 3_m, 0_m, 0_deg },
        frc::Pose2d { 4_m, 0_m, 0_deg },
      },
      false,
      true,
      drive),
    DriveTrajectoryCommand(
      {
        frc::Pose2d { 4_m, 0_m, 0_deg },
        frc::Pose2d { 3_m, 0_m, 0_deg },
        frc::Pose2d { 2_m, 0_m, 0_deg },
        frc::Pose2d { 1_m, 0_m, 0_deg },
        frc::Pose2d { 0_m, 0_m, 0_deg },
      },
      false,
      false,
      drive));
}
