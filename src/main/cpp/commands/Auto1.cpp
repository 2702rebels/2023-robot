#include "commands/Auto1.h"

#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

Auto1::Auto1(DriveSubsystem* drive) {
  AddCommands(
    DriveTrajectoryCommand(
      frc::Pose2d { 0_m, 0_m, 0_deg },
      {
        frc::Translation2d { 1_m, 0_m },
        frc::Translation2d { 3_m, 0_m },
      },
      frc::Pose2d { 4_m, 0_m, 0_deg },
      drive),
    frc2::InstantCommand([drive] { drive->Stop(); }, { drive }),
    DriveTrajectoryCommand(
      frc::Pose2d { 0_m, 0_m, 0_deg },
      {
        frc::Translation2d { 1_m, -1_m },
        frc::Translation2d { 1_m, 0_m },
      },
      frc::Pose2d { -1_m, 0_m, 0_deg },
      drive),
    frc2::InstantCommand([drive] { drive->Stop(); }, { drive }));
}
