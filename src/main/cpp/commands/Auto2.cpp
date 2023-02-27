#include "commands/Auto2.h"

#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

Auto2::Auto2(DriveSubsystem* drive) {
  AddCommands(
    DriveTrajectoryCommand(
      // start at the origin facing the +X direction
      frc::Pose2d { 0_m, 0_m, 0_deg },
      // pass through these two interior waypoints, making an 's' curve path
      { frc::Translation2d { 1_m, 1_m }, frc::Translation2d { 2_m, -1_m } },
      // end 3 meters straight ahead of where we started, facing forward
      frc::Pose2d { 3_m, 0_m, 0_deg },
      false,
      true,
      drive),
    frc2::InstantCommand([drive] { drive->Stop(); }, { drive }));
}