#include "commands/auto/Auto2.h"

#include "commands/AutoBalanceCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

constexpr auto initialPose = frc::Pose2d { 0_m, 0_m, 90_deg };

Auto2::Auto2(DriveSubsystem* drive) {
  AddCommands(
    frc2::InstantCommand([drive] { drive->ResetOdometry(initialPose); }, {}),
    AutoBalanceCommand(drive, Side::LEFT));
}
