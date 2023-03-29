#include "commands/auto/Auto3.h"

#include "commands/AutoBalanceCommand.h"
#include "commands/DriveOverBridgeCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"

constexpr auto initialPose = frc::Pose2d { 0_m, 0_m, 180_deg };

Auto3::Auto3(DriveSubsystem* drive) {
  AddCommands(
    frc2::InstantCommand([drive] { drive->ResetOdometry(initialPose); }, {}),
    DriveOverBridgeCommand(drive),
    AutoBalanceCommand(drive, Side::RIGHT));
}
