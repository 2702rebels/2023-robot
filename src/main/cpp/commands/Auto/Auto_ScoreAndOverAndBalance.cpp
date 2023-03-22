#include "commands/auto/Auto_ScoreAndOverAndBalance.h"

#include "commands/AutoBalanceCommand.h"
#include "commands/DriveOverBridgeCommand.h"
#include "frc2/command/InstantCommand.h"

Auto_ScoreAndOverAndBalance::Auto_ScoreAndOverAndBalance(
  DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
  AddCommands(DriveOverBridgeCommand(drive), AutoBalanceCommand(drive));
}
