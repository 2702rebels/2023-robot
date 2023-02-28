#include "commands/Auto3.h"

#include "commands/DriveRotateCommand.h"
#include "commands/DriveTrajectoryCommand.h"

Auto3::Auto3(DriveSubsystem* drive) {
  AddCommands(DriveRotateCommand(frc::Rotation2d { 180_deg }, drive));
}
