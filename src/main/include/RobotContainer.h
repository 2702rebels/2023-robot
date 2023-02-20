#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DoubleJointedArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ExtendingArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.
 * Since command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void OnDisable();

 private:
  // The driver's controller
  frc::XboxController m_driverController { OIConstants::kDriverControllerPort };

  // The robot's subsystems
  DriveSubsystem m_drive;

#ifdef EXTENDING_ARM_ENABLED
  ExtendingArmSubsystem m_exArm;
#endif

#ifdef DOUBLE_JOINTED_ARM_ENABLED
  DoubleJointedArmSubsystem m_djArm;
#endif

#ifdef INTAKE_ENABLED
  IntakeSubsystem m_intake;
#endif

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
