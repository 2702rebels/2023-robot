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
#include "commands/auto/Auto2.h"
#include "commands/auto/Auto3.h"
#include "commands/auto/Auto_DoNothing.h"
#include "commands/auto/Auto_DriveDistance.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LedSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.
 * Since command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

enum RobotMode { CONE, CUBE };

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void OnDisable();
  void OnAutoEnable();
  void OnTeleEnable();

  void SetRobotMode(RobotMode mode);
  void ToggleRobotMode();

  std::string GetRobotModeString();

 private:
  // The driver's controller
  frc::XboxController m_driverController { OIConstants::kDriverControllerPort };
  frc::XboxController m_operatorController { OIConstants::kDriverControllerPort };

  // The robot's subsystems
#ifdef DRIVETRAIN_ENABLED
  DriveSubsystem m_drive;
#endif

#ifdef ARM_ENABLED
  ArmSubsystem m_arm;
#endif

#ifdef ARM_ENABLED
#ifdef INTAKE_ENABLED
  IntakeSubsystem m_intake;
#endif
#endif

#ifdef LED_ENABLED
  LedSubsystem m_leds;
#endif

  // The autonomous routines
  Auto_DoNothing m_auto_doNothing;
  Auto_DriveDistance m_auto_driveForwards { &m_drive };
  Auto2 m_auto2 { &m_drive };
  Auto3 m_auto3 { &m_drive, &m_arm };

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  RobotMode m_mode = RobotMode::CONE;

  void ConfigureButtonBindings();
};
