#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "commands/ArmMoveCommand.h"
#include "commands/ArmScoreConeCommand.h"
#include "commands/ArmScoreCubeCommand.h"
#include "commands/SetLedCommand.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // initialize all of your commands and subsystems here

#ifdef LED_ENABLED
  m_leds.SetRGB(255, 0, 0);
  m_leds.SetPattern(LEDPattern::BREATH, 1_s);
#endif

// register subsystems on the shuffleboard
#ifdef DRIVETRAIN_ENABLED
  frc::Shuffleboard::GetTab("Drivetrain").Add(m_drive);
#endif
#ifdef ARM_ENABLED
  frc::Shuffleboard::GetTab("Arm").Add(m_arm);
#endif

#ifdef TRACE_LOG
  // Log Shuffleboard events for command initialize, execute, finish, interrupt
  frc2::CommandScheduler::GetInstance().OnCommandInitialize([](const frc2::Command& command) {
    frc::Shuffleboard::AddEventMarker(
      "Command initialized", command.GetName(), frc::ShuffleboardEventImportance::kNormal);
    frc::DataLogManager::Log(fmt::format("Scheduler|{}: Initialize", command.GetName()));
  });
  frc2::CommandScheduler::GetInstance().OnCommandExecute([](const frc2::Command& command) {
    frc::Shuffleboard::AddEventMarker(
      "Command executed", command.GetName(), frc::ShuffleboardEventImportance::kNormal);
    // frc::DataLogManager::Log(fmt::format("Scheduler|{}: Execute", command.GetName()));
  });
  frc2::CommandScheduler::GetInstance().OnCommandFinish([](const frc2::Command& command) {
    frc::Shuffleboard::AddEventMarker(
      "Command finished", command.GetName(), frc::ShuffleboardEventImportance::kNormal);
    frc::DataLogManager::Log(fmt::format("Scheduler|{}: Finish", command.GetName()));
  });
  frc2::CommandScheduler::GetInstance().OnCommandInterrupt([](const frc2::Command& command) {
    frc::Shuffleboard::AddEventMarker(
      "Command interrupted", command.GetName(), frc::ShuffleboardEventImportance::kNormal);
    frc::DataLogManager::Log(fmt::format("Scheduler|{}: Interupt", command.GetName()));
  });
#endif

  // configure the button bindings
  ConfigureButtonBindings();

  // set up default drive command:
  // - the left stick controls translation of the robot
  // - turning is controlled by the X axis of the right stick
#ifdef DRIVETRAIN_ENABLED
  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_drive.Drive(
        -units::meters_per_second_t {
          frc::ApplyDeadband(m_driverController.GetLeftY(), m_drive.GetDeadband()) },
        -units::meters_per_second_t {
          frc::ApplyDeadband(m_driverController.GetLeftX(), m_drive.GetDeadband()) },
        -units::radians_per_second_t {
          frc::ApplyDeadband(m_driverController.GetRightX(), m_drive.GetDeadband()) },
        true,
        true);
    },
    { &m_drive }));
#endif

  // autonomous command chooser
#ifdef AUTO_CHOOSER
  frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("Auto");
  m_chooser.SetDefaultOption("DoNothing", &m_auto_doNothing);
  m_chooser.AddOption("DriveForwards", &m_auto_driveForwards);
  m_chooser.AddOption("Auto2", &m_auto2);
  m_chooser.AddOption("Auto3", &m_auto3);
  tab.Add(m_chooser);
  tab.AddPersistent("DriveAutoDistance", 1.0);
#endif
}

void RobotContainer::OnAutoEnable() {
  m_drive.ResetOdometry(frc::Pose2d {});
}

void RobotContainer::OnTeleEnable() {
  SetRobotMode(RobotMode::CONE);
}

void RobotContainer::OnDisable() {
#ifdef ARM_ENABLED
  m_arm.OnDisable();
#endif

#ifdef LED_ENABLED
  m_leds.SetRGB(255, 0, 0);
  m_leds.SetPattern(LEDPattern::BREATH, 3_s);
#endif
}

void RobotContainer::SetRobotMode(RobotMode mode) {
  m_mode = mode;
  if (m_mode == RobotMode::CUBE) {
#ifdef LED_ENABLED
    m_leds.SetRGB(255, 0, 255);
    m_leds.SetPattern(LEDPattern::SOLID, 0_s);
#endif
  } else if (m_mode == RobotMode::CONE) {
#ifdef LED_ENABLED
    m_leds.SetRGB(255, 255, 0);
    m_leds.SetPattern(LEDPattern::SOLID, 0_s);
#endif
  }
}

void RobotContainer::ToggleRobotMode() {
  if (m_mode == RobotMode::CONE) {
    SetRobotMode(RobotMode::CUBE);
  } else {
    SetRobotMode(RobotMode::CONE);
  }
}

std::string RobotContainer::GetRobotModeString() {
  if (m_mode == RobotMode::CONE) {
    return "CONE";
  } else if (m_mode == RobotMode::CUBE) {
    return "CUBE";
  }
  return "Undefined";
}

void RobotContainer::ConfigureButtonBindings() {
#ifdef ARM_ENABLED
#ifdef INTAKE_ENABLED
  // Reload parameters.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
    .OnTrue(new frc2::InstantCommand(
      [this] {
        m_arm.ConfigureTuningParams();
        m_intake.ConfigureTuningParams();
      },
      {}));
#endif
#endif

  // Toggle robot mode.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
    .OnTrue(frc2::InstantCommand([this] { ToggleRobotMode(); }, {}).ToPtr());

  // Testing LEDs
  frc2::Trigger(
    m_driverController.POVLeft(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(new SetLedCommand(&m_leds, 255, 255, 255, LEDPattern::FLASH, 200_ms, 3_s));

  // Testing LEDs
  frc2::Trigger(
    m_driverController.POVRight(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(new SetLedCommand(&m_leds, 0, 255, 0, LEDPattern::BREATH, 3_s, 0_s));

#ifdef DRIVETRAIN_ENABLED
  // Reset swerve heading by zeroing the gyro.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightStick)
    .OnTrue(frc2::InstantCommand([this] { m_drive.ZeroHeading(); }, { &m_drive }).ToPtr());

  // Lock the wheels.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftStick)
    .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, { &m_drive }));

  // frc2::Trigger(
  //   m_driverController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(frc2::InstantCommand([this] { m_drive.ResetOdometry(frc::Pose2d {}); }, { &m_drive })
  //             .ToPtr());
#endif

#ifdef ARM_ENABLED
  // Zero's the arm axis. TODO: Change this to run the homing command.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
    .OnTrue(new frc2::InstantCommand(
      [this] {
        m_arm.ZeroAxis1();
        m_arm.ZeroAxis2();
      },
      { &m_arm }));

  frc2::Trigger(
    m_driverController.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(m_arm.JogUp());

  frc2::Trigger(
    m_driverController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(m_arm.JogDown());

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
    .OnTrue(ArmMoveCommand(&m_arm, ArmPosition::STOWED).ToPtr());

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
    .OnTrue(frc2::cmd::Either(
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_MID_CONE).ToPtr(),
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_MID_CUBE).ToPtr(),
      [this] { return m_mode == RobotMode::CONE; }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
    .OnTrue(frc2::cmd::Either(
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_HIGH_CONE).ToPtr(),
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_HIGH_CUBE).ToPtr(),
      [this] { return m_mode == RobotMode::CONE; }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
    .OnTrue(frc2::cmd::Sequence(
      ArmMoveCommand(&m_arm, ArmPosition::SCORE_LOW_CUBE),
      frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(-0.5); }, { &m_intake }).ToPtr()))
    .OnFalse(frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(0.0); }, { &m_intake }).ToPtr());

#ifdef INTAKE_ENABLED
  // Intake automatically.
  frc2::Trigger(m_driverController.LeftTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::cmd::Either(
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_STATION_CONE),
        frc2::InstantCommand([this] { m_intake.IntakeCone(); }, { &m_intake })),
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_STATION_CUBE),
        frc2::InstantCommand([this] { m_intake.IntakeCube(); }, { &m_intake })),
      [this] { return m_mode == RobotMode::CONE; }))
    .OnFalse(frc2::cmd::Sequence(
      frc2::InstantCommand([this] { m_intake.Stop(); }, { &m_intake }),
      ArmMoveCommand(&m_arm, ArmPosition::STOWED)));

  // Score automatically.
  // frc2::Trigger(m_driverController.LeftTrigger(
  //                 0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
  //   .OnTrue(frc2::cmd::Sequence(
  //     m_exArm.MoveArm(ExtendingArmPositions::INTAKE_FLOOR_CONE),
  //     frc2::InstantCommand(
  //       [this] { m_intake.SetIntakeSpeed(IntakeConstants::kIntakingSpeed); }, { &m_intake })))
  //   .OnFalse(frc2::cmd::Sequence(
  //     frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(0.0); }, { &m_intake }),
  //     m_exArm.MoveArm(ExtendingArmPositions::STOWED)));

  // Intake manually.
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper)
    .OnTrue(frc2::cmd::Either(
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_FLOOR_CONE),
        frc2::InstantCommand([this] { m_intake.IntakeCone(); }, { &m_intake })),
      frc2::cmd::Sequence(
        ArmMoveCommand(&m_arm, ArmPosition::INTAKE_FLOOR_CUBE),
        frc2::InstantCommand([this] { m_intake.IntakeCube(); }, { &m_intake })),
      [this] { return m_mode == RobotMode::CONE; }))
    .OnFalse(frc2::cmd::Sequence(
      frc2::InstantCommand([this] { m_intake.Stop(); }, { &m_intake }),
      ArmMoveCommand(&m_arm, ArmPosition::STOWED)));

  // Score Manually.
  frc2::Trigger(m_driverController.RightTrigger(
                  0.5, frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()))
    .OnTrue(frc2::cmd::Either(
      ArmScoreConeCommand(&m_arm, &m_intake).ToPtr(),
      ArmScoreCubeCommand(&m_arm, &m_intake).ToPtr(),
      [this] { return m_mode == RobotMode::CONE; }))
    .OnFalse(ArmMoveCommand(&m_arm, ArmPosition::STOWED).ToPtr());

  // Score manually.
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
  //   .OnTrue(frc2::cmd::Either(
  //     m_exArm.ScoreCone(), m_exArm.ScoreCube(), [this] { return m_mode == RobotMode::CONE; }));
#endif
#endif
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
#ifdef AUTO_CHOOSER
  return m_chooser.GetSelected();
#else
#ifdef DRIVETRAIN_ENABLED
  // set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

  // add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // an example trajectory to follow (all units in meters)
  const auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    {
      frc::Pose2d { 0_m, 0_m, 0_deg },
      frc::Pose2d { 1_m, 0_m, 0_deg },
      frc::Pose2d { 2_m, 0_m, 0_deg },
      frc::Pose2d { 3_m, 0_m, 0_deg },
      frc::Pose2d { 4_m, 0_m, 0_deg },
    },
    // pass the config
    config);

  frc::ProfiledPIDController<units::radians> thetaController {
    AutoConstants::kPThetaController,
    AutoConstants::kIThetaController,
    AutoConstants::kDThetaController,
    AutoConstants::kThetaControllerConstraints
  };

  thetaController.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });

  const auto rotation = frc::Rotation2d { 180_deg };
  frc2::SwerveControllerCommand<NUMBER_OF_MODULES> swerveControllerCommand(
    exampleTrajectory,
    [this]() { return m_drive.GetPose(); },
    m_drive.kDriveKinematics,
    frc2::PIDController {
      AutoConstants::kPXController, AutoConstants::kIXController, AutoConstants::kDXController },
    frc2::PIDController {
      AutoConstants::kPYController, AutoConstants::kIYController, AutoConstants::kDYController },
    thetaController,
    [rotation]() { return rotation; },
    [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
    { &m_drive });

  // reset odometry to the starting pose of the trajectory
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
    std::move(swerveControllerCommand),
    frc2::InstantCommand([this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, true, true); }, {}));
#else
  return new frc2::InstantCommand(
    [this] { frc::DataLogManager::Log("Drivetrain disabled. Auto not run."); });
#endif
#endif
}
