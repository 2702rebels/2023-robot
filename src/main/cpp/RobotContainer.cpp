#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "commands/HomeAxis1Command.h"
#include "commands/HomeAxis2Command.h"
#include "commands/MoveAxis1Command.h"
#include "commands/MoveAxis2Command.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // initialize all of your commands and subsystems here

// register subsystems on the shuffleboard
#ifdef DRIVETRAIN_ENABLED
  frc::Shuffleboard::GetTab("Drivetrain").Add(m_drive);
#endif
#ifdef EXTENDING_ARM_ENABLED
  frc::Shuffleboard::GetTab("Arm").Add(m_exArm);
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
  m_chooser.SetDefaultOption("Auto1", &m_auto1);
  m_chooser.AddOption("Auto2", &m_auto2);
  frc::Shuffleboard::GetTab("Auto").Add(m_chooser);
#endif
}

void RobotContainer::OnDisable() {
#ifdef EXTENDING_ARM_ENABLED
  m_exArm.OnDisable();
#endif
}

void RobotContainer::ConfigureButtonBindings() {
#ifdef DRIVETRAIN_ENABLED
  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper)
  //   .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, { &m_drive }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightStick)
    .OnTrue(new frc2::InstantCommand([this] { m_drive.ZeroHeading(); }, { &m_drive }));

  // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
  //   .OnTrue(
  //     new frc2::InstantCommand([this] { m_drive.ResetOdometry(frc::Pose2d {}); }, { &m_drive }));
#endif

#ifdef EXTENDING_ARM_ENABLED
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
    .OnTrue(new frc2::InstantCommand([this] { m_exArm.ConfigureTuningParams(); }, { &m_exArm }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
    .OnTrue(new frc2::InstantCommand(
      [this] {
        m_exArm.ZeroAxis1();
        m_exArm.ZeroAxis2();
      },
      { &m_exArm }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
    .OnTrue(HomeAxis2Command(&m_exArm).ToPtr());

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
    .OnTrue(
      new frc2::InstantCommand([this] { m_exArm.MoveArmSimple(0.0_deg, 0.0_in); }, { &m_exArm }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
    .OnTrue(
      new frc2::InstantCommand([this] { m_exArm.MoveArmSimple(70.0_deg, 7.0_in); }, { &m_exArm }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
    .OnTrue(
      new frc2::InstantCommand([this] { m_exArm.MoveArmSimple(80.0_deg, 20.0_in); }, { &m_exArm }));
#endif

#ifdef INTAKE_ENABLED
  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper)
    .OnTrue(new frc2::InstantCommand(
      [this] { m_intake.SetIntakeSpeed(IntakeConstants::kIntakingSpeed); }, { &m_intake }))
    .OnFalse(new frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(0.0); }, { &m_intake }));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
    .OnTrue(new frc2::InstantCommand(
      [this] { m_intake.SetIntakeSpeed(IntakeConstants::kOuttakingSpeed); }, { &m_intake }))
    .OnFalse(new frc2::InstantCommand([this] { m_intake.SetIntakeSpeed(0.0); }, { &m_intake }));
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
    // start at the origin facing the +X direction
    frc::Pose2d { 0_m, 0_m, 0_deg },
    // pass through these two interior waypoints, making an 's' curve path
    { frc::Translation2d { 1_m, 1_m }, frc::Translation2d { 2_m, -1_m } },
    // end 3 meters straight ahead of where we started, facing forward
    frc::Pose2d { 3_m, 0_m, 0_deg },
    // pass the config
    config);

  frc::ProfiledPIDController<units::radians> thetaController {
    AutoConstants::kPThetaController, 0, 0, AutoConstants::kThetaControllerConstraints
  };

  thetaController.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });

  frc2::SwerveControllerCommand<NUMBER_OF_MODULES> swerveControllerCommand(
    exampleTrajectory,
    [this]() { return m_drive.GetPose(); },
    m_drive.kDriveKinematics,
    frc2::PIDController { AutoConstants::kPXController, 0, 0 },
    frc2::PIDController { AutoConstants::kPYController, 0, 0 },
    thetaController,
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
