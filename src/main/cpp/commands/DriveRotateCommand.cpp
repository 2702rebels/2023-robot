#include "commands/DriveRotateCommand.h"

#include <frc/Timer.h>
#include <units/math.h>

DriveRotateCommand::DriveRotateCommand(
  const frc::Rotation2d& desiredRotation, DriveSubsystem* drive) :
  m_drive(drive),
  m_desiredRotation(std::move(desiredRotation)),
  m_controller {
    AutoConstants::kPThetaController,
    AutoConstants::kIThetaController,
    AutoConstants::kDThetaController,
    AutoConstants::kThetaControllerConstraints,
  } {
  m_controller.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });

  AddRequirements(drive);
}

void DriveRotateCommand::Initialize() {
  m_timer.Restart();
  m_controller.Reset(m_drive->GetPose().Rotation().Radians());
}

void DriveRotateCommand::Execute() {
  const auto rotation = m_drive->GetPose().Rotation().Radians();
  const auto thetaFF =
    units::radians_per_second_t { m_controller.Calculate(rotation, m_desiredRotation.Radians()) };

  const auto targetModuleStates = m_drive->kDriveKinematics.ToSwerveModuleStates(
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(0_mps, 0_mps, thetaFF, rotation));

  m_drive->SetModuleStates(targetModuleStates);
}

void DriveRotateCommand::End(bool interrupted) {
  m_timer.Stop();
  m_drive->Stop();
}

bool DriveRotateCommand::IsFinished() {
  const auto rotation = m_drive->GetPose().Rotation().Degrees();

  // keep track of the timestamp when the rotation has stabilized
  if (units::math::abs(rotation - m_lastRotation) > 1_deg) {
    m_lastRotationAt = m_timer.Get();
  }

  m_lastRotation = rotation;

  // we are finished if we have stopped rotating for at least 1 second
  // or we are at the objective within 2 degrees
  return (m_timer.Get() - m_lastRotationAt > 1_s) ||
         units::math::abs(rotation - m_desiredRotation.Degrees()) < 2_deg;
}
