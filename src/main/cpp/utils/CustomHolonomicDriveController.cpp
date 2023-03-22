#include "utils/CustomHolonomicDriveController.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

#include "Directives.h"

CustomHolonomicDriveController::CustomHolonomicDriveController(
  frc2::PIDController xController,
  frc2::PIDController yController,
  frc::ProfiledPIDController<units::radian> thetaController) :
  m_xController(std::move(xController)),
  m_yController(std::move(yController)),
  m_thetaController(std::move(thetaController)) {
  m_thetaController.EnableContinuousInput(
    units::radian_t { -std::numbers::pi }, units::radian_t { std::numbers::pi });
}

bool CustomHolonomicDriveController::AtReference() const {
  const auto& eTranslate = m_poseError.Translation();
  const auto& eRotate = m_rotationError;
  const auto& tolTranslate = m_poseTolerance.Translation();
  const auto& tolRotate = m_poseTolerance.Rotation();
  return units::math::fabs(eTranslate.X()) < tolTranslate.X() &&
         units::math::fabs(eTranslate.Y()) < tolTranslate.Y() &&
         units::math::fabs(eRotate.Radians()) < tolRotate.Radians();
}

frc::ChassisSpeeds CustomHolonomicDriveController::Calculate(
  const frc::Pose2d& currentPose,
  const frc::Pose2d& trajectoryPose,
  const units::meters_per_second_t desiredLinearVelocity,
  const frc::Rotation2d& desiredRotation,
  const units::radians_per_second_t desiredAngularVelocity) {
  // reset theta controller to current pose on the first run
  if (m_firstRun) {
    m_thetaController.Reset(currentPose.Rotation().Radians());
    m_firstRun = false;
  }

  // calculate feedforward velocities (field-relative)
  const auto xFF = desiredLinearVelocity * trajectoryPose.Rotation().Cos();
  const auto yFF = desiredLinearVelocity * trajectoryPose.Rotation().Sin();
  const auto thetaFF = desiredAngularVelocity;

  m_poseError = trajectoryPose.RelativeTo(currentPose);
  m_rotationError = desiredRotation - currentPose.Rotation();

  if (!m_enabled) {
    return frc::ChassisSpeeds::FromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.Rotation());
  }

  // calculate feedback velocities (based on position error)
  const auto xFeedback = units::meters_per_second_t {
    m_xController.Calculate(currentPose.X().value(), trajectoryPose.X().value()),
  };

  const auto yFeedback = units::meters_per_second_t {
    m_yController.Calculate(currentPose.Y().value(), trajectoryPose.Y().value()),
  };

  const auto thetaFeedback = units::radians_per_second_t {
    m_thetaController.Calculate(currentPose.Rotation().Radians(), desiredRotation.Radians()),
  };

#ifdef AUTO_COMMAND_LOG
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/ΔΘ ", m_rotationError.Radians().convert<units::degree>().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/ΔX", m_poseError.Translation().X().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/ΔY", m_poseError.Translation().Y().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/Θ FF", thetaFF.convert<units::degrees_per_second>().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/Θ FB", thetaFeedback.convert<units::degrees_per_second>().value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/X FF", xFF.value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/X FB", xFeedback.value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/Y FF", yFF.value());
  frc::SmartDashboard::PutNumber("Telemetry/Holonomic/Y FB", yFeedback.value());
  // clang-format on
#endif

  return frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.Rotation());
}

frc::ChassisSpeeds CustomHolonomicDriveController::Calculate(
  const frc::Pose2d& currentPose,
  const frc::Trajectory::State& desiredState,
  const RotationProfile::State& desiredRotation) {
  return Calculate(
    currentPose,
    desiredState.pose,
    desiredState.velocity,
    desiredRotation.rotation,
    desiredRotation.velocity);
}
