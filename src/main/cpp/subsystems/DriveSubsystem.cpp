#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem() :
  m_frontLeft {
    kFrontLeftDrivingCanId,
    kFrontLeftTurningCanId,
    kFrontLeftChassisAngularOffset,
  },
  m_rearLeft {
    kRearLeftDrivingCanId,
    kRearLeftTurningCanId,
    kRearLeftChassisAngularOffset,
  },
  m_frontRight {
    kFrontRightDrivingCanId,
    kFrontRightTurningCanId,
    kFrontRightChassisAngularOffset,
  },
  m_rearRight {
    kRearRightDrivingCanId,
    kRearRightTurningCanId,
    kRearRightChassisAngularOffset,
  },
  m_odometry {
    kDriveKinematics,
    frc::Rotation2d(GetGyroAngle()),
    { m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearLeft.GetPosition(),
      m_rearRight.GetPosition() },
    frc::Pose2d {},
  } {
  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutData("Gyro", &m_gyro);
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(
    frc::Rotation2d(GetGyroAngle()),
    { m_frontLeft.GetPosition(),
      m_rearLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearRight.GetPosition() });
  m_field.SetRobotPose(m_odometry.GetPose());
}

void DriveSubsystem::Drive(
  units::meters_per_second_t xSpeed,
  units::meters_per_second_t ySpeed,
  units::radians_per_second_t rot,
  bool fieldRelative,
  bool rateLimit) {
  double xSpeedCommanded;
  double ySpeedCommanded;

  if (rateLimit) {
    // convert XY to polar for rate limiting
    const auto inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    const auto inputTranslationMag = sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // calculate the direction slew rate based on an estimate of the lateral acceleration
    const auto directionSlewRate =
      m_currentTranslationMag != 0.0
        ? abs(DriveConstants::kDirectionSlewRate / m_currentTranslationMag)
        : 500.0;  // some high number that means the slew rate is effectively instantaneous

    const auto currentTime = wpi::Now() * 1e-6;
    const auto elapsedTime = currentTime - m_prevTime;
    const auto angleDiff =
      SwerveUtils::AngleDifference(inputTranslationDir, m_currentTranslationDir);

    if (angleDiff < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
        m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
    } else if (angleDiff > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag > 1e-4) {
        // some small number to avoid floating-point errors with
        // equality checking keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      } else {
        m_currentTranslationDir =
          SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
    } else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
        m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(0.0);
    }

    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.Calculate(rot.value());
  } else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = rot.value();
  }

  // convert the commanded speeds into the correct units for the drivetrain
  const auto xSpeedDelivered = xSpeedCommanded * DriveConstants::kMaxSpeed;
  const auto ySpeedDelivered = ySpeedCommanded * DriveConstants::kMaxSpeed;
  const auto rotDelivered = m_currentRotation * DriveConstants::kMaxAngularSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
    fieldRelative
      ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          xSpeedDelivered, ySpeedDelivered, rotDelivered, frc::Rotation2d(GetGyroAngle()))
      : frc::ChassisSpeeds { xSpeedDelivered, ySpeedDelivered, rotDelivered });

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(frc::SwerveModuleState { 0_mps, frc::Rotation2d { 45_deg } });
  m_frontRight.SetDesiredState(frc::SwerveModuleState { 0_mps, frc::Rotation2d { -45_deg } });
  m_rearLeft.SetDesiredState(frc::SwerveModuleState { 0_mps, frc::Rotation2d { -45_deg } });
  m_rearRight.SetDesiredState(frc::SwerveModuleState { 0_mps, frc::Rotation2d { 45_deg } });
}

void DriveSubsystem::SetModuleStates(
  wpi::array<frc::SwerveModuleState, NUMBER_OF_MODULES> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return frc::Rotation2d(GetGyroAngle()).Degrees();
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate() const {
  return -m_gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() const {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
    GetHeading(),
    {
      m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearLeft.GetPosition(),
      m_rearRight.GetPosition(),
    },
    pose);
}

units::degree_t DriveSubsystem::GetGyroAngle() const {
  return units::degree_t { m_gyro.GetAngle() };
}
