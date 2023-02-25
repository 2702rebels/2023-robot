#include "subsystems/DriveSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/StateSpaceUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <wpi/DataLog.h>
#include <wpi/sendable/SendableBuilder.h>

#include "Constants.h"
#include "Settings.h"
#include "utils/MathUtils.h"
#include "utils/SwerveUtils.h"

using namespace DriveConstants;

REGISTER_DOUBLE(DrivePoseStdDevStateX, "Drive/Pose/StdDev State/X", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevStateY, "Drive/Pose/StdDev State/Y", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevStateH, "Drive/Pose/StdDev State/θ", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevVisionX, "Drive/Pose/StdDev Vision/X", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevVisionY, "Drive/Pose/StdDev Vision/Y", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevVisionH, "Drive/Pose/StdDev Vision/θ", 0.1)
REGISTER_DOUBLE(DriveSlewRateDirection, "Drive/Slew/Direction", DriveConstants::kDirectionSlewRate)
REGISTER_DOUBLE(DriveSlewRateMagnitude, "Drive/Slew/Magnitude", DriveConstants::kMagnitudeSlewRate)
REGISTER_DOUBLE(DriveSlewRateRotation, "Drive/Slew/Rotation", DriveConstants::kRotationalSlewRate)
REGISTER_DOUBLE(DriveTuneTranslationStep, "Drive/Tune/Translation Step", 5)
REGISTER_DOUBLE(DriveTuneMaxSpeed, "Drive/Tune/Max Speed (mps)", 3)
REGISTER_DOUBLE(DriveTuneMaxAngularSpeed, "Drive/Tune/Max Angular Speed (dps)", 360)

DriveSubsystem::DriveSubsystem() :
  m_magLimiter { GetDriveSlewRateMagnitude() / 1_s },
  m_rotLimiter { GetDriveSlewRateRotation() / 1_s },
  m_slewRateDirection { GetDriveSlewRateDirection() },
  m_maxSpeed { units::meters_per_second_t { GetDriveTuneMaxSpeed() } },
  m_maxAngularSpeed { units::convert<units::degrees_per_second, units::radians_per_second>(
    GetDriveTuneMaxAngularSpeed()) },
  m_translationStep { units::convert<units::degree, units::radian>(GetDriveTuneTranslationStep()) },
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
    GetHeading(),
    {
      m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearLeft.GetPosition(),
      m_rearRight.GetPosition(),
    },
    frc::Pose2d {},
#ifdef DRIVETRAIN_ODOMETRY_VISION
    { GetDrivePoseStdDevStateX(), GetDrivePoseStdDevStateY(), GetDrivePoseStdDevStateH() },
    { GetDrivePoseStdDevVisionX(), GetDrivePoseStdDevVisionY(), GetDrivePoseStdDevVisionH() },
#endif
  } {
  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutData("Gyro", &m_gyro);
}

void DriveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  SubsystemBase::InitSendable(builder);

  builder.SetSmartDashboardType("Drive");

  builder.AddDoubleProperty(
    "Telemetry/Translation Heading", [this] { return m_currentTranslationDir; }, nullptr);
  builder.AddDoubleProperty(
    "Telemetry/Translation Magnitude", [this] { return m_currentTranslationMag; }, nullptr);
}

void DriveSubsystem::Periodic() {
  UpdateOdometry();
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
    auto inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    const auto inputTranslationMag = sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    if (m_translationStep > 0) {
      inputTranslationDir = MathUtils::RoundToStep(inputTranslationDir, m_translationStep);
    }

    // calculate the direction slew rate based on an estimate of the lateral acceleration
    const auto directionSlewRate =
      m_currentTranslationMag != 0.0
        ? abs(m_slewRateDirection / m_currentTranslationMag)
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
      // some small number to avoid floating-point errors with equality checking
      if (m_currentTranslationMag > 1e-4) {
        // keep currentTranslationDir unchanged
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
  const auto xSpeedDelivered = xSpeedCommanded * m_maxSpeed;
  const auto ySpeedDelivered = ySpeedCommanded * m_maxSpeed;
  const auto rotDelivered = m_currentRotation * m_maxAngularSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                      xSpeedDelivered, ySpeedDelivered, rotDelivered, GetHeading())
                  : frc::ChassisSpeeds { xSpeedDelivered, ySpeedDelivered, rotDelivered });

  kDriveKinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);

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
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, m_maxSpeed);
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

units::degree_t DriveSubsystem::GetGyroAngle() const {
  return units::degree_t { -m_gyro.GetAngle() };  // flip the sign if changing gyro orientation
}

frc::Rotation2d DriveSubsystem::GetHeading() const {
  return frc::Rotation2d(GetGyroAngle());
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate() const {
  return m_gyro.GetRate();  // flip the sign if changing gyro orientation
}

void DriveSubsystem::UpdateOdometry() {
  m_odometry.Update(
    GetHeading(),
    {
      m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearLeft.GetPosition(),
      m_rearRight.GetPosition(),
    });

  // feed global position from camera
  // m_odometry.AddVisionMeasurement(pose, timestamp);

  // update field position for debugging purposes
  m_field.SetRobotPose(GetPose());
}

frc::Pose2d DriveSubsystem::GetPose() const {
#ifdef DRIVETRAIN_ODOMETRY_VISION
  return m_odometry.GetEstimatedPosition();
#else
  return m_odometry.GetPose();
#endif
}

void DriveSubsystem::ResetOdometry(const frc::Pose2d& pose) {
  m_gyro.Reset();
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
