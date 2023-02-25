#pragma once

#include <ctre/Phoenix.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "MAXSwerveModule.h"

// number of MAXSwerveModule managed by drive subsystem
#define NUMBER_OF_MODULES 4

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Custom subsystem telemetry.
   */
  void InitSendable(wpi::SendableBuilder& builder) override;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  void Drive(
    units::meters_per_second_t xSpeed,
    units::meters_per_second_t ySpeed,
    units::radians_per_second_t rot,
    bool fieldRelative,
    bool rateLimit);

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, NUMBER_OF_MODULES> desiredStates);

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from 180 to 180
   */
  frc::Rotation2d GetHeading() const;

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second.
   */
  double GetTurnRate() const;

  /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
  frc::Pose2d GetPose() const;

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(const frc::Pose2d& pose);

  frc::SwerveDriveKinematics<NUMBER_OF_MODULES> kDriveKinematics {
    frc::Translation2d { DriveConstants::kWheelBase / 2, DriveConstants::kTrackWidth / 2 },
    frc::Translation2d { DriveConstants::kWheelBase / 2, -DriveConstants::kTrackWidth / 2 },
    frc::Translation2d { -DriveConstants::kWheelBase / 2, DriveConstants::kTrackWidth / 2 },
    frc::Translation2d { -DriveConstants::kWheelBase / 2, -DriveConstants::kTrackWidth / 2 }
  };

 private:
  /**
   * Gets the angle reading of the gyroscope.
   * @returns The angle in degrees.
   */
  units::degree_t GetGyroAngle() const;

  frc::SlewRateLimiter<units::scalar> m_magLimiter;
  frc::SlewRateLimiter<units::scalar> m_rotLimiter;

  // Slew rate filter variables for controlling lateral acceleration
  double m_currentRotation = 0.0;
  double m_currentTranslationDir = 0.0;
  double m_currentTranslationMag = 0.0;
  double m_slewRateDirection = DriveConstants::kDirectionSlewRate;

  units::meters_per_second_t m_maxSpeed = DriveConstants::kMaxSpeed;
  units::radians_per_second_t m_maxAngularSpeed = DriveConstants::kMaxAngularSpeed;

  // Constrain translational direction heading to a discrete step values (in radians)
  double m_translationStep = 0.0;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearRight;

  TalonSRX m_gyroController { IntakeConstants::kIntakeLeaderControllerCanId };
  WPI_PigeonIMU m_gyro { m_gyroController };

  double m_prevTime = wpi::Now() * 1e-6;

  // Odometry tracking can be based on either XxxOdometry or XxxPoseEstimator
  // XxxOdometry relies only on encoders and gyro to estimate the pose
  // XxxPoseEstimator extends basic odometry tracking by accepting pose from
  // external vision system to compensate for the drift
  void UpdateOdometry();

#ifdef DRIVETRAIN_ODOMETRY_VISION
  frc::SwerveDrivePoseEstimator<NUMBER_OF_MODULES> m_odometry;
#else
  frc::SwerveDriveOdometry<NUMBER_OF_MODULES> m_odometry;
#endif

  // Tracks robot position on the field (for debugging via Glass)
  frc::Field2d m_field;
};
