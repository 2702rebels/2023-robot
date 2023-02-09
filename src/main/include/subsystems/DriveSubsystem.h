#pragma once

#include <ctre/Phoenix.h>
#include <frc/ADIS16470_IMU.h>
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
  units::degree_t GetHeading() const;

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
  void ResetOdometry(frc::Pose2d pose);

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

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearRight;

  TalonSRX m_gyroController { IntakeConstants::kIntakeLeaderControllerCanId };
  WPI_PigeonIMU m_gyro { m_gyroController };

  // Slew rate filter variables for controlling lateral acceleration
  double m_currentRotation = 0.0;
  double m_currentTranslationDir = 0.0;
  double m_currentTranslationMag = 0.0;

  frc::SlewRateLimiter<units::scalar> m_magLimiter { DriveConstants::kMagnitudeSlewRate / 1_s };
  frc::SlewRateLimiter<units::scalar> m_rotLimiter { DriveConstants::kRotationalSlewRate / 1_s };
  double m_prevTime = wpi::Now() * 1e-6;

  // Odometry class for tracking robot pose
  frc::SwerveDriveOdometry<NUMBER_OF_MODULES> m_odometry;

  // Tracks robot position on the field (for debugging via Glass)
  frc::Field2d m_field;
};
