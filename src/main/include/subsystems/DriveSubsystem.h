#pragma once

#include <ctre/Phoenix.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include "Constants.h"
#include "MAXSwerveModule.h"
#include "utils/AngleSlewRateLimiter.h"
#include "utils/SwerveDriveKinematics2.h"

#ifdef SWERVE_SECOND_ORDER_KINEMATICS
template <size_t NumModules>
using SwerveDriveKinematics = SwerveDriveKinematics2<NumModules>;
#else
template <size_t NumModules>
using SwerveDriveKinematics = frc::SwerveDriveKinematics<NumModules>;
#endif

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

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed Input speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Input speed of the robot in the y direction (sideways).
   * @param rotSpeed Angular speed of the robot.
   * @param fieldRelative Indicates that provided x and y speeds are relative to the field.
   */
  void Drive(double xSpeed, double ySpeed, double rotSpeed, bool fieldRelative);

  /**
   * Re-reads and re-assigns various tuning parameters.
   */
  void UpdateTuningParameters();

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Returns the heading of the robot.
   * @return The robot's heading.
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
   * Gets the drive deadband.
   */
  double GetDeadband() const;

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(const frc::Pose2d& pose);

  /**
   * Stops the robot.
   */
  void Stop();

  /**
   * Sets the drive subsystem to the specified chassis speeds.
   *
   * @param speeds Chassis speeds.
   */
  void SetChassisSpeeds(const frc::ChassisSpeeds& speeds);

  /**
   * Sets the drive motor controllers to the specified speeds and orientations.
   *
   * @param desiredStates Drive module states.
   * @param stopped Indicates that robot is stopped.
   */
  void SetModuleStates(
    wpi::array<SwerveModuleState, NUMBER_OF_MODULES> desiredStates, bool stopped = false);

  /**
   * Kinematics describing the drive type.
   */
  SwerveDriveKinematics<NUMBER_OF_MODULES> kDriveKinematics {
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

  /**
   * Gets effective maximum speed.
   */
  units::meters_per_second_t GetMaxSpeed() const;

  frc::SlewRateLimiter<units::scalar> m_rotLimiter;
  frc::SlewRateLimiter<units::scalar> m_magLimiter;
  AngleSlewRateLimiter m_dirLimiter;

  double m_currentRotation = 0.0;
  double m_currentSpeed = 0.0;
  units::radian_t m_currentHeading = 0_rad;

  double m_inputRotation = 0.0;  // [-1, +1]
  double m_inputSpeed = 0.0;     // [-1,+ 1]
  units::radian_t m_inputHeading = 0_rad;

  units::meters_per_second_t m_currentVx;
  units::meters_per_second_t m_currentVy;
  units::radians_per_second_t m_currentOmega;

  double m_deadband = 0.2;
  units::meters_per_second_t m_maxSpeed = DriveConstants::kMaxSpeed;
  units::radians_per_second_t m_maxAngularSpeed = DriveConstants::kMaxAngularSpeed;

  // constrain translational direction heading to a discrete step values (in radians)
  double m_headingStep = 0.0;

  // indicates that robot should maintain X wheel formation when stopped
  bool m_stopX;

  // components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods
  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_rearRight;

  wpi::array<SwerveModuleState, NUMBER_OF_MODULES> m_desiredStates = {
    SwerveModuleState(),
    SwerveModuleState(),
    SwerveModuleState(),
    SwerveModuleState(),
  };

  TalonSRX m_gyroController { IntakeConstants::kIntakeLeaderControllerCanId };
  WPI_PigeonIMU m_gyro { m_gyroController };

  // odometry tracking can be based on either XxxOdometry or XxxPoseEstimator
  // XxxOdometry relies only on encoders and gyro to estimate the pose
  // XxxPoseEstimator extends basic odometry tracking by accepting pose from
  // external vision system to compensate for the drift
  void UpdateOdometry();

#ifdef DRIVETRAIN_ODOMETRY_VISION
  frc::SwerveDrivePoseEstimator<NUMBER_OF_MODULES> m_odometry;
#else
  frc::SwerveDriveOdometry<NUMBER_OF_MODULES> m_odometry;
#endif

  nt::DoubleArraySubscriber m_visionPose;

  // Tracks robot position on the field (for debugging via Glass)
  frc::Field2d m_field;
};
