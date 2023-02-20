// indicates that detailed logging of DS and joystick controls should be enabled
#define LOG_DATA_DRIVER_STATION

// Indicates if the drivetrain commands are bound.
// Currently the drivetrain object is still instantiated.
#define DRIVETRAIN_ENABLED

// indicates that drivetrain odometry should use pose estimation coupled with
// an external vision system that can provide global positioning estimates;
// if not enabled, then basic odometry tracking based on encoders and gyro only
// will be used to estimate robot position
#define DRIVETRAIN_ODOMETRY_VISION

// Indicates if the extending arm is instantiated and it's commands are bound.
#define EXTENDING_ARM_ENABLED

// Inidicates if the intake is instantiated and it's commands are bound.
#define INTAKE_ENABLED