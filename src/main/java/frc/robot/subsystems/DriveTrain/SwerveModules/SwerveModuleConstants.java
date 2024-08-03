package frc.robot.subsystems.DriveTrain.SwerveModules;

import frc.util.PIDFGains;

public enum SwerveModuleConstants {
    DEVBOT(6.75, 12.5, 0.0508, 2, false,
            true, true, 0.302, -0.44,
            -0.164, 0.303,
            new PIDFGains(10, 0, 0, 1.2, 0.1, 0, 1 / SwerveModule.moduleThreadHz,
                    3, 100),
            new PIDFGains(10, 0, 0, 0.14, 0.1, 0, 1 / SwerveModule.moduleThreadHz)),
    COMPBOT(6.75, 12.5 * 3, 0.0508, 2, false,
            false, false, 0.59625, 0.9368,
            0.226, 0.750,
            new PIDFGains(0, 0.00, 0, 0, 0.1, 0, 1 / SwerveModule.moduleThreadHz,
                    3, 100),
            new PIDFGains(0, 0, 0, 0, 0.1, 0, 1 / SwerveModule.moduleThreadHz));

    public final double driveGearRatio;
    public final double steerGearRatio;

    public final double wheelRadiusMeters;
    public final double wheelCircumferenceMeters;

    public final double maxDriveVelocityMetersPerSecond;

    public final boolean isDriveInverted;
    public final boolean isSteerInverted;
    public final boolean isAbsEncoderInverted;

    public final double front_left_zeroOffset; // the offset between the absolute encoder reading on the front left module, in degrees
    public final double front_right_zeroOffset; // the offset between the absolute encoder on the front left module, in degrees
    public final double back_left_zeroOffset; // the offset between the absolute encoder on the back left module, in degrees
    public final double back_right_zeroOffset; // the offset between the absolute encoder on the back right module, in degrees

    public final PIDFGains driveMotorPID;
    public final PIDFGains steerMotorPID;

    SwerveModuleConstants(double driveGearRatio, double steerGearRatio, double wheelRadiusMeters,
                           double maxDriveVelocityMetersPerSecond, boolean isDriveInverted, boolean isSteerInverted,
                           boolean isAbsEncoderInverted, double front_left_zeroOffset, double front_right_zeroOffset,
                           double back_left_zeroOffset, double back_right_zeroOffset, PIDFGains driveMotorPID,
                           PIDFGains steerMotorPID) {

        this.driveGearRatio = driveGearRatio;
        this.steerGearRatio = steerGearRatio;
        this.wheelRadiusMeters = wheelRadiusMeters;
        wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters;
        this.maxDriveVelocityMetersPerSecond = maxDriveVelocityMetersPerSecond;
        this.isDriveInverted = isDriveInverted;
        this.isSteerInverted = isSteerInverted;
        this.isAbsEncoderInverted = isAbsEncoderInverted;
        this.front_left_zeroOffset = front_left_zeroOffset;
        this.front_right_zeroOffset = front_right_zeroOffset;
        this.back_left_zeroOffset = back_left_zeroOffset;
        this.back_right_zeroOffset = back_right_zeroOffset;
        this.driveMotorPID = driveMotorPID;
        this.steerMotorPID = steerMotorPID;
    }
}
