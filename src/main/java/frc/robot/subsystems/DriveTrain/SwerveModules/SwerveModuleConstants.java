package frc.robot.subsystems.DriveTrain.SwerveModules;

import frc.util.PIDFGains;

public enum SwerveModuleConstants {
    /**
     * usually the chassis
     */
    DEVBOT(5.14, 12.8, 0.0508, 4.75,
            false, true, true,

            0.298, -0.41, -0.152, 0.322,

            new PIDFGains[]{
                    new PIDFGains(0.3, 0, 0), //FL
                    new PIDFGains(0.3, 0, 0), //FR
                    new PIDFGains(0.3, 0, 0), //BL
                    new PIDFGains(0.3, 0, 0) //BR
            },
            new PIDFGains[]{
                    new PIDFGains(0.01, 0, 0),
                    new PIDFGains(0.01, 0, 0),
                    new PIDFGains(0.01, 0, 0),
                    new PIDFGains(0.01, 0, 0)
            }),

    /**
     * usually the final robot
     */
    COMPBOT(6.75, 12.5, 0.0508, 4.5,
            false, false, false,

            0.59625, 0.9368, 0.226, 0.750,

            new PIDFGains[]{
                new PIDFGains(0.525, 0, 0.0025, 0, 0.2, 0), //FL
                new PIDFGains(0.666, 0, 0.0058, 0, 0.2, 0), //FR
                new PIDFGains(0.625, 0, 0.0083, 0, 0.2, 0), //BL
                new PIDFGains(0.541, 0, 0.0025, 0, 0.2, 0)  //BR
            },
            new PIDFGains[]{
                    new PIDFGains(1, 0, 0, 0.07,0.15,0),
                    new PIDFGains(1, 0, 0, 0.07,0.15,0),
                    new PIDFGains(1, 0, 0, 0.07,0.15,0),
                    new PIDFGains(1, 0, 0, 0.07,0.15,0)
            });

    /**
     * the gear ratio (not including circular movement to liner) between the drive motor and the wheel
     */
    public final double DRIVE_GEAR_RATIO;

    /**
     * the gear ratio between the steer motor and the module itself
     */
    public final double STEER_GEAR_RATIO;

    /**
     * the radius of the drive wheel in meters
     */
    public final double WHEEL_RADIUS_METERS;

    /**
     * the circumference of the wheel in meters
     */
    public final double WHEEL_CIRCUMFERENCE_METERS;

    /**
     * the max velocity of the module in meters per second
     */
    public final double MAX_WHEEL_LINEAR_VELOCITY;

    /**
     * if the drive motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean DRIVE_INVERTED;

    /**
     * if the steer motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean STEER_INVERTED;

    /**
     * if the absolute encoder is inverted (meaning positive is counter-clockwise)
     */
    public final boolean ABSOLUTE_ENCODER_INVERTED;

    /**
     * the offset between the zero of the magnet of the encoder and the zero of the module (in rotations)
     */
    public final double[] ABSOLUTE_ZERO_OFFSETS = new double[4];

    /**
     * the PIDF gains for the drive motor
     */
    public final PIDFGains[] DRIVE_MOTOR_PID;

    /**
     * the PIDF gains for the steer motor
     */
    public final PIDFGains[] STEER_MOTOR_PID;

    SwerveModuleConstants(double driveGearRatio, double steerGearRatio, double wheelRadiusMeters,
                          double maxDriveVelocityMetersPerSecond, boolean isDriveInverted, boolean isSteerInverted,
                          boolean isAbsEncoderInverted, double front_left_zeroOffset, double front_right_zeroOffset,
                          double back_left_zeroOffset, double back_right_zeroOffset, PIDFGains[] steerMotorPID,
                          PIDFGains[] driveMotorPID) {

        this.DRIVE_GEAR_RATIO = driveGearRatio;
        this.STEER_GEAR_RATIO = steerGearRatio;
        this.WHEEL_RADIUS_METERS = wheelRadiusMeters;
        this.WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * wheelRadiusMeters;
        this.MAX_WHEEL_LINEAR_VELOCITY = maxDriveVelocityMetersPerSecond;
        this.DRIVE_INVERTED = isDriveInverted;
        this.STEER_INVERTED = isSteerInverted;
        this.ABSOLUTE_ENCODER_INVERTED = isAbsEncoderInverted;

        this.ABSOLUTE_ZERO_OFFSETS[0] = front_left_zeroOffset;
        this.ABSOLUTE_ZERO_OFFSETS[1] = front_right_zeroOffset;
        this.ABSOLUTE_ZERO_OFFSETS[2] = back_left_zeroOffset;
        this.ABSOLUTE_ZERO_OFFSETS[3] = back_right_zeroOffset;

        this.DRIVE_MOTOR_PID = driveMotorPID;
        this.STEER_MOTOR_PID = steerMotorPID;
    }
}
