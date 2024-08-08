package frc.robot.subsystems.DriveTrain.SwerveModules;

import frc.util.PIDFGains;

public enum SwerveModuleConstants {
    /**
     * usually the chassis
     */
    DEVBOT(6.75, 12.5, 0.0508, 4.75,
            false, true, true,

            0.302, -0.44, -0.164, 0.303,

            new PIDFGains[]{
                    new PIDFGains(8, 0, 0.07, 1 / SwerveModule.moduleThreadHz), //FL
                    new PIDFGains(10, 0, 0.07, 1 / SwerveModule.moduleThreadHz), //FR
                    new PIDFGains(7.5, 0, 0.1, 1 / SwerveModule.moduleThreadHz), //BL
                    new PIDFGains(10, 0, 0.09, 1 / SwerveModule.moduleThreadHz) //BR
            },
            new PIDFGains[]{
                    new PIDFGains(0, 0, 0),
                    new PIDFGains(0, 0, 0),
                    new PIDFGains(0, 0, 0),
                    new PIDFGains(0, 0, 0)
            }),

    /**
     * usually the final robot
     */
    COMPBOT(6.75, 12.5 * 3, 0.0508, 4.75,
            false, false, false,

            0.59625, 0.9368, 0.226, 0.750,

            new PIDFGains[]{
                    new PIDFGains(7, 0, 0.07, 1 / SwerveModule.moduleThreadHz), //FL
                    new PIDFGains(8, 0, 0.07, 1 / SwerveModule.moduleThreadHz), //FR
                    new PIDFGains(7.5, 0, 0.1, 1 / SwerveModule.moduleThreadHz), //BL
                    new PIDFGains(6.5, 0, 0.03, 1 / SwerveModule.moduleThreadHz) //BR
            },
            new PIDFGains[]{
                    new PIDFGains(2.7, 0, 0,1,0,0),
                    new PIDFGains(2.7, 0, 0,1,0,0),
                    new PIDFGains(2.7, 0, 0,1,0,0),
                    new PIDFGains(2.7, 0, 0,1,0,0)
            });

    /**
     * the gear ratio (not including circular movement to liner) between the drive motor and the wheel
     */
    public final double driveGearRatio;

    /**
     * the gear ratio between the steer motor and the module itself
     */
    public final double steerGearRatio;

    /**
     * the radius of the drive wheel in meters
     */
    public final double wheelRadiusMeters;

    /**
     * the circumference of the wheel in meters
     */
    public final double wheelCircumferenceMeters;

    /**
     * the max velocity of the module in meters per second
     */
    public final double maxDriveVelocityMetersPerSecond;

    /**
     * if the drive motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean isDriveInverted;

    /**
     * if the steer motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean isSteerInverted;

    /**
     * if the absolute encoder is inverted (meaning positive is counter-clockwise)
     */
    public final boolean isAbsEncoderInverted;

    /**
     * the offset between the zero of the magnet of the encoder and the zero of the module (in rotations)
     */
    public final double[] abs_zeroOffsets = new double[4];

    /**
     * the PIDF gains for the drive motor
     */
    public final PIDFGains[] driveMotorPID;

    /**
     * the PIDF gains for the steer motor
     */
    public final PIDFGains[] steerMotorPID;

    SwerveModuleConstants(double driveGearRatio, double steerGearRatio, double wheelRadiusMeters,
                          double maxDriveVelocityMetersPerSecond, boolean isDriveInverted, boolean isSteerInverted,
                          boolean isAbsEncoderInverted, double front_left_zeroOffset, double front_right_zeroOffset,
                          double back_left_zeroOffset, double back_right_zeroOffset, PIDFGains[] steerMotorPID,
                          PIDFGains[] driveMotorPID) {

        this.driveGearRatio = driveGearRatio;
        this.steerGearRatio = steerGearRatio;
        this.wheelRadiusMeters = wheelRadiusMeters;
        this.wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters;
        this.maxDriveVelocityMetersPerSecond = maxDriveVelocityMetersPerSecond;
        this.isDriveInverted = isDriveInverted;
        this.isSteerInverted = isSteerInverted;
        this.isAbsEncoderInverted = isAbsEncoderInverted;

        this.abs_zeroOffsets[0] = front_left_zeroOffset;
        this.abs_zeroOffsets[1] = front_right_zeroOffset;
        this.abs_zeroOffsets[2] = back_left_zeroOffset;
        this.abs_zeroOffsets[3] = back_right_zeroOffset;

        this.driveMotorPID = driveMotorPID;
        this.steerMotorPID = steerMotorPID;
    }
}
