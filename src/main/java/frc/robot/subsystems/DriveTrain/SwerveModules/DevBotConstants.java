package frc.robot.subsystems.DriveTrain.SwerveModules;

import frc.util.PIDFGains;

public enum DevBotConstants {
    FRONT_LEFT(2, 3, 4,
            true, true, 0,
            new PIDFGains(0.1, 0, 0), //Drive motor PID
            new PIDFGains(0.1, 0, 0)), //Steer motor PID

    FRONT_RIGHT(5, 6, 7,
            true, true, 0,
            new PIDFGains(0.1, 0, 0), //Drive motor PID
            new PIDFGains(0.1, 0, 0)), //Steer motor PID

    BACK_LEFT(8, 9, 10,
            true, true, 0,
            new PIDFGains(0.1, 0, 0), //Drive motor PID
            new PIDFGains(0.1, 0, 0)), //Steer motor PID

    BACK_RIGHT(11, 12, 13,
            true, true, 0,
            new PIDFGains(0.1, 0, 0), //Drive motor PID
            new PIDFGains(0.1, 0, 0)); //Steer motor PID

    public static final double DRIVE_GEAR_RATIO = 5.14;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double WHEEL_RADIUS_METERS = 0.0508;
    public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;

    public static final double MAX_WHEEL_LINEAR_VELOCITY = 4.5;

    /**
     * the motor id for the drive motor
     */
    public final int DRIVE_MOTOR_ID;

    /**
     * the motor id for the steer motor
     */
    public final int STEER_MOTOR_ID;

    /**
     * the motor id for the absolute encoder
     */
    public final int ABSOLUTE_ENCODER_ID;

    /**
     * if the drive motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean DRIVE_INVERTED;

    /**
     * if the steer motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean STEER_INVERTED;

    /**
     * the offset between the zero of the magnet of the encoder and the zero of the module (in rotations)
     */
    public final double ABSOLUTE_ZERO_OFFSET;

    /**
     * the PIDF gains for the drive motor
     */
    public final PIDFGains DRIVE_MOTOR_PID;

    /**
     * the PIDF gains for the steer motor
     */
    public final PIDFGains STEER_MOTOR_PID;

    DevBotConstants(int DRIVE_MOTOR_ID, int STEER_MOTOR_ID, int ABSOLUTE_ENCODER_ID, boolean driveInverted, boolean steerInverted, double absZeroOffset, PIDFGains driveMotorPID, PIDFGains steerMotorPID) {
        this.DRIVE_INVERTED = driveInverted;
        this.STEER_INVERTED = steerInverted;
        this.ABSOLUTE_ZERO_OFFSET = absZeroOffset;
        this.DRIVE_MOTOR_PID = driveMotorPID;
        this.STEER_MOTOR_PID = steerMotorPID;

        this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
        this.STEER_MOTOR_ID = STEER_MOTOR_ID;
        this.ABSOLUTE_ENCODER_ID = ABSOLUTE_ENCODER_ID;
    }
}
