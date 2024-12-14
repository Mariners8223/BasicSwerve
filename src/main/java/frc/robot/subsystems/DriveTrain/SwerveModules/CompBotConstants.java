package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.util.PIDFGains;

public enum CompBotConstants {
    FRONT_LEFT(3, 4, 5,
            true, true, 0,
            new PIDFGains(3, 0, 0, 1.8749), //Drive motor PID
            new PIDFGains(15, 10, 2, 0, 0.01, 0), //Steer motor PID
            0.20083, 1.8749, 0.11683, 0, 1, 1),

    FRONT_RIGHT(6, 7, 8,
            true, true, 0,
            new PIDFGains(3, 0, 0, 1.8552), //Drive motor PID
            new PIDFGains(15, 10, 2, 0, 0.01, 0), //Steer motor PID
            0.063408, 1.8552, 0.10686, 0, 1, 1),

    BACK_LEFT(9, 10, 11,
            true, true, 0,
            new PIDFGains(3, 0, 0, 1.9608), //Drive motor PID
            new PIDFGains(15, 10, 2, 0, 0.01, 0), //Steer motor PID
            0.15526, 1.9608, 0.13495, 0, 1, 1),

    BACK_RIGHT(12, 13, 14,
            true, true, 0,
            new PIDFGains(3, 0, 0, 1.8443), //Drive motor PID
            new PIDFGains(15, 10, 2, 0, 0.01, 0), //Steer motor PID
            0.090777, 1.8443, 0.083782, 0, 1, 1);

    public static final double DRIVE_GEAR_RATIO = 5.14;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double WHEEL_RADIUS_METERS = 0.0508;
    public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;

    public static final double MAX_WHEEL_LINEAR_VELOCITY = 4.5;

    public static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getKrakenX60(1);

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

    /**
     * the kS for the drive motor
     * the voltage needed to overcome static friction
     * units are volts
     */
    public final double DRIVE_KS;

    /**
     * the kV for the drive motor
     * units are volts per radian per second (V/rad/s)
     */
    public final double DRIVE_KV;

    /**
     * the kA for the drive motor
     * units are volts per radian per second squared (V/rad/s^2)
     */
    public final double DRIVE_KA;

    /**
     * the kS for the steer motor
     * the voltage needed to overcome static friction
     * units are volts
     */
    public final double STEER_KS;

    /**
     * the kV for the steer motor
     * units are volts per radian per second (V/rad/s)
     */
    public final double STEER_KV;

    /**
     * the kA for the steer motor
     * units are volts per radian per second squared (V/rad/s^2)
     */
    public final double STEER_KA;

    CompBotConstants(int DRIVE_MOTOR_ID, int STEER_MOTOR_ID, int ABSOLUTE_ENCODER_ID, boolean driveInverted,
                    boolean steerInverted, double absZeroOffset, PIDFGains driveMotorPID, PIDFGains steerMotorPID,
                    double DRIVE_KS, double DRIVE_KV, double DRIVE_KA, double STEER_KS, double STEER_KV, double STEER_KA) {
        this.DRIVE_INVERTED = driveInverted;
        this.STEER_INVERTED = steerInverted;
        this.ABSOLUTE_ZERO_OFFSET = absZeroOffset;
        this.DRIVE_MOTOR_PID = driveMotorPID;
        this.STEER_MOTOR_PID = steerMotorPID;

        this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
        this.STEER_MOTOR_ID = STEER_MOTOR_ID;
        this.ABSOLUTE_ENCODER_ID = ABSOLUTE_ENCODER_ID;

        this.DRIVE_KS = DRIVE_KS;
        this.DRIVE_KV = DRIVE_KV;
        this.DRIVE_KA = DRIVE_KA;

        this.STEER_KS = STEER_KS;
        this.STEER_KV = STEER_KV;
        this.STEER_KA = STEER_KA;
    }
}
