package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.littletonrobotics.junction.AutoLog;

public abstract class SwerveModuleIO{
    @AutoLog
    static class SwerveModuleIOInputs {
        public SwerveModuleState currentState = new SwerveModuleState();

        public double drivePositionMeters = 0.0;
    }

    /**
     * Updates the inputs of the module
     */
    abstract void updateInputs(SwerveModuleIOInputsAutoLogged inputs);

    /**
     * sets the reference for the drive motor
     *
     * @param reference the target for the built-in PID controller
     */
    public abstract void setDriveMotorReference(double reference);

    /**
     * sets the reference for the drive motor
     * @param reference the target for the built-in PID controller
     * @param accelerationFeedForward the acceleration feed forward in (volts) / (m/s^2)
     */
    public abstract  void setDriveMotorReference(double reference, double accelerationFeedForward);

    /**
     * sets the voltage for the drive motor
     * @param voltage the voltage to set the motor to
     */
    public abstract void setDriveMotorVoltage(double voltage);

    /**
     * sets the reference for the steer motor
     *
     * @param reference the target for the built-in PID controller
     */
    public abstract void setSteerMotorReference(double reference);

    /**
     * sets the voltage for the steer motor
     * @param voltage the voltage to set the motor to
     */
    public abstract void setSteerMotorVoltage(double voltage);

    /**
     * sets the idle mode of the module
     *
     * @param isBrakeMode true for brake mode, false for coast mode
     */
    abstract void setIdleMode(boolean isBrakeMode);

    /**
     * resets the drive encoder
     */
    abstract void resetDriveEncoder();

    abstract void startDriveCalibration();
    abstract void endDriveCalibration();

    abstract void startSteerCalibration();
    abstract void endSteerCalibration();

    /**
     * configures the absolute encoder (duty cycle encoder)
     *
     * @param absEncoderID the port of the abs encoder on the rio
     * @param zeroOffset   the zero offsets of the abs encoder
     * @return the configured abs encoder
     */
    protected DutyCycleEncoder configDutyCycleEncoder(int absEncoderID, double zeroOffset) {
        return new DutyCycleEncoder(absEncoderID, 1, zeroOffset);
    }

    /**
     * configures the absolute encoder (CANCoder)
     *
     * @param absEncoderID              the port of the abs encoder on the CAN bus
     * @param absoluteEncoderZeroOffset the zero offsets of the abs encoder
     * @return the configured CANCoder
     */
    protected CANcoder configCANCoder(int absEncoderID, double absoluteEncoderZeroOffset, int updateRate) {
        CANcoder canCoder = new CANcoder(absEncoderID);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.FutureProofConfigs = false;

        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -absoluteEncoderZeroOffset;

        canCoder.getConfigurator().apply(config);

        canCoder.setPosition(canCoder.getAbsolutePosition().getValueAsDouble());

        canCoder.getPosition().setUpdateFrequency(updateRate);
        canCoder.getVelocity().setUpdateFrequency(updateRate);


        canCoder.optimizeBusUtilization();

        return canCoder;
    }
}
