package frc.util.MarinersController;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.PIDFGains;

/**
 * A class to control a TalonFX motor controller
 *
 * @see MarinersController
 * @see TalonFX
 */
public class MarinersTalonFX extends MarinersController {

    /**
     * the TalonFX motor controller
     */
    private final TalonFX motor;

    /**
     * the configuration for the motor output
     * (needed that info is not lost when changing the motor output)
     */
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    /**
     * create a new measurement object for the motor
     * using the built-in position, velocity, and acceleration
     * this creates a measurement object that waits for all the signals to update before returning the value
     * (that means that it is a blocking call)
     *
     * @param gearRatio the gear ratio of the motor
     * @return the new measurement object
     */
    private MarinersMeasurements createMeasurement(double gearRatio) {
        return new MarinersMeasurements(
                () -> motor.getPosition().getValueAsDouble(),
                () -> motor.getVelocity().getValueAsDouble(),
                () -> motor.getAcceleration().getValueAsDouble(),
                gearRatio
        );
    }

    /**
     * creates the controller
     *
     * @param name     the name of the controller (for logging)
     * @param location the location of the controller (RIO or MOTOR)
     */
    public MarinersTalonFX(String name, ControllerLocation location, int id) {
        super(name, location);

        this.motor = createMotor(id);
        super.setMeasurements(createMeasurement(1));
    }

    /**
     * creates the controller
     *
     * @param name     the name of the controller (for logging)
     * @param location the location of the controller (RIO or MOTOR)
     * @param gains    the PIDF gains for the controller (the units are voltage to measurements units)
     */
    public MarinersTalonFX(String name, ControllerLocation location, int id, PIDFGains gains) {
        this(name, location, id, gains, 1);
    }

    /**
     * creates the controller
     *
     * @param name      the name of the controller (for logging)
     * @param location  the location of the controller (RIO or MOTOR)
     * @param gains     the PIDF gains for the controller
     * @param gearRatio the gear ratio of the motor
     */
    public MarinersTalonFX(String name, ControllerLocation location, int id, PIDFGains gains, double gearRatio) {
        super(name, location);

        this.motor = createMotor(id);

        super.setPIDF(gains);

        super.setMeasurements(createMeasurement(gearRatio));
    }

    /**
     * @return the TalonFX motor controller
     */
    public TalonFX getMotor() {
        return motor;
    }

    /**
     * creates a new TalonFX motor controller
     *
     * @param id the id of the motor controller
     * @return the new motor controller
     */
    private TalonFX createMotor(int id) {
        TalonFX talonFX = new TalonFX(id);

        talonFX.getConfigurator().apply(new TalonFXConfiguration());

        talonFX.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));

        StatusCode error;

        error = talonFX.getPosition().setUpdateFrequency(RUN_HZ);
        reportError("Error setting position update frequency", error);

        error = talonFX.getVelocity().setUpdateFrequency(RUN_HZ);
        reportError("Error setting velocity update frequency", error);

        error = talonFX.getAcceleration().setUpdateFrequency(RUN_HZ);
        reportError("Error setting acceleration update frequency", error);

        error = talonFX.getDeviceTemp().setUpdateFrequency(10);
        reportError("Error setting temperature update frequency", error);

        error = talonFX.getSupplyCurrent().setUpdateFrequency(10);
        reportError("Error setting supply current update frequency", error);

        error = talonFX.getStatorCurrent().setUpdateFrequency(10);
        reportError("Error setting stator current update frequency", error);

        error = talonFX.getSupplyVoltage().setUpdateFrequency(10);
        reportError("Error setting supply voltage update frequency", error);

        error = talonFX.getMotorVoltage().setUpdateFrequency(50);
        reportError("Error setting motor voltage update frequency", error);

        error = talonFX.getDutyCycle().setUpdateFrequency(100);
        reportError("Error setting duty cycle update frequency", error);

        error = talonFX.optimizeBusUtilization();
        reportError("Error optimizing bus utilization", error);

        return talonFX;
    }


    @Override
    protected void setPIDFMotor(PIDFGains gains) {
        Slot0Configs slot0 = new Slot0Configs();

        slot0.kP = gains.getP() / measurements.getGearRatio() / 12;

        slot0.kI = gains.getI() / measurements.getGearRatio() / 12;

        slot0.kD = gains.getD() / measurements.getGearRatio() / 12;

        StatusCode error = motor.getConfigurator().apply(slot0);
        reportError("Error setting PIDF gains", error);
    }

    @Override
    public void setCurrentLimits(double currentLimit, double currentThreshold) {

        if (currentLimit <= 0 || currentThreshold <= 0) {
            DriverStation.reportError("Current limit and threshold must be greater than 0 for motor" + name, false);
            return;
        }

        CurrentLimitsConfigs limit = new CurrentLimitsConfigs();

        limit.SupplyCurrentLimit = currentLimit;

        limit.SupplyCurrentLimitEnable = true;

        limit.SupplyCurrentThreshold = currentThreshold;

        StatusCode error = motor.getConfigurator().apply(limit);
        reportError("Error setting current limits", error);
    }

    @Override
    protected void setMotorSoftLimit(double minimum, double maximum) {
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maximum;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minimum;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        StatusCode error = motor.getConfigurator().apply(config.SoftwareLimitSwitch);
        reportError("Error setting soft limits", error);
    }

    @Override
    protected void disableSoftLimitMotor() {
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        StatusCode error = motor.getConfigurator().apply(config.SoftwareLimitSwitch);
        reportError("Error disabling soft limits", error);
    }

    @Override
    protected void setMaxMinOutputMotor(double max, double min) {
        config.MotorOutput.PeakForwardDutyCycle = max / 12;

        config.MotorOutput.PeakReverseDutyCycle = -Math.abs(min / 12);

        StatusCode error = motor.getConfigurator().apply(config.MotorOutput);
        reportError("Error setting max and min output", error);
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        motor.setInverted(inverted);

        config.MotorOutput.Inverted =
                inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
    }

    @Override
    public void resetMotorEncoder() {
        motor.setPosition(0);
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        motor.setPosition(position * measurements.getGearRatio());
    }

    @Override
    protected void setMotorDeadBandDutyCycleMotor(double deadBand) {
        config.MotorOutput.DutyCycleNeutralDeadband = Math.abs(deadBand);

        StatusCode error = motor.getConfigurator().apply(config.MotorOutput);
        reportError("Error setting deadband", error);
    }


    @Override
    public void setMotorIdleMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        config.MotorOutput.NeutralMode = mode;
        motor.setNeutralMode(mode);
    }

    @Override
    protected void updateInputs(MotorInputs inputs) {
        inputs.currentDraw = motor.getSupplyCurrent().getValueAsDouble();
        inputs.currentOutput = motor.getStatorCurrent().getValueAsDouble();
        inputs.voltageOutput = motor.getMotorVoltage().getValueAsDouble();
        inputs.voltageInput = motor.getSupplyVoltage().getValueAsDouble();
        inputs.powerDraw = inputs.currentDraw * inputs.voltageInput;
        inputs.powerOutput = inputs.currentOutput * inputs.voltageOutput;
        inputs.temperature = motor.getDeviceTemp().getValueAsDouble();
        inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();

    }

    @Override
    protected void setMotorFollower(MarinersController master, boolean invert) {
        assert master instanceof MarinersTalonFX;

        MarinersTalonFX base = (MarinersTalonFX) master;

        Follower follower = new Follower(base.getMotor().getDeviceID(), invert);

        motor.setControl(follower);
    }

    @Override
    protected void stopMotorOutput() {
        motor.stopMotor();
    }

    @Override
    protected void setOutput(double motorOutput, ControlMode controlMode, double feedForward) {
        ControlRequest request = switch (controlMode) {
            case Position, ProfiledPosition -> new PositionVoltage(motorOutput)
                    .withFeedForward(feedForward);

            case Velocity, ProfiledVelocity -> new VelocityVoltage(motorOutput)
                    .withFeedForward(feedForward);

            case Voltage -> new VoltageOut(motorOutput);

            case DutyCycle -> new DutyCycleOut(motorOutput);

            default -> switch (config.MotorOutput.NeutralMode) {
                case Brake -> new StaticBrake();
                case Coast -> new CoastOut();
            };
        };

        StatusCode error = motor.setControl(request);
        reportError("Error setting motor output", error);
}

/**
 * reports an error to the driver station
 *
 * @param message    the message to report
 * @param statusCode the status code of the error
 */
private void reportError(String message, StatusCode statusCode) {
    if (!statusCode.isOK()) {
        DriverStation.reportError(message + " for motor" + name + "with ID" + motor.getDeviceID() + ": " + statusCode, false);
    }
}
}
