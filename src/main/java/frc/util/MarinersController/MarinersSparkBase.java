package frc.util.MarinersController;

import com.revrobotics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.PIDFGains;

import java.util.function.Function;

/**
 * a class for spark motor controllers (for both spark max and spark flex)
 * this class is used for controlling spark motor controllers
 * this class is a subclass of {@link MarinersController}
 * this does automatic error reporting to the driver station
 * and also built in logging in advantage kit
 * has support for basic output control and PIDF control and profiled control
 */
public class MarinersSparkBase extends MarinersController {

    /**
     * the type of motor controller
     */
    public enum MotorType {
        SPARK_MAX,
        SPARK_FLEX
    }

    /**
     * the motor controller object
     */
    private final CANSparkBase motor;

    /**
     * the type of motor controller
     * used for when setting an external encoder
     */
    private final MotorType type;

    /**
     * sets the measurements for the motor controller using the built-in encoder
     *
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by)
     */
    private void setMeasurements(double gearRatio) {
        RelativeEncoder encoder = motor.getEncoder();

        super.setMeasurements(
                new MarinersMeasurements(
                        encoder::getPosition,
                        () -> encoder.getVelocity() / 60,
                        gearRatio
                )
        );
    }

    /**
     * creates a new spark motor controller
     *
     * @param id          the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type        the type of motor controller
     * @param gains       the PIDF gains for the motor controller
     * @param gearRatio   the gear ratio of the motor controller (the value that the measurements will be divided by)
     * @param name        the name of the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, double gearRatio) {
        super(name, location);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        super.setPIDF(gains);

        setMeasurements(gearRatio);
    }

    /**
     * creates a new spark motor controller
     *
     * @param id          the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type        the type of motor controller
     * @param gains       the PIDF gains for the motor controller
     * @param name        the name of the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains) {
        this(name, location, id, isBrushless, type, gains, 1);
    }

    /**
     * creates a new spark motor controller
     *
     * @param id          the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type        the type of motor controller
     * @param gains       the PIDF gains for the motor controller
     * @param profile     the trapezoid profile for the motor controller (used for motion profiling)
     * @param gearRatio   the gear ratio of the motor controller (the value that the measurements will be divided by)
     * @param name        the name of the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile, double gearRatio) {
        super(name, location);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        super.setPIDF(gains);

        super.setProfile(profile);

        setMeasurements(gearRatio);
    }

    /**
     * creates a new spark motor controller
     *
     * @param id          the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type        the type of motor controller
     * @param gains       the PIDF gains for the motor controller
     * @param profile     the trapezoid profile for the motor controller (used for motion profiling)
     * @param name        the name of the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile) {
        this(name, location, id, isBrushless, type, gains, profile, 1);
    }

    /**
     * creates a new spark motor controller
     * this will create a motor that can only be controlled by duty cycle or voltage
     * until {@link #setPIDF(PIDFGains)} or {@link #setPIDF(PIDFGains, Function)} is called
     *
     * @param id          the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type        the type of motor controller
     * @param name        the name of the motor controller
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type) {
        super(name, location);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        setMeasurements(1);
    }

    /**
     * creates a new spark motor controller
     *
     * @param id                    the id of the motor controller
     * @param isBrushless           if the motor is brushless
     * @param type                  the type of motor controller
     * @param name                  the name of the motor controller
     * @param gains                 the PIDF gains for the motor controller
     * @param gearRatio             the gear ratio of the motor controller (the value that the measurements will be divided by)
     * @param firstDerivativeLimit  the first derivative limit for the motor controller (used for motion profiling)
     * @param secondDerivativeLimit the second derivative limit for the motor controller (used for motion profiling)
     */
    public MarinersSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, double gearRatio, double firstDerivativeLimit, double secondDerivativeLimit) {
        super(name, location);
        super.setProfile(firstDerivativeLimit, secondDerivativeLimit);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        super.setPIDF(gains);

        setMeasurements(gearRatio);
    }

    /**
     * if using a thorough bore encoder use this function instead of {@link #useExternalAbsoluteEncoder(boolean, double, double)}
     * this will have more precise measurements and better overall
     * use this only if you have a thorough bore encoder connected to a spark flex
     *
     * @param inverted   if the encoder is inverted (use to set the same direction as the motor)
     * @param zeroOffset the zero offset for the encoder (a value between 0 and 1) that will be removed from the absolute encoder's position
     * @param gearRatio  the gear ratio of the motor controller (the value that the measurements will be divided by) (need to add this if using a gear ratio between the encoder and the motor)
     */
    public void useExternalAbsoluteEncoderUsingRelative(boolean inverted, double zeroOffset, double gearRatio) {

        if (type != MotorType.SPARK_FLEX) {
            throw new IllegalArgumentException("This function is only for spark flex controllers");
        }

        CANSparkFlex sparkFlex = (CANSparkFlex) motor;

        RelativeEncoder encoder = sparkFlex.getExternalEncoder(8192);

        AbsoluteEncoder absoluteEncoder = getAbsoluteEncoder(inverted, zeroOffset, gearRatio);

        REVLibError error;

        error = encoder.setInverted(inverted);
        reportError("Error setting relative encoder inverted", error);

        error = encoder.setPositionConversionFactor(gearRatio);
        reportError("Error setting relative encoder position conversion factor", error);

        error = encoder.setVelocityConversionFactor(gearRatio);
        reportError("Error setting relative encoder velocity conversion factor", error);

        error = encoder.setPosition(absoluteEncoder.getPosition());
        reportError("Error setting relative encoder position", error);

        error = motor.getPIDController().setFeedbackDevice(encoder);
        reportError("Error setting feedback device", error);

        error = motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4, (int) (1000 / RUN_HZ));
        reportError("Error setting status 4 frame period", error);

        super.setMeasurements(
                new MarinersMeasurements(
                        encoder::getPosition,
                        () -> encoder.getVelocity() / 60,
                        gearRatio
                )
        );
    }

    /**
     * if using a thorough bore encoder use this function instead of {@link #useExternalAbsoluteEncoder(boolean, double)}
     * this will have more precise measurements and better overall
     * use this only if you have a thorough bore encoder connected to a spark flex
     *
     * @param inverted   if the encoder is inverted (use to set the same direction as the motor)
     * @param zeroOffset the zero offset for the encoder (a value between 0 and 1) that will be removed from the absolute encoder's position
     */
    public void useExternalAbsoluteEncoderUsingRelative(boolean inverted, double zeroOffset) {
        useExternalAbsoluteEncoderUsingRelative(inverted, zeroOffset, 1);
    }

    /**
     * sets the measurements for the motor controller using an external encoder
     * also works for using the motor's built-in controller
     * if using a spark flex controller and a thorough bore encoder use {@link #useExternalAbsoluteEncoderUsingRelative(boolean, double, double)}
     *
     * @param inverted   if the encoder is inverted (use to set the same direction as the motor)
     * @param zeroOffset the zero offset for the encoder (a value between 0 and 1) that will be removed from the absolute encoder's position
     * @param gearRatio  the gear ratio of the motor controller (the value that the measurements will be divided by) (need to add this if using a gear ratio between the encoder and the motor)
     */
    public void useExternalAbsoluteEncoder(boolean inverted, double zeroOffset, double gearRatio) {
        AbsoluteEncoder encoder = getAbsoluteEncoder(inverted, zeroOffset, gearRatio);

        super.setMeasurements(
                new MarinersMeasurements(
                        encoder::getPosition,
                        gearRatio
                )
        );
    }

    /**
     * sets the measurements for the motor controller using an external encoder
     * also works for using the motor's built-in controller
     * if using a spark flex controller and a thorough bore encoder use {@link #useExternalAbsoluteEncoderUsingRelative(boolean, double)}
     *
     * @param inverted   if the encoder is inverted (use to set the same direction as the motor)
     * @param zeroOffset the zero offset for the encoder (a value between 0 and 1) that will be removed from the absolute encoder's position
     */
    public void useExternalAbsoluteEncoder(boolean inverted, double zeroOffset) {
        useExternalAbsoluteEncoder(inverted, zeroOffset, 1);
    }

    /**
     * gets the absolute encoder for the motor controller
     *
     * @param inverted   if the encoder is inverted
     * @param zeroOffset the zero offset for the encoder
     * @param gearRatio  the gear ratio of the motor controller
     * @return the absolute encoder for the motor controller
     */
    private AbsoluteEncoder getAbsoluteEncoder(boolean inverted, double zeroOffset, double gearRatio) {
        AbsoluteEncoder encoder = motor.getAbsoluteEncoder();

        REVLibError error;

        error = encoder.setInverted(inverted);
        reportError("Error setting absolute encoder inverted", error);

        error = encoder.setZeroOffset(zeroOffset);
        reportError("Error setting absolute encoder zero offset", error);

        error = encoder.setPositionConversionFactor(gearRatio);
        reportError("Error setting absolute encoder position conversion factor", error);

        error = encoder.setVelocityConversionFactor(gearRatio);
        reportError("Error setting absolute encoder velocity conversion factor", error);

        error = motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, (int) (1000 / RUN_HZ));
        reportError("Error setting status 5 frame period", error);

        return encoder;
    }


    /**
     * creates a new spark motor controller
     *
     * @param id          the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type        the type of motor controller
     * @return the new spark motor controller
     */
    private CANSparkBase createSparkBase(int id, boolean isBrushless, MotorType type) {

        CANSparkBase sparkBase = switch (type) {
            case SPARK_MAX ->
                    new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);
            case SPARK_FLEX ->
                    new CANSparkFlex(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);
        };

        REVLibError error = sparkBase.restoreFactoryDefaults();
        reportError("Error restoring factory defaults", error);

        int period = (int) (1000 / RUN_HZ);

        sparkBase.setControlFramePeriodMs(period);

        error = sparkBase.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 1000 / 100);
        reportError("Error setting status 0 frame period", error);

        error = sparkBase.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, period);
        reportError("Error setting status 1 frame period", error);

        error = sparkBase.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, period);
        reportError("Error setting status 2 frame period", error);

        error = sparkBase.enableVoltageCompensation(12);
        reportError("Error enabling voltage compensation", error);

        return sparkBase;
    }

    /**
     * sets the current limits for the motor
     *
     * @param stallCurrentLimit     the current limit for the motor when stalled
     * @param freeSpeedCurrentLimit the current limit for the motor when at free speed
     * @param thresholdRPM          the rpm that above it will be considered free speed
     * @param thresholdCurrentLimit the current threshold for the motor (above this value the motor will be disabled for a short period of time)
     */
    public void setCurrentLimits(int stallCurrentLimit, int freeSpeedCurrentLimit, int thresholdRPM, int thresholdCurrentLimit) {

        REVLibError error = motor.setSmartCurrentLimit(stallCurrentLimit, freeSpeedCurrentLimit, thresholdRPM);
        reportError("Error setting smart current limit", error);

        error = motor.setSecondaryCurrentLimit(thresholdCurrentLimit);
        reportError("Error setting secondary current limit", error);
    }

    /**
     * sets the current limits for the motor
     *
     * @param smartCurrentLimit     the current limit for the motor (the motor will try to stay below this value)
     * @param thresholdCurrentLimit the current threshold for the motor (above this value the motor will be disabled for a short period of time)
     */
    public void setCurrentLimits(int smartCurrentLimit, int thresholdCurrentLimit) {

        REVLibError error = motor.setSmartCurrentLimit(smartCurrentLimit);
        reportError("Error setting smart current limit", error);

        error = motor.setSecondaryCurrentLimit(thresholdCurrentLimit);
        reportError("Error setting secondary current limit", error);
    }

    /**
     * gets the motor object
     *
     * @return the motor object
     */
    public CANSparkBase getMotor() {
        return motor;
    }

    /**
     * reports an error to the driver station
     *
     * @param message the message to report
     * @param error   the error to report
     */
    private void reportError(String message, REVLibError error) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(message + " for motor" + name + "with ID" + motor.getDeviceId() + ": " + error, false);
        }
    }

    @Override
    protected void updateInputs(MotorInputs inputs) {
        inputs.currentOutput = motor.getOutputCurrent();
        inputs.dutyCycle = motor.getAppliedOutput();
        inputs.voltageInput = motor.getBusVoltage();
        inputs.voltageOutput = inputs.dutyCycle * inputs.voltageInput;
        inputs.temperature = motor.getMotorTemperature();
        inputs.currentDraw = (inputs.currentOutput * inputs.voltageInput) / inputs.voltageInput;
        inputs.powerOutput = inputs.voltageOutput * inputs.currentOutput;
        inputs.powerDraw = inputs.voltageInput * inputs.currentDraw;

        inputs.currentFaults = motor.getLastError().name();
    }

    @Override
    protected void setMotorFollower(MarinersController master, boolean invert) {
        assert master instanceof MarinersSparkBase;

        MarinersSparkBase base = (MarinersSparkBase) master;

        motor.follow(base.getMotor());

        motor.setInverted(invert != base.getMotor().getInverted());
    }


    @Override
    public void setMotorIdleMode(boolean brake) {
        motor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }


    @Override
    protected void stopMotorOutput() {
        motor.stopMotor();
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        motor.getEncoder().setPosition(position * measurements.getGearRatio());
    }

    @Override
    protected void setPIDFMotor(PIDFGains gains) {
        SparkPIDController controller = motor.getPIDController();

        REVLibError error;

        error = controller.setP(gains.getP() / measurements.getGearRatio() / 12);
        reportError("Error setting P gain", error);

        error = controller.setI(gains.getI() / measurements.getGearRatio() / 12);
        reportError("Error setting I gain", error);

        error = controller.setD(gains.getD() / measurements.getGearRatio() / 12);
        reportError("Error setting D gain", error);

        error = controller.setIZone(gains.getIZone() / measurements.getGearRatio() / 12);
        reportError("Error setting I zone", error);
    }

    @Override
    public void setCurrentLimits(double currentLimit, double currentThreshold) {
        setCurrentLimits((int) currentLimit, (int) currentThreshold);
    }

    @Override
    protected void setMotorSoftLimit(double minimum, double maximum) {
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) maximum);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) minimum);

        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
    }

    @Override
    protected void disableSoftLimitMotor() {
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false);
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false);
    }

    @Override
    protected void setMaxMinOutputMotor(double max, double min) {
        motor.getPIDController().setOutputRange(-Math.abs(min) / 12, max / 12);
    }

    @Override
    protected void setMotorDeadBandDutyCycleMotor(double deadBand) {
        DriverStation.reportWarning("Dead band for spark controllers is only available for controllers running on rio", false);
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    @Override
    public void resetMotorEncoder() {
        motor.getEncoder().setPosition(0);
    }

    @Override
    protected void setOutput(double output, ControlMode controlMode, double feedForward) {
        CANSparkBase.ControlType controlType = switch (controlMode) {
            case Voltage -> CANSparkBase.ControlType.kVoltage;
            case Velocity, ProfiledVelocity -> CANSparkBase.ControlType.kVelocity;
            case Position, ProfiledPosition -> CANSparkBase.ControlType.kPosition;
            default -> CANSparkBase.ControlType.kDutyCycle;
        };

        REVLibError error = motor.getPIDController().setReference(output, controlType, 0, feedForward);
        reportError("Error setting reference", error);
    }
}
