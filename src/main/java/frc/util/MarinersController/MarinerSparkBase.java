package frc.util.MarinersController;

import com.revrobotics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.PIDFGains;

import java.util.function.Function;

/**
 * a class for spark motor controllers (for both spark max and spark flex)
 * this class is used for controlling spark motor controllers
 * this class is a subclass of {@link BaseController}
 * this does automatic error reporting to the driver station
 * and also built in logging in advantage kit
 * has support for basic output control and PIDF control and profiled control
 */
public class MarinerSparkBase extends BaseController {

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
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param gains the PIDF gains for the motor controller
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by)
     * @param name the name of the motor controller
     */
    public MarinerSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, double gearRatio) {
        super(name, location, gains);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        setMeasurements(gearRatio);
    }

    /**
     * creates a new spark motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param gains the PIDF gains for the motor controller
     * @param name the name of the motor controller
     */
    public MarinerSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains) {
        this(name, location, id, isBrushless, type, gains, 1);
    }

    /**
     * creates a new spark motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param gains the PIDF gains for the motor controller
     * @param profile the trapezoid profile for the motor controller (used for motion profiling)
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by)
     * @param name the name of the motor controller
     */
    public MarinerSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile, double gearRatio) {
        super(name, location, gains, profile);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        setMeasurements(gearRatio);
    }

    /**
     * creates a new spark motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param gains the PIDF gains for the motor controller
     * @param profile the trapezoid profile for the motor controller (used for motion profiling)
     * @param name the name of the motor controller
     */
    public MarinerSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile) {
        this(name, location, id, isBrushless, type, gains, profile, 1);
    }

    /**
     * creates a new spark motor controller
     * this will create a motor that can only be controlled by duty cycle or voltage
     * until {@link #setPIDF(PIDFGains)} or {@link #setPIDF(PIDFGains, Function)} is called
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param name the name of the motor controller
     */
    public MarinerSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type) {
        super(name, location);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        setMeasurements(1);
    }

    /**
     * creates a new spark motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
     * @param name the name of the motor controller
     * @param gains the PIDF gains for the motor controller
     * @param gearRatio the gear ratio of the motor controller (the value that the measurements will be divided by)
     * @param firstDerivativeLimit the first derivative limit for the motor controller (used for motion profiling)
     * @param secondDerivativeLimit the second derivative limit for the motor controller (used for motion profiling)
     */
    public MarinerSparkBase(String name, ControllerLocation location, int id, boolean isBrushless, MotorType type, PIDFGains gains, double gearRatio, double firstDerivativeLimit, double secondDerivativeLimit) {
        super(name, location, gains);
        super.setProfile(firstDerivativeLimit, secondDerivativeLimit);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        setMeasurements(gearRatio);
    }


    /**
     * creates a new spark motor controller
     * @param id the id of the motor controller
     * @param isBrushless if the motor is brushless
     * @param type the type of motor controller
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

        error = sparkBase.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, period);
        reportError("Error setting status 0 frame period", error);

        error = sparkBase.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, period);
        reportError("Error setting status 1 frame period", error);

        error = sparkBase.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, period);
        reportError("Error setting status 2 frame period", error);

        return sparkBase;
    }

    /**
     * sets the current limits for the motor
     * @param stallCurrentLimit the current limit for the motor when stalled
     * @param freeSpeedCurrentLimit the current limit for the motor when at free speed
     * @param thresholdRPM the rpm that above it will be considered free speed
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
     * @param smartCurrentLimit the current limit for the motor (the motor will try to stay below this value)
     * @param thresholdCurrentLimit the current threshold for the motor (above this value the motor will be disabled for a short period of time)
     */
    public void setCurrentLimits(int smartCurrentLimit, int thresholdCurrentLimit){

        REVLibError error = motor.setSmartCurrentLimit(smartCurrentLimit);
        reportError("Error setting smart current limit", error);

        error = motor.setSecondaryCurrentLimit(thresholdCurrentLimit);
        reportError("Error setting secondary current limit", error);
    }

    /**
     * gets the motor object
     * @return the motor object
     */
    public CANSparkBase getMotor(){
        return motor;
    }

    /**
     * reports an error to the driver station
     * @param message the message to report
     * @param error the error to report
     */
    private void reportError(String message, REVLibError error) {
        if(error != REVLibError.kOk){
            DriverStation.reportError(message + " for motor" + name + "with ID" + motor.getDeviceId() + ": " + error, false);
        }
    }

    @Override
    protected void updateInputs(BaseControllerInputsAutoLogged inputs) {
        inputs.currentOutput = motor.getOutputCurrent();
        inputs.dutyCycle = motor.getAppliedOutput();
        inputs.voltageInput = motor.getBusVoltage();
        inputs.voltageOutput = inputs.dutyCycle * inputs.voltageInput;
        inputs.temperature = motor.getMotorTemperature();
        inputs.currentDraw = (inputs.currentOutput * inputs.voltageInput) / inputs.voltageInput;
        inputs.powerOutput = inputs.voltageOutput * inputs.currentOutput;
        inputs.powerDraw = inputs.voltageInput * inputs.currentDraw;

        short faults = motor.getFaults();

        inputs.currentFaults = REVLibError.fromInt(faults).name();
    }


    @Override
    public void setMotorIdleMode(boolean brake){
        motor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }


    @Override
    protected void stopMotorOutput(){
        motor.stopMotor();
    }

    @Override
    protected void setPIDFMotor(PIDFGains gains) {
        SparkPIDController controller = motor.getPIDController();

        REVLibError error;

        error = controller.setP(gains.getP() / measurements.getGearRatio());
        reportError("Error setting P gain", error);

        error = controller.setI(gains.getI() / measurements.getGearRatio());
        reportError("Error setting I gain", error);

        error = controller.setD(gains.getD() / measurements.getGearRatio());
        reportError("Error setting D gain", error);

        error = controller.setIZone(gains.getIZone() / measurements.getGearRatio());
        reportError("Error setting I zone", error);
    }

    @Override
    public void setCurrentLimits(double currentLimit, double currentThreshold) {
        setCurrentLimits((int) currentLimit, (int) currentThreshold);
    }

    @Override
    protected void setMaxMinOutputMotor(double max, double min) {
        motor.getPIDController().setOutputRange(Math.abs(min) / 12, max / 12);
    }

    @Override
    protected void setMotorDeadBandDutyCycleMotor(double deadBand) {
        DriverStation.reportError("Dead band for spark controllers is only available for controllers running on rio", false);
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
    protected void setOutput(double output, ControlMode controlMode) {
        if(location == ControllerLocation.RIO){
            if(controlMode == ControlMode.DutyCycle) motor.set(output);
            else motor.setVoltage(output);
        }
        else{
            double feedForwardValue;
            CANSparkBase.ControlType controlType;

            switch (controlMode){
                case Voltage -> {
                    controlType = CANSparkBase.ControlType.kVoltage;
                    feedForwardValue = 0;
                }
                case Velocity, ProfiledVelocity -> {
                    controlType = CANSparkBase.ControlType.kVelocity;
                    feedForwardValue = feedForward.apply(measurements.getPosition()) * output;
                }
                case Position, ProfiledPosition -> {
                    controlType = CANSparkBase.ControlType.kPosition;
                    feedForwardValue = feedForward.apply(measurements.getVelocity()) * output;
                }
                default -> {
                    controlType = CANSparkBase.ControlType.kDutyCycle;
                    feedForwardValue = 0;
                }
            }

            REVLibError error = motor.getPIDController().setReference(output, controlType, 0, feedForwardValue);
            reportError("Error setting reference", error);
        }
    }
}
