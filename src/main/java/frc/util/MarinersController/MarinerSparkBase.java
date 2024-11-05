package frc.util.MarinersController;

import com.revrobotics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.PIDFGains;

import java.util.function.Function;

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
                        encoder::getVelocity,
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
    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, PIDFGains gains, double gearRatio, String name) {
        super(gains, name);

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
    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, PIDFGains gains, String name) {
        this(id, isBrushless, type, gains, 1, name);
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
    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile, double gearRatio, String name) {
        super(gains, profile, name);

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
    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile, String name) {
        this(id, isBrushless, type, gains, profile, 1, name);
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
    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, String name) {
        super(name);

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
    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, String name, PIDFGains gains, double gearRatio, double firstDerivativeLimit, double secondDerivativeLimit) {
        super(gains, name);
        super.setProfile(firstDerivativeLimit, secondDerivativeLimit);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        setMeasurements(gearRatio);
    }


    private CANSparkBase createSparkBase(int id, boolean isBrushless, MotorType type) {

        CANSparkBase sparkBase = switch (type) {
            case SPARK_MAX ->
                    new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);
            case SPARK_FLEX ->
                    new CANSparkFlex(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);
        };

        REVLibError error = sparkBase.restoreFactoryDefaults();

        reportError("Error restoring factory defaults", error);

        sparkBase.setControlFramePeriodMs(1000 / BaseController.RUN_HZ);

        error = sparkBase.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 1000 / BaseController.RUN_HZ);

        reportError("Error setting status 0 frame period", error);

        error = sparkBase.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 1000 / BaseController.RUN_HZ);

        reportError("Error setting status 1 frame period", error);

        error = sparkBase.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 1000 / BaseController.RUN_HZ);

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

//    private void setPIDController(PIDFGains gains) {
//        SparkPIDController controller = motor.getPIDController();
//
//        REVLibError error =  controller.setP(gains.getP() / super.measurements.getGearRatio());
//
//        reportError("Error setting P gain", error);
//
//        error = controller.setI(gains.getI() / super.measurements.getGearRatio());
//
//        reportError("Error setting I gain", error);
//
//        error = controller.setD(gains.getD() / super.measurements.getGearRatio());
//
//        reportError("Error setting D gain", error);
//
//        error = controller.setFF(gains.getF() / super.measurements.getGearRatio());
//
//        reportError("Error setting F gain", error);
//
//        error = controller.setIZone(gains.getIZone() / super.measurements.getGearRatio());
//
//        reportError("Error setting I zone", error);
//    }

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
            DriverStation.reportError(message + "for motor" + name + "with ID" + motor.getDeviceId() + ": " + error, false);
        }
    }

    @Override
    protected void updateInputs(BaseControllerInputsAutoLogged inputs) {
        //no current draw for motor
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

    private ControlMode controlMode = ControlMode.DutyCycle;

    @Override
    protected void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    @Override
    protected void stopMotorOutput(){
        motor.stopMotor();
    }

    @Override
    public void run() {
        if(location == ControllerLocation.RIO){
            if(controlMode == ControlMode.DutyCycle) motor.set(motorOutput);
            else motor.setVoltage(motorOutput);
        }
        else{
            CANSparkBase.ControlType controlType = switch (controlMode){
                case Stopped, DutyCycle -> CANSparkBase.ControlType.kDutyCycle;
                case Voltage -> CANSparkBase.ControlType.kVoltage;
                case Velocity, ProfiledVelocity -> CANSparkBase.ControlType.kVelocity;
                case Position, ProfiledPosition -> CANSparkBase.ControlType.kPosition;
            };

            motor.getPIDController().setReference(motorOutput, controlType);
        }
    }
}
