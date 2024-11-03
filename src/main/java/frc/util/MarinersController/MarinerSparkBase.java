package frc.util.MarinersController;

import com.revrobotics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.PIDFGains;

public class MarinerSparkBase extends BaseController {

    public enum MotorType {
        SPARK_MAX,
        SPAR_FLEX
    }

    private final CANSparkBase motor;

    private final MotorType type;

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

    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, PIDFGains gains, double gearRatio, String name) {
        super(gains, name);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        setMeasurements(gearRatio);
        setPIDController(gains);
    }

    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, PIDFGains gains, String name) {
        this(id, isBrushless, type, gains, 1, name);
    }

    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile, double gearRatio, String name) {
        super(gains, profile, name);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        setMeasurements(gearRatio);
    }

    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, PIDFGains gains, TrapezoidProfile profile, String name) {
        this(id, isBrushless, type, gains, profile, 1, name);
    }

    public MarinerSparkBase(int id, boolean isBrushless, MotorType type, String name) {
        super(name);

        this.type = type;
        motor = createSparkBase(id, isBrushless, type);

        setMeasurements(1);
    }


    private CANSparkBase createSparkBase(int id, boolean isBrushless, MotorType type) {

        CANSparkBase sparkBase = switch (type) {
            case SPARK_MAX ->
                    new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);
            case SPAR_FLEX ->
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

    private void setCurrentLimits(int stallCurrentLimit, int freeSpeedCurrentLimit, int thresholdRPM, int thresholdCurrentLimit) {
        REVLibError error = motor.setSmartCurrentLimit(stallCurrentLimit, freeSpeedCurrentLimit, thresholdRPM);

        reportError("Error setting smart current limit", error);

        error = motor.setSecondaryCurrentLimit(thresholdCurrentLimit);

        reportError("Error setting secondary current limit", error);
    }

    private void setCurrentLimits(int smartCurrentLimit, int thresholdCurrentLimit){
        REVLibError error = motor.setSmartCurrentLimit(smartCurrentLimit);

        reportError("Error setting smart current limit", error);

        error = motor.setSecondaryCurrentLimit(thresholdCurrentLimit);

        reportError("Error setting secondary current limit", error);
    }

    private void setPIDController(PIDFGains gains) {
        SparkPIDController controller = motor.getPIDController();

        REVLibError error =  controller.setP(gains.getP() / super.measurements.getGearRatio());

        reportError("Error setting P gain", error);

        error = controller.setI(gains.getI() / super.measurements.getGearRatio());

        reportError("Error setting I gain", error);

        error = controller.setD(gains.getD() / super.measurements.getGearRatio());

        reportError("Error setting D gain", error);

        error = controller.setFF(gains.getF() / super.measurements.getGearRatio());

        reportError("Error setting F gain", error);

        error = controller.setIZone(gains.getIZone() / super.measurements.getGearRatio());

        reportError("Error setting I zone", error);
    }

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
        if(controlMode == ControlMode.DutyCycle) motor.set(motorOutput);
        else motor.setVoltage(motorOutput);
    }
}
