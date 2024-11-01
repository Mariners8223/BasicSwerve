package frc.util.MarinersController;

import com.revrobotics.*;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.util.PIDFGains;

public class MarinerSparkMax extends BaseController {

    private final CANSparkMax motor;

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

    public MarinerSparkMax(int id, boolean isBrushless, PIDFGains gains, double gearRatio, String name) {
        super(gains, name);

        motor = new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);

        setMeasurements(gearRatio);
        setPIDController(gains);
    }

    public MarinerSparkMax(int id, boolean isBrushless, PIDFGains gains, String name) {
        this(id, isBrushless, gains, 1, name);
    }

    public MarinerSparkMax(int id, boolean isBrushless, PIDFGains gains, ExponentialProfile profile, double gearRatio, String name) {
        super(gains, profile, name);

        motor = new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);

        setMeasurements(gearRatio);
    }

    public MarinerSparkMax(int id, boolean isBrushless, PIDFGains gains, ExponentialProfile profile, String name) {
        this(id, isBrushless, gains, profile, 1, name);
    }

    public MarinerSparkMax(int id, boolean isBrushless, String name) {
        super(name);

        motor = new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);

        setMeasurements(1);
    }


    private CANSparkMax createSparkMax(int id, boolean isBrushless) {
        CANSparkMax sparkMax = new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);

        REVLibError error = sparkMax.restoreFactoryDefaults();

        reportError("Error restoring factory defaults", error);

        sparkMax.setControlFramePeriodMs(1000 / BaseController.RUN_HZ);

        error = sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 1000 / BaseController.RUN_HZ);

        reportError("Error setting status 0 frame period", error);

        error = sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 1000 / BaseController.RUN_HZ);

        reportError("Error setting status 1 frame period", error);

        error = sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 1000 / BaseController.RUN_HZ);

        reportError("Error setting status 2 frame period", error);

        return sparkMax;
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

    @Override
    public void run() {
        motor.setVoltage(outputVoltage);
    }
}
