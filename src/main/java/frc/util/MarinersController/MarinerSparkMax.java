package frc.util.MarinersController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
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


    private CANSparkMax createSparkMax(int id, boolean isBrushless, PIDFGains gains) {
        CANSparkMax sparkMax = new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);

        sparkMax.restoreFactoryDefaults();

        sparkMax.setControlFramePeriodMs(1000 / BaseController.RUN_HZ);

        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 1000 / BaseController.RUN_HZ);
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 1000 / BaseController.RUN_HZ);
        sparkMax.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 1000 / BaseController.RUN_HZ);

        return sparkMax;
    }

    private void setPIDController(PIDFGains gains) {
        SparkPIDController controller = motor.getPIDController();

        controller.setP(gains.getP() / super.measurements.getGearRatio());
        controller.setI(gains.getI() / super.measurements.getGearRatio());
        controller.setD(gains.getD() / super.measurements.getGearRatio());
        controller.setFF(gains.getF() / super.measurements.getGearRatio());
        controller.setIZone(gains.getIZone() / super.measurements.getGearRatio());
    }

    @Override
    protected void updateInputs(BaseControllerInputsAutoLogged inputs) {
        //no current draw for motor
        inputs.currentOutput = motor.getOutputCurrent();
        inputs.dutyCycle = motor.getAppliedOutput();
        inputs.voltageInput = motor.getBusVoltage();
        inputs.voltageOutput = inputs.dutyCycle * inputs.voltageInput;
        inputs.temperature = motor.getMotorTemperature();
    }

    @Override
    public void run() {
        motor.setVoltage(outputVoltage);
    }
}
