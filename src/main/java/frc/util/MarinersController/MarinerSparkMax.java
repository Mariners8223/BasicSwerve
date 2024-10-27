package frc.util.MarinersController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import frc.util.PIDFGains;

public class MarinerSparkMax extends BaseController{

    private final CANSparkMax sparkMax;

    private void setMeasurements(double gearRatio){
        RelativeEncoder encoder = sparkMax.getEncoder();

        super.setMeasurements(
                new MarinersMeasurements(
                        encoder::getPosition,
                        encoder::getVelocity,
                        gearRatio
                )
        );
    }

    public MarinerSparkMax(int id, boolean isBrushless, PIDFGains gains, double gearRatio, String name){
        super(gains, name);

        sparkMax = new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);

        setMeasurements(gearRatio);
    }

    public MarinerSparkMax(int id, boolean isBrushless, PIDFGains gains, String name){
        this(id, isBrushless, gains, 1, name);
    }

    public MarinerSparkMax(int id, boolean isBrushless, PIDFGains gains, ExponentialProfile profile, double gearRatio, String name){
        super(gains, profile, name);

        sparkMax = new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);

        setMeasurements(gearRatio);
    }

    public MarinerSparkMax(int id, boolean isBrushless, PIDFGains gains, ExponentialProfile profile, String name){
        this(id, isBrushless, gains, profile, 1, name);
    }

    public MarinerSparkMax(int id, boolean isBrushless, String name){
        super(name);

        sparkMax = new CANSparkMax(id, isBrushless ? CANSparkMax.MotorType.kBrushless : CANSparkMax.MotorType.kBrushed);

        setMeasurements(1);
    }

    @Override
    protected void updateInputs(BaseControllerInputsAutoLogged inputs) {
        //no current draw for motor
        inputs.currentOutput = sparkMax.getOutputCurrent();
        inputs.dutyCycle = sparkMax.getAppliedOutput();
        inputs.voltageInput = sparkMax.getBusVoltage();
        inputs.voltageOutput = inputs.dutyCycle * inputs.voltageInput;
        inputs.temperature = sparkMax.getMotorTemperature();
    }

    @Override
    public void run() {
        sparkMax.setVoltage(outputVoltage);
    }
}
