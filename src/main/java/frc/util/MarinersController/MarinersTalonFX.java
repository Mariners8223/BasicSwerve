package frc.util.MarinersController;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.util.PIDFGains;

public class MarinersTalonFX extends BaseController{
    private final TalonFX motor;

    private MarinersMeasurements createMeasurement(double gearRatio){
        return new MarinersMeasurements(
            motor.getPosition()::getValueAsDouble,
            motor.getVelocity()::getValueAsDouble,
            motor.getAcceleration()::getValueAsDouble,
            gearRatio
        );
    }

    public MarinersTalonFX(int id, String name){
        super(name);

        this.motor = createMotor(id);
        super.setMeasurements(createMeasurement(1));
    }

    public MarinersTalonFX(int id, String name, PIDFGains gains){
        this(id, name, gains, 1);
    }

    public MarinersTalonFX(int id, String name, PIDFGains gains, double gearRatio){
        super(gains, name);

        this.motor = createMotor(id);
        super.setMeasurements(createMeasurement(gearRatio));
    }

    private TalonFX createMotor(int id){
        TalonFX talonFX = new TalonFX(id);

        talonFX.getPosition().setUpdateFrequency(BaseController.RUN_HZ);
        talonFX.getVelocity().setUpdateFrequency(BaseController.RUN_HZ);
        talonFX.getAcceleration().setUpdateFrequency(BaseController.RUN_HZ);

        return talonFX;
    }

    private ControlMode controlMode = ControlMode.DutyCycle;

    @Override
    protected void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    @Override
    protected void updateInputs(BaseControllerInputsAutoLogged inputs) {
        BaseStatusSignal.getLatencyCompensatedValue()

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
    protected void stopMotorOutput() {
        motor.stopMotor();
    }

    @Override
    public void run() {
        if(controlMode == ControlMode.DutyCycle) motor.set(motorOutput);
        else motor.setVoltage(motorOutput);
    }
}
