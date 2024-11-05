package frc.util.MarinersController;

import com.ctre.phoenix6.controls.*;
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

    public MarinersTalonFX(String name, ControllerLocation location, int id){
        super(name, location);

        this.motor = createMotor(id);
        super.setMeasurements(createMeasurement(1));
    }

    public MarinersTalonFX(String name, ControllerLocation location, int id, PIDFGains gains){
        this(name, location, id, gains, 1);
    }

    public MarinersTalonFX(String name, ControllerLocation location, int id, PIDFGains gains, double gearRatio){
        super(name, location, gains);

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
        if(location == ControllerLocation.RIO){
            if(controlMode == ControlMode.DutyCycle) motor.set(motorOutput);
            else motor.setVoltage(motorOutput);
        }
        else{
            ControlRequest request = switch (controlMode){
                case Stopped -> new StaticBrake();
                case Voltage -> new VoltageOut(motorOutput);
                case DutyCycle -> new DutyCycleOut(motorOutput);
                case Position, ProfiledPosition -> new PositionVoltage(motorOutput);
                case Velocity, ProfiledVelocity -> new VelocityVoltage(motorOutput);
            };

            motor.setControl(request);
        }
    }
}
