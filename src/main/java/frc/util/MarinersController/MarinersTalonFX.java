package frc.util.MarinersController;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

        talonFX.getPosition().setUpdateFrequency(RUN_HZ);
        talonFX.getVelocity().setUpdateFrequency(RUN_HZ);
        talonFX.getAcceleration().setUpdateFrequency(RUN_HZ);

        return talonFX;
    }

    @Override
    public void setMotorIdleMode(boolean brake){
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
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
    protected void setOutput(double motorOutput, ControlMode controlMode) {
        if(location == ControllerLocation.RIO){
            if(controlMode == ControlMode.DutyCycle) motor.set(motorOutput);
            else motor.setVoltage(motorOutput);
        }
        else{
            ControlRequest request = switch (controlMode){
                case Voltage -> new VoltageOut(motorOutput);

                case Position, ProfiledPosition -> new PositionVoltage(motorOutput)
                        .withFeedForward(motorOutput * feedForward.apply(measurements.getPosition()));

                case Velocity, ProfiledVelocity -> new VelocityVoltage(motorOutput)
                        .withFeedForward(motorOutput * feedForward.apply(measurements.getVelocity()));

                default -> new DutyCycleOut(motorOutput);
            };

            motor.setControl(request);
        }
    }
}
