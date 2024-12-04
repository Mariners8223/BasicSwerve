package frc.util.MarinersController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.util.PIDFGains;

import java.util.function.Supplier;

public class MarinersSimMotor extends MarinersController {
    private final DCMotorSim motor;

    private Supplier<Double> motorMasterVoltage;

    private double motorOutput = 0;

    public MarinersSimMotor(String name, DCMotor motorType, double gearRatio, double momentOfInertia) {
        super(name, ControllerLocation.RIO);

        motor = new DCMotorSim(motorType, gearRatio, momentOfInertia);


        super.setMeasurements(new MarinersMeasurements(
                motor::getAngularPositionRotations,
                () -> motor.getAngularVelocityRPM() / 60,
                1
        ));
    }

    public MarinersSimMotor(String name, DCMotor motorType, double momentOfInertia) {
        this(name, motorType, 1, momentOfInertia);
    }

    @Override
    protected void setOutput(double output, ControlMode controlMode, double feedForward) {
        switch (controlMode) {
            case Position, ProfiledPosition, Velocity, ProfiledVelocity ->
                    throw new UnsupportedOperationException("can't use motor controller in simulation mode for position or velocity control");
            case Stopped, Follower -> motorOutput = 0;
            case DutyCycle -> motorOutput = output * 12;
            case Voltage -> motorOutput = output;
        }

        if (motorMasterVoltage != null) {
            motorOutput = motorMasterVoltage.get();
        }

        motor.setInputVoltage(motorOutput);
    }

    @Override
    protected void updateInputs(MotorInputs inputs) {
        inputs.currentDraw = motor.getCurrentDrawAmps();
        inputs.voltageInput = 12;
        inputs.powerDraw = inputs.currentDraw * 12;
        inputs.voltageOutput = motorOutput;
        inputs.powerOutput = inputs.powerDraw;
        inputs.currentOutput = inputs.powerOutput / inputs.voltageOutput;
        inputs.temperature = 0;
        inputs.dutyCycle = motorOutput / 12;
    }

    @Override
    protected void setMotorFollower(MarinersController master, boolean invert) {
        assert master instanceof MarinersSimMotor;

        motorMasterVoltage = invert ?
                () -> -((MarinersSimMotor) master).motorOutput :
                () -> ((MarinersSimMotor) master).motorOutput;
    }

    @Override
    public void setMotorIdleMode(boolean brake) {
        //nothing to do here
    }

    @Override
    protected void stopMotorOutput() {
        motorOutput = 0;
    }

    @Override
    protected void setPIDFMotor(PIDFGains gains) {
        //nothing to do here
    }

    @Override
    public void setCurrentLimits(double currentLimit, double currentThreshold) {
        //nothing to do here
    }

    @Override
    protected void setMaxMinOutputMotor(double max, double min) {
        //nothing to do here
    }

    @Override
    protected void setMotorDeadBandDutyCycleMotor(double deadBand) {
        //nothing to do here
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        //nothing to do here
    }

    @Override
    public void resetMotorEncoder() {
        motor.setState(0, 0);
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        motor.setState(position, motor.getAngularVelocityRPM());
    }
}
