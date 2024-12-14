package frc.util.MarinersController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.util.PIDFGains;

import java.util.function.Supplier;

/**
 * A class to control a simulated motor
 * used in conjunction with the {@link MarinersController} class
 * can be used to simulate any motor
 */
public class MarinersSimMotor extends MarinersController {
    /**
     * the motor object
     */
    private final DCMotorSim motor;

    /**
     * the voltage of the master motor (if this motor is a follower) (null if this motor is not a follower)
     */
    private Supplier<Double> motorMasterVoltage;

    /**
     * the output of the motor in volts
     */
    private double motorOutput = 0;

    private MarinersMeasurements createMeasurement() {
        return new MarinersMeasurements(
                () -> {
                    motor.update(1 / RUN_HZ);
                    return motor.getAngularPositionRotations();
                },
                () -> motor.getAngularVelocity().in(Units.RotationsPerSecond),
                () -> motor.getAngularAcceleration().in(Units.RotationsPerSecondPerSecond),
                1
        );
    }

    /**
     * creates the controller
     * @param name the name of the controller (for logging)
     * @param motorType the type of motor to simulate
     * @param gearRatio the gear ratio of the motor (larger than 1 if the motor is geared down)
     * @param momentOfInertia the moment of inertia of the system (in kg m^2)
     */
    public MarinersSimMotor(String name, DCMotor motorType, double gearRatio, double momentOfInertia) {
        super(name, ControllerLocation.RIO);

        motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorType, momentOfInertia, gearRatio),
                motorType.withReduction(gearRatio));

        super.setMeasurements(createMeasurement());
    }

    /**
     * creates the controller
     * using a gear ratio of 1
     * @param name the name of the controller (for logging)
     * @param motorType the type of motor to simulate
     * @param momentOfInertia the moment of inertia of the system (in kg m^2)
     */
    public MarinersSimMotor(String name, DCMotor motorType, double momentOfInertia) {
        this(name, motorType, 1, momentOfInertia);
    }

    public MarinersSimMotor(String name, DCMotor motorType, double kV, double kA, double gearRatio){
        super(name, ControllerLocation.RIO);

        motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA),
                motorType.withReduction(gearRatio));

        super.setMeasurements(createMeasurement());
    }

    /**
     * creates the controller
     * @param name the name of the motor
     * @param motor the sim motor object (don't forget to set the gear ratio in the motor object)
     */
    public MarinersSimMotor(String name, DCMotorSim motor){
        super(name, ControllerLocation.RIO);

        this.motor = motor;

        super.setMeasurements(createMeasurement());
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

        motor.setInputVoltage(0);
    }

    @Override
    protected void setPIDFMotor(PIDFGains gains) {
        //nothing to do here
    }

    @Override
    public void setCurrentLimits(int currentLimit, int currentThreshold) {
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
