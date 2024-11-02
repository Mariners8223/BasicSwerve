package frc.util.MarinersController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.util.PIDFGains;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

public abstract class BaseController implements Runnable {

    public enum ControlMode {
        Stopped,
        DutyCycle,
        Voltage,
        Position,
        Velocity,
        ProfiledPosition,
        ProfiledVelocity,
    }

    @AutoLog
    public static class BaseControllerInputs{
        public double position = 0;
        public double velocity = 0;
        public String controlMode = "DutyCycle";
        public double setpoint = 0;
        public double goal = 0;
        public double temperature = 0;
        public double currentDraw = 0;
        public double currentOutput = 0;
        public double voltageOutput = 0;
        public double voltageInput = 0;
        public double dutyCycle = 0;
        public String currentFaults = "";
    }

    /**
     * The frequency at which the controller runs in Hz
     */
    public static final int RUN_HZ = 200;

    /**
     * The measurements of the system (position, velocity, acceleration)
     */
    protected MarinersMeasurements measurements;

    /**
     * The PID controller used for the controller
     */
    private PIDController pidController;

    /**
     * The feed forward function used for the controller
     */
    private Function<Double, Double> feedForward;

    /**
     * The profile used for the controller
     */
    private TrapezoidProfile profile;

    /**
     * The maximum and minimum output of the controller in volts
     */
    private double[] maxMinOutput = {12, -12};

    /**
     * The control mode of the controller
     */
    protected final AtomicReference<ControlMode> controlMode = new AtomicReference<>(ControlMode.Stopped);

    /**
     * The setpoint of the controller
     */
    private final AtomicReference<Double> setpoint = new AtomicReference<>(0.0);

    /**
     * The goal of the controller
     */
    private final AtomicReference<TrapezoidProfile.State> goal = new AtomicReference<>(new TrapezoidProfile.State());

    /**
     * The inputs of the controller
     */
    private final BaseControllerInputsAutoLogged inputs = new BaseControllerInputsAutoLogged();

    protected final String name;

    /**
     * The output voltage of the controller
     */
    protected double motorOutput = 0;

    /**
     * runs the controller
     */
    public void runController() {
        measurements.update((double) 1 / RUN_HZ);

        ControlMode controlMode = this.controlMode.get();
        setControlMode(controlMode);

        // If the controller is in a control mode that doesn't require pid control, set the output voltage to the setpoint
        switch (controlMode) {
            case DutyCycle -> {
                motorOutput = MathUtil.clamp(setpoint.get(), maxMinOutput[1] / 12, maxMinOutput[0] / 12);
                run();
                return;
            }

            case Voltage -> {
                motorOutput = MathUtil.clamp(setpoint.get(), maxMinOutput[1], maxMinOutput[0]);
                run();
                return;
            }

            case Stopped -> {
                return;
            }
        }

        // If the controller is in a profiled control mode, check if the profile and goal are set
        if ((controlMode == ControlMode.ProfiledPosition || controlMode == ControlMode.ProfiledVelocity)
                && (profile == null || goal.get() == null)) {
            throw new IllegalStateException("Profiled control mode requires a profile");
        }

        // If the controller is in a pid control mode, check if the pid gains are set
        if(pidController == null){
            throw new IllegalStateException("PID control mode requires pid gains");
        }

        // if the controller is in a profiled control mode, calculate the setpoint based on the profile
        // else set the setpoint to the setpoint
        double setpoint = switch (controlMode) {
            case ProfiledPosition -> profile.calculate((double) 1 / RUN_HZ,
                    new TrapezoidProfile.State(measurements.getPosition(), measurements.getVelocity()), goal.get()).position;
            case ProfiledVelocity -> profile.calculate((double) 1 / RUN_HZ,
                    new TrapezoidProfile.State(measurements.getVelocity(), measurements.getAcceleration()), goal.get()).position;
            default -> this.setpoint.get();
        };

        // calculate the measurement based on the control mode
        double measurement = switch (controlMode) {
            case Position, ProfiledPosition -> measurements.getPosition();
            case Velocity, ProfiledVelocity -> measurements.getVelocity();
            default -> 0;
        };

        // calculate the output of the pid controller
        double output = pidController.calculate(measurement, setpoint);

        // calculate the feed forward output
        output += feedForward.apply(measurements.getPosition()) * setpoint;

        // makes sure the output is within the max and min output
        motorOutput = MathUtil.clamp(output, maxMinOutput[1], maxMinOutput[0]);

        run();
    }

    /**
     * used to setting the control mode for the specific motor controller
     * @param controlMode the control mode of the controller
     */
    protected abstract void setControlMode(ControlMode controlMode);

    /**
     * updates the inputs of the controller
     * this will be used for logging
     */
    public void update(){
        inputs.controlMode = controlMode.get().name();
        inputs.setpoint = setpoint.get();
        TrapezoidProfile.State goal = this.goal.get();

        inputs.goal = goal == null ? 0 : goal.position;
        inputs.position = measurements.getPosition();
        inputs.velocity = measurements.getVelocity();

        updateInputs(inputs);

        Logger.processInputs("Motors/" + name, inputs);
    }

    /**
     * updates the inputs of the controller (only for motor specif logging)
     * @param inputs the inputs of the controller
     */
    protected abstract void updateInputs(BaseControllerInputsAutoLogged inputs);

    /**
     * gets the current control mode of the controller
     * @return the current control mode of the controller
     */
    public ControlMode getControlMode() {
        return controlMode.get();
    }

    /**
     * sets the reference of the controller
     * @param setpoint the setpoint of the controller (needs to be appropriately set for the control mode)
     * @param controlMode the control mode of the controller
     */
    public void setReference(double setpoint, ControlMode controlMode) {
        switch (controlMode) {
            case DutyCycle, Voltage, Position, Velocity -> this.setpoint.set(setpoint);
            case ProfiledPosition, ProfiledVelocity -> goal.set(new TrapezoidProfile.State(setpoint, 0));
        }
        this.controlMode.set(controlMode);
    }

    /**
     * sets the reference of the controller
     * @param goal the goal of the controller (needs to be appropriately set for the control mode)
     *             when using profiled position control mode, this would be the position and velocity of the controlled value
     *             when using profiled velocity control mode, this would be the velocity and acceleration of the controlled value
     * this will be used when the end state first derivative is not zero
     * @param controlMode the control mode of the controller (needs to be ProfiledPosition or ProfiledVelocity)
     */
    public void setReference(TrapezoidProfile.State goal, ControlMode controlMode) {
        if(controlMode != ControlMode.ProfiledPosition && controlMode != ControlMode.ProfiledVelocity){
            throw new IllegalArgumentException("Goal is only valid for Profiled control modes");
        }
        this.controlMode.set(controlMode);
        this.goal.set(goal);
    }

    public void stopMotor(){
        controlMode.set(ControlMode.Stopped);
        motorOutput = 0;
        run();
    }

    /**
     * sets the voltage output of the controller
     * equivalent to {@link #setReference(double, ControlMode)} with ControlMode.Voltage
     * @param voltage the voltage output of the controller
     */
    public void setVoltage(double voltage){
        setReference(voltage, ControlMode.Voltage);
    }

    /**
     * sets the duty cycle output of the controller
     * equivalent to {@link #setReference(double, ControlMode)} with ControlMode.DutyCycle
     * @param dutyCycle the duty cycle output of the controller (from -1 to 1)
     */
    public void setDutyCycle(double dutyCycle){
        setReference(dutyCycle, ControlMode.DutyCycle);
    }

    /**
     * sets the pid gains of the controller
     * @param gains the pid gains of the controller
     */
    public void setPIDF(PIDFGains gains) {
        pidController = gains.createPIDController();
        feedForward = (measurement) -> gains.getF();
    }

    /**
     * sets the pid gains and feed forward of the controller
     * @param gains the pid gains of the controller (F is ignored)
     * @param feedForward the function that calculates the feed forward of the controller based on the measurement
     */
    public void setPIDF(PIDFGains gains, Function<Double, Double> feedForward) {
        pidController = gains.createPIDController();
        this.feedForward = feedForward;
    }

    /**
     * sets the measurements of the controller
     * @param measurements the measurements of the controller
     *                     these are the position, velocity, and acceleration of the controlled value
     */
    public void setMeasurements(MarinersMeasurements measurements) {
        this.measurements = measurements;
    }

    /**
     * sets the max and min output of the controller in volts
     * @param maxMinOutput the max and min output of the controller in volts
     */
    public void setMaxMinOutput(double[] maxMinOutput) {
        this.maxMinOutput = maxMinOutput;
    }

    /**
     * sets the max and min output of the controller in volts
     * @param max the max output of the controller in volts
     * @param min the min output of the controller in volts
     */
    public void setMaxMinOutput(double max, double min) {
        this.maxMinOutput = new double[]{max, min};
    }

    /**
     * sets the profile of the controller
     * @param profile the profile of the controller
     *                (needs to be appropriately set for the control mode)
     *                normally, this would be the max velocity and acceleration of the controlled value
     *                but if used profiled velocity control, this would be the max acceleration and jerk
     */
    public void setProfile(TrapezoidProfile profile) {
        this.profile = profile;
    }

    /**
     * sets the profile of the controller
     * @param constraints the constraints of the profile (needs to be appropriately set for the control mode)
     *                    normally, this would be the max velocity and acceleration of the controlled value
     *                    but if used profiled velocity control, this would be the max acceleration and jerk
     */
    public void setProfile(TrapezoidProfile.Constraints constraints) {
        profile = new TrapezoidProfile(constraints);
    }

    /**
     * sets the profile of the controller
     * @param first_derivative the max value of the first derivative of the controlled value (for example, if using position control, this would be velocity)
     * @param second_derivative the max value of the second derivative of the controlled value (for example, if using position control, this would be acceleration)
     */
    public void setProfile(double first_derivative, double second_derivative) {
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(first_derivative, second_derivative));
    }

    protected BaseController(String name) {
        this.name = name;

        ControllerMaster.getInstance().addController(this);
    }

    protected BaseController(PIDFGains gains, String name) {
        feedForward = (measurement) -> gains.getF();
        pidController = gains.createPIDController();

        this.name = name;

        ControllerMaster.getInstance().addController(this);
    }

    protected BaseController(PIDFGains gains, TrapezoidProfile profile, String name) {
        feedForward = (measurement) -> gains.getF();
        pidController = gains.createPIDController();
        this.profile = profile;

        this.name = name;

        ControllerMaster.getInstance().addController(this);
    }

    protected BaseController(PIDFGains gains, Function<Double, Double> feedForward, String name) {
        this.feedForward = feedForward;
        pidController = gains.createPIDController();

        this.name = name;

        ControllerMaster.getInstance().addController(this);
    }

    protected BaseController(PIDFGains gains, TrapezoidProfile profile, Function<Double, Double> feedForward, String name) {
        this.feedForward = feedForward;
        pidController = gains.createPIDController();
        this.profile = profile;

        this.name = name;

        ControllerMaster.getInstance().addController(this);
    }

    protected BaseController(PIDFGains gains, double[] maxMinOutput, String name) {
        feedForward = (measurement) -> gains.getF();
        pidController = gains.createPIDController();
        this.maxMinOutput = maxMinOutput;

        this.name = name;

        ControllerMaster.getInstance().addController(this);
    }

    protected BaseController(PIDFGains gains, TrapezoidProfile profile, double[] maxMinOutput, String name) {
        feedForward = (measurement) -> gains.getF();
        pidController = gains.createPIDController();
        this.profile = profile;
        this.maxMinOutput = maxMinOutput;

        this.name = name;

        ControllerMaster.getInstance().addController(this);
    }

    protected BaseController(PIDFGains gains, Function<Double, Double> feedForward, double[] maxMinOutput, String name) {

        this.feedForward = feedForward;
        pidController = gains.createPIDController();
        this.maxMinOutput = maxMinOutput;

        this.name = name;

        ControllerMaster.getInstance().addController(this);
    }

    protected BaseController(PIDFGains gains, TrapezoidProfile profile, Function<Double, Double> feedForward, double[] maxMinOutput, String name) {
        this.feedForward = feedForward;
        pidController = gains.createPIDController();
        this.profile = profile;
        this.maxMinOutput = maxMinOutput;

        this.name = name;

        ControllerMaster.getInstance().addController(this);
    }

}
