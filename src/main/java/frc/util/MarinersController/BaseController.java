package frc.util.MarinersController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import frc.util.PIDFGains;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

public abstract class BaseController implements Runnable {

    public enum ControlMode {
        DutyCycle,
        Voltage,
        Position,
        Velocity,
        ProfiledPosition,
        ProfiledVelocity,
    }

    /**
     * The frequency at which the controller runs in Hz
     */
    public static final int RUN_HZ = 200;

    /**
     * The measurements of the system (position, velocity, acceleration)
     */
    private MarinersMeasurements measurements;

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
    private ExponentialProfile profile;

    /**
     * The maximum and minimum output of the controller in volts
     */
    private double[] maxMinOutput = {12, -12};

    /**
     * The control mode of the controller
     */
    private final AtomicReference<ControlMode> controlMode = new AtomicReference<>(ControlMode.DutyCycle);

    /**
     * The setpoint of the controller
     */
    private final AtomicReference<Double> setpoint = new AtomicReference<>(0.0);

    /**
     * The goal of the controller
     */
    private final AtomicReference<ExponentialProfile.State> goal = new AtomicReference<>(new ExponentialProfile.State());

    /**
     * The output voltage of the controller
     */
    protected double outputVoltage = 0;

    public void run() {
        measurements.update((double) 1 / RUN_HZ);

        switch (controlMode.get()) {
            case DutyCycle -> {
                outputVoltage = MathUtil.clamp(setpoint.get(), maxMinOutput[1] / 12, maxMinOutput[0] / 12);
                return;
            }

            case Voltage -> {
                outputVoltage = MathUtil.clamp(setpoint.get(), maxMinOutput[1], maxMinOutput[0]);
                return;
            }
        }

        if ((controlMode.get() == ControlMode.ProfiledPosition || controlMode.get() == ControlMode.ProfiledVelocity)
                && (profile == null || goal.get() == null)) {
            throw new IllegalStateException("Profiled control mode requires a profile");
        }

        double setpoint = switch (controlMode.get()) {
            case ProfiledPosition -> profile.calculate((double) 1 / RUN_HZ,
                    new ExponentialProfile.State(measurements.getPosition(), measurements.getVelocity()), goal.get()).position;
            case ProfiledVelocity -> profile.calculate((double) 1 / RUN_HZ,
                    new ExponentialProfile.State(measurements.getVelocity(), measurements.getAcceleration()), goal.get()).position;
            default -> this.setpoint.get();
        };

        double measurement = switch (controlMode.get()) {
            case Position, ProfiledPosition -> measurements.getPosition();
            case Velocity, ProfiledVelocity -> measurements.getVelocity();
            default -> 0;
        };

        double output = pidController.calculate(measurement, setpoint);

        output += feedForward.apply(measurements.getPosition()) * setpoint;

        outputVoltage = MathUtil.clamp(output, maxMinOutput[1], maxMinOutput[0]);

        run();
    }

    public ControlMode getControlMode() {
        return controlMode.get();
    }

    public void setReference(double setpoint, ControlMode controlMode) {
        switch (controlMode) {
            case DutyCycle, Voltage, Position, Velocity -> this.setpoint.set(setpoint);
            case ProfiledPosition, ProfiledVelocity -> goal.set(new ExponentialProfile.State(setpoint, 0));
        }
        this.controlMode.set(controlMode);
    }

    public void setReference(ExponentialProfile.State goal, ControlMode controlMode) {
        if(controlMode != ControlMode.ProfiledPosition && controlMode != ControlMode.ProfiledVelocity){
            throw new IllegalArgumentException("Goal is only valid for Profiled control modes");
        }
        this.controlMode.set(controlMode);
        this.goal.set(goal);
    }

    public void setPIDF(PIDFGains gains) {
        pidController = gains.createPIDController();
        feedForward = (measurement) -> gains.getF();
    }

    public void setPIDF(PIDFGains gains, Function<Double, Double> feedForward) {
        pidController = gains.createPIDController();
        this.feedForward = feedForward;
    }

    public void setMeasurements(MarinersMeasurements measurements) {
        this.measurements = measurements;
    }

    public void setMaxMinOutput(double[] maxMinOutput) {
        this.maxMinOutput = maxMinOutput;
    }

    public void setProfile(ExponentialProfile profile) {
        this.profile = profile;
    }

    public void setProfile(ExponentialProfile.Constraints constraints) {
        profile = new ExponentialProfile(constraints);
    }

    protected BaseController(PIDFGains gains, MarinersMeasurements measurements) {
        this.measurements = measurements;
        feedForward = (measurement) -> gains.getF();
        pidController = gains.createPIDController();
    }

    protected BaseController(PIDFGains gains, MarinersMeasurements measurements, ExponentialProfile profile) {
        this.measurements = measurements;
        feedForward = (measurement) -> gains.getF();
        pidController = gains.createPIDController();
        this.profile = profile;
    }

    protected BaseController(PIDFGains gains, MarinersMeasurements measurements, Function<Double, Double> feedForward) {
        this.measurements = measurements;
        this.feedForward = feedForward;
        pidController = gains.createPIDController();
    }

    protected BaseController(PIDFGains gains, MarinersMeasurements measurements, ExponentialProfile profile, Function<Double, Double> feedForward) {
        this.measurements = measurements;
        this.feedForward = feedForward;
        pidController = gains.createPIDController();
        this.profile = profile;
    }

    protected BaseController(PIDFGains gains, MarinersMeasurements measurements, double[] maxMinOutput) {
        this.measurements = measurements;
        feedForward = (measurement) -> gains.getF();
        pidController = gains.createPIDController();
        this.maxMinOutput = maxMinOutput;
    }

    protected BaseController(PIDFGains gains, MarinersMeasurements measurements, ExponentialProfile profile, double[] maxMinOutput) {
        this.measurements = measurements;
        feedForward = (measurement) -> gains.getF();
        pidController = gains.createPIDController();
        this.profile = profile;
        this.maxMinOutput = maxMinOutput;
    }

    protected BaseController(PIDFGains gains, MarinersMeasurements measurements, Function<Double, Double> feedForward, double[] maxMinOutput) {
        this.measurements = measurements;
        this.feedForward = feedForward;
        pidController = gains.createPIDController();
        this.maxMinOutput = maxMinOutput;
    }

    protected BaseController(PIDFGains gains, MarinersMeasurements measurements, ExponentialProfile profile, Function<Double, Double> feedForward, double[] maxMinOutput) {
        this.measurements = measurements;
        this.feedForward = feedForward;
        pidController = gains.createPIDController();
        this.profile = profile;
        this.maxMinOutput = maxMinOutput;
    }

}
