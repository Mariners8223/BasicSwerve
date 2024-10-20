package frc.util.MarinersController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public static final int RUN_HZ = 200;

    private MarinersMeasurements measurements;

    private PIDController pidController;

    private Function<Double, Double> feedForward = (measurement) -> 0.0;

    private ExponentialProfile profile;

    private double[] maxMinOutput = {12, -12};

    private AtomicReference<ControlMode> controlMode = new AtomicReference<>(ControlMode.DutyCycle);

    private AtomicReference<Double> setpoint = new AtomicReference<>(0.0);

    private AtomicReference<TrapezoidProfile.State> goal = new AtomicReference<>(new TrapezoidProfile.State());

    protected double outputVoltage = 0;

    public void run() {
        measurements.update((double) 1 / RUN_HZ);

        switch (controlMode.get()) {
            case DutyCycle -> {
                outputVoltage = MathUtil.clamp((setpoint.get() + feedForward.apply(measurements.getPosition()) * 12),
                        maxMinOutput[1] / 12, maxMinOutput[0] / 12);
                return;
            }

            case Voltage -> {
                outputVoltage = MathUtil.clamp(setpoint.get() + feedForward.apply(measurements.getPosition()),
                        maxMinOutput[1], maxMinOutput[0]);
                return;
            }
        }

        if ((controlMode.get() == ControlMode.Position || controlMode.get() == ControlMode.Velocity) && profile != null) {
            throw new IllegalStateException("Profiled control mode requires a profile");
        }

        
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
