package frc.util.MarinersController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.util.PIDFGains;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Supplier;

public abstract class BaseController implements Runnable{

    public enum ControlMode{
        DutyCycle,
        Voltage,
        Position,
        Velocity,
    }

    /**
     * The controllers for the motor
     * can be position or velocity controllers
     * have an option to be a profiled controller (for motion profiling)
     */
    private PIDController[] controllers = new PIDController[3];

    private Optional<TrapezoidProfile.Constraints>[] constraints = new Optional[3];

    private final TrapezoidProfile.State currentState = new TrapezoidProfile.State();

    /**
     * The suppliers for the position of the motor (in the chosen units)
     */
    private Supplier<Double> positionSupplier;

    /**
     * The suppliers for the velocity of the motor (in the chosen units)
     */
    private Supplier<Double> velocitySupplier;

    private Supplier<Double> accelerationSupplier;

    /**
     * Feedforward functions for each controller
     * accepts the measurements and returns the feedforward value (can be a constant or based on the measurement)
     * returns the kF not the feedforward value
     */
    private Function<Double, Double>[] feedforward = new Function[3];

    /**
     * The control mode of the motor
     * can be duty cycle, voltage, position, or velocity
     * default is duty cycle
     */
    private ControlMode controlMode = ControlMode.DutyCycle;

    /**
     * The min and max output of the motor (in volts)
     * used for clamping the output of the controller
     * default is -12 to 12 volts
     * min is the first element, max is the second element
     */
    private double[] minMaxOutput = {-12, 12};

    /**
     * The setpoints for the controllers
     * used for setting the setpoint of the controller
     */
    private AtomicReference<Double> setpoints = new AtomicReference();

    private AtomicReference<TrapezoidProfile.State> goal = new AtomicReference<>(new TrapezoidProfile.State());

    /**
     * The current controller being used
     */
    private int currentController = 0;

    protected double voltageOutput = 0;

    protected BaseController(Supplier<Double> positionSupplier, Supplier<Double> velocitySupplier, PIDFGains[] pidfGains, double[] minMaxOutput, Function<Double, Double>[] feedforward) {
        for (int i = 0; i < controllers.length; i++) {
            this.controllers[i] = pidfGains[i].createPIDController();

            this.feedforward[i] = feedforward[i];
        }

        this.minMaxOutput = minMaxOutput;

        this.positionSupplier = positionSupplier;
        this.velocitySupplier = velocitySupplier;
    }

    /**
     * Constructor for the BaseController
     * @param positionSupplier the position supplier for the motor
     * @param velocitySupplier the velocity supplier for the motor
     */
    protected BaseController(Supplier<Double> positionSupplier, Supplier<Double> velocitySupplier) {
        this.positionSupplier = positionSupplier;
        this.velocitySupplier = velocitySupplier;
    }

    /**
     * Constructor for the BaseController
     * @param positionSupplier the position supplier for the motor
     * @param velocitySupplier the velocity supplier for the motor
     * @param gearRatio the gear ratio (multiplier for the position and velocity)
     */
    protected BaseController(Supplier<Double> positionSupplier, Supplier<Double> velocitySupplier, double gearRatio) {
        this(() -> positionSupplier.get() * gearRatio, () -> velocitySupplier.get() * gearRatio);
    }

    protected BaseController(Supplier<Double> positionSupplier, Supplier<Double> velocitySupplier, double[] minMaxOutput) {
        this(positionSupplier, velocitySupplier);
        this.minMaxOutput = minMaxOutput;
    }

    protected BaseController(Supplier<Double> positionSupplier, Supplier<Double> velocitySupplier, PIDFGains pidfGains) {
        this(positionSupplier, velocitySupplier);
        this.controllers[0] = pidfGains.createPIDController();
        this.feedforward[0] = (x) -> pidfGains.getF();
    }

    protected BaseController(Supplier<Double> positionSupplier, Supplier<Double> velocitySupplier, PIDFGains pidfGains, double[] minMaxOutput) {
        this(positionSupplier, velocitySupplier, pidfGains);
        this.minMaxOutput = minMaxOutput;
    }

    public void runController(){

        if(goal.get() != null){
            runWithProfile();
        }
        else{
            runWithOutProfile();
        }
    }

    private void runWithProfile(){
        TrapezoidProfile.State target = goal.get();

        if(controlMode == ControlMode.Voltage || controlMode == ControlMode.DutyCycle){
            voltageOutput = 0;
            return;
        }

        currentState.position = switch (controlMode) {
            case Position -> positionSupplier.get();
            case Velocity -> velocitySupplier.get();
            default -> 0;
        };

        currentState.velocity = switch (controlMode) {
            case Position -> velocitySupplier.get();
            case Velocity -> accelerationSupplier.get();
            default -> 0;
        };

        double setPoint = pro
    }

    private void runWithOutProfile(){
        double reference = setpoints.get();

        if(controlMode == ControlMode.Voltage){
            voltageOutput = MathUtil.clamp(reference, minMaxOutput[0], minMaxOutput[1]);
            return;
        }
        else if(controlMode == ControlMode.DutyCycle){
            voltageOutput = MathUtil.clamp(reference * 12, minMaxOutput[0], minMaxOutput[1]);
            return;
        }

        double measurement = switch (controlMode) {
            case Position -> positionSupplier.get();
            case Velocity -> velocitySupplier.get();
            default -> 0;
        };

        double output = controllers[currentController].calculate(measurement, setpoints.get());

        output += feedforward[currentController].apply(measurement) * setpoints.get();

        voltageOutput = MathUtil.clamp(output, minMaxOutput[0], minMaxOutput[1]);
    }

    public void update(){}
}
