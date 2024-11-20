package frc.util.MarinersController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotState;
import frc.util.PIDFGains;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Function;

/**
 * A class to control a motor controller
 * this holds the pid controller, feed forward, and profile of the controller
 * this class is used to interface with the motor controller
 * you can run a pid controller on the robo rio or the motor controller
 * you can also add a motion profile to the controller
 * also you can add a feed forward to the controller that is based on the measurement (or static)
 * this class is abstract and is meant to be extended for specific motor controllers
 *
 * @see MarinersMeasurements
 * @see PIDController
 * @see TrapezoidProfile
 */
public abstract class MarinersController {

    /**
     * available control modes for the controller
     */
    public enum ControlMode {
        /**
         * the controller is stopped (that means no output is sent to the motor) (depends on the idle mode of the motor)
         */
        Stopped,

        /**
         * the controller is running in duty cycle control mode
         * the output is between -1 and 1 (where -1 is full reverse and 1 is full forward)
         */
        DutyCycle,

        /**
         * the controller is running in voltage control mode
         * the output is in volts
         * the max and min is dependent on the battery voltage
         */
        Voltage,

        /**
         * the controller is running in position control mode
         * the output is in position units (default is rotation)
         */
        Position,

        /**
         * the controller is running in velocity control mode
         * the output is in velocity units (default is rotation per second)
         */
        Velocity,

        /**
         * the controller is running in profiled position control mode
         * that means there is a motion profile set for the controller
         * the output is in position units (default is rotation)
         */
        ProfiledPosition,

        /**
         * the controller is running in profiled velocity control mode
         * that means there is a motion profile set for the controller
         * the output is in velocity units (default is rotation per second)
         */
        ProfiledVelocity;

        public boolean needPID() {
            return this != DutyCycle && this != Voltage && this != Stopped;
        }

        public boolean needMotionProfile() {
            return this == ProfiledPosition || this == ProfiledVelocity;
        }
    }

    /**
     * the location of the controller where it is running
     * this effects the speed of the thread of the motor
     * and also can effect the amount of can bus traffic
     */
    public enum ControllerLocation {
        /**
         * the controller is running on the robo rio
         * that means the pid is calculated on the robo rio and the output is sent to the motor controller
         * (more can bus traffic)
         */
        RIO,

        /**
         * the controller is running on the motor controller
         * that means the pid is calculated on the motor controller and the output is sent to the motor
         * but also there is a motion profile on the robo rio that can be used
         * (less can bus traffic)
         */
        MOTOR
    }

    @AutoLog
    public static class BaseControllerInputs {
        public double position = 0;
        public double velocity = 0;
        public double acceleration = 0;
        public String controlMode = "Stopped";
        public double setpoint = 0;
        public double goal = 0;
        public double temperature = 0;
        public double currentDraw = 0;
        public double currentOutput = 0;
        public double voltageOutput = 0;
        public double voltageInput = 0;
        public double powerDraw = 0;
        public double powerOutput = 0;
        public double dutyCycle = 0;
        public String currentFaults = "";
    }

    /**
     * The measurements of the system (position, velocity, acceleration)
     */
    protected MarinersMeasurements measurements = new MarinersMeasurements(() -> 0.0, 1);

    /**
     * The frequency that the controller runs at
     */
    public final double RUN_HZ;

    /**
     * The PID controller used for the controller
     */
    private PIDController pidController;

    /**
     * The feed forward function used for the controller
     */
    protected Function<Double, Double> feedForward;

    /**
     * The profile used for the controller
     */
    private TrapezoidProfile profile;

    /**
     * The maximum and minimum output of the controller in volts
     */
    private double[] maxMinOutput = {12, -12};

    /**
     * The lock for the setpoint of the controller
     * this is used to prevent the setpoint from being changed while the controller is running
     * (locks when setpoint, goal, or control mode is being changed or used)
     */
    private final ReentrantLock setpointLock = new ReentrantLock();

    /**
     * The lock for the measurements of the controller
     * this is used to prevent the measurements from being changed while the controller is running
     * (locks when measurements are being updated or being used)
     */
    private final ReentrantLock measurementLock = new ReentrantLock();

    /**
     * The control mode of the controller
     */
    protected ControlMode controlMode = ControlMode.Stopped;

    /**
     * The setpoint of the controller
     */
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    /**
     * The goal of the controller
     */
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    /**
     * The inputs of the controller
     */
    private final BaseControllerInputsAutoLogged inputs = new BaseControllerInputsAutoLogged();

    private Double[] wrappingMinMax;

    /**
     * The deadband of the motor in voltage
     * if the motor output is less than this value, the motor will not put out any power
     */
    private double motorVoltageDeadBand = 0.0;

    /**
     * motor name
     */
    public final String name;

    /**
     * the location of the controller where it is running
     */
    protected final ControllerLocation location;

    /**
     * runs the controller
     */
    public void runController() {
        double dt = 1 / RUN_HZ;

        try {
            measurementLock.lock();
            measurements.update(dt);
        } finally {
            measurementLock.unlock();
        }

        // If the controller is in a control mode that doesn't require pid control, set the output voltage to the setpoint
        if (!controlMode.needPID()) return;

        double output;
        ControlMode controlMode;

        try {
            setpointLock.lock();

            controlMode = this.controlMode;

            double measurement = switch (controlMode) {
                case Position, ProfiledPosition -> measurements.getPosition();
                case Velocity, ProfiledVelocity -> measurements.getVelocity();
                default -> 0;
            };

            if (wrappingMinMax != null && controlMode != ControlMode.Position && controlMode != ControlMode.ProfiledPosition) {
                // Get error which is the smallest distance between goal and measurement
                double errorBound = (wrappingMinMax[1] - wrappingMinMax[0]) / 2.0;
                
                double setpointMinDistance =
                    MathUtil.inputModulus(setpoint.position - measurement, -errorBound, errorBound);
                
                setpoint.position = setpointMinDistance + measurement;

                if(controlMode == ControlMode.ProfiledPosition){
                    double goalMinDistance =
                        MathUtil.inputModulus(goal.position - measurement, -errorBound, errorBound);
                        
                    goal.position = goalMinDistance + measurement;
                }
            }

            if (controlMode.needMotionProfile()) setpoint = profile.calculate(1 / RUN_HZ, setpoint, goal);

            if (location == ControllerLocation.MOTOR) {
                setOutput(setpoint.position * measurements.getGearRatio(), controlMode);
                return;
            }


            // calculate the output of the pid controller
            output = pidController.calculate(measurement, setpoint.position);

            // if the pid controller is at the setpoint, set the output to the feed forward
            if (pidController.atSetpoint()) {
                output = feedForward.apply(measurement) * setpoint.position;
            } else {
                output += feedForward.apply(measurement) * setpoint.position;
            }

            if (Math.abs(output) <= motorVoltageDeadBand) {
                output = 0;
            }
        }
        finally {
            setpointLock.unlock();
        }

        //sends the motor output to the motor
        setOutput(MathUtil.clamp(output, maxMinOutput[1], maxMinOutput[0]), controlMode);
    }


    protected abstract void setOutput(double output, ControlMode controlMode);

    /**
     * updates the inputs of the controller
     * this will be used for logging
     */
    public void update() {
        //locks and updates the setPoints of the controller (setPoint, goal and controlMode)
        try {
            setpointLock.lock();

            inputs.controlMode = controlMode.name();
            inputs.setpoint = setpoint.position;
            TrapezoidProfile.State goal = this.goal;

            inputs.goal = goal == null ? 0 : goal.position;
        } finally {
            setpointLock.unlock();
        }

        //locks and updates the measurements of the controller (position, velocity, acceleration)
        try {
            measurementLock.lock();

            inputs.position = measurements.getPosition();
            inputs.velocity = measurements.getVelocity();
            inputs.acceleration = measurements.getAcceleration();
        } finally {
            measurementLock.unlock();
        }

        updateInputs(inputs);

        Logger.processInputs("Motors/" + name, inputs);
    }

    /**
     * updates the inputs of the controller (only for motor specif logging)
     *
     * @param inputs the inputs of the controller
     */
    protected abstract void updateInputs(BaseControllerInputsAutoLogged inputs);

    /**
     * gets the current control mode of the controller
     *
     * @return the current control mode of the controller
     */
    public ControlMode getControlMode() {
        try {
            setpointLock.lock();
            return controlMode;
        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * gets the current position of the motor after gear ratio (default is rotations)
     *
     * @return the current position of the motor after gear ratio
     */
    public double getPosition() {
        try {
            measurementLock.lock();
            return measurements.getPosition();
        } finally {
            measurementLock.unlock();
        }
    }

    /**
     * gets the current velocity of the motor after gear ratio (default is rotations per second)
     *
     * @return the current velocity of the motor after gear ratio
     */
    public double getVelocity() {
        try {
            measurementLock.lock();
            return measurements.getVelocity();
        } finally {
            measurementLock.unlock();
        }
    }

    /**
     * gets the current acceleration of the motor after gear ratio (default is rotations per second squared)
     *
     * @return the current acceleration of the motor after gear ratio
     */
    public double getAcceleration() {
        try {
            measurementLock.lock();
            return measurements.getAcceleration();
        } finally {
            measurementLock.unlock();
        }
    }

    /**
     * gets the current setpoint of the controller (if using profiled control mode, this will be the next setoint (not the goal always)
     *
     * @return the current setpoint of the controller
     */
    public double getSetpoint() {
        try {
            setpointLock.lock();
            return setpoint.position;
        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * if using profiled control mode this will return the goal of the controller (the end state)
     * if not using profiled control mode, this will return the setpoint of the controller
     *
     * @return the current goal of the controller
     */
    public TrapezoidProfile.State getGoal() {

        try {
            setpointLock.lock();
            return controlMode.needMotionProfile() ? goal : setpoint;
        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * @return true if the pid controller is at the setpoint (within the tolerance)
     */
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    /**
     * sets the reference of the controller
     *
     * @param setpoint    the setpoint of the controller (needs to be appropriately set for the control mode)
     * @param controlMode the control mode of the controller
     */
    public void setReference(double setpoint, ControlMode controlMode) {
        if (controlMode == null) {
            throw new IllegalArgumentException("Control mode cannot be null");
        }

        if (controlMode.needPID() && pidController == null)
            throw new IllegalStateException("PID control on mode requires pid gains");

        if (controlMode.needMotionProfile() && profile == null)
            throw new IllegalStateException("Profiled control mode requires a profile");

        try {
            setpointLock.lock();

            if (RobotState.isDisabled()) {
                if (this.controlMode != ControlMode.Stopped) {
                    stopMotorOutput();
                    this.controlMode = ControlMode.Stopped;

                }
                return;
            }

            this.controlMode = controlMode;

            switch (controlMode) {
                case Stopped -> stopMotorOutput();
                case Voltage, DutyCycle -> setOutput(setpoint, controlMode);
                case Position, Velocity -> this.setpoint.position = setpoint;
                case ProfiledPosition, ProfiledVelocity -> this.goal.position = setpoint;
            }

        } finally {
            setpointLock.unlock();
        }

    }

    /**
     * sets the reference of the controller
     *
     * @param goal        the goal of the controller (needs to be appropriately set for the control mode)
     *                    when using profiled position control mode, this would be the position and velocity of the controlled value
     *                    when using profiled velocity control mode, this would be the velocity and acceleration of the controlled value
     *                    this will be used when the end state first derivative is not zero
     * @param controlMode the control mode of the controller (needs to be ProfiledPosition or ProfiledVelocity)
     */
    public void setReference(TrapezoidProfile.State goal, ControlMode controlMode) {
        if (goal == null) {
            throw new IllegalArgumentException("Goal cannot be null");
        }

        if (controlMode == null) {
            throw new IllegalArgumentException("Control mode cannot be null");
        }

        if (!controlMode.needMotionProfile()) {
            throw new IllegalArgumentException("Goal is only valid for Profiled control modes");
        }

        if (pidController == null)
            throw new IllegalStateException("PID control on mode requires pid gains");

        if (profile == null)
            throw new IllegalStateException("Profiled control mode requires a profile");

        try {
            setpointLock.lock();

            if (RobotState.isDisabled()) {
                if (this.controlMode != ControlMode.Stopped) {
                    stopMotorOutput();
                    this.controlMode = ControlMode.Stopped;

                }
                return;
            }

            this.controlMode = controlMode;
            this.goal = goal;
        } finally {
            setpointLock.unlock();
        }
    }

    /**
     * stops the motor (stops the pid controller and motor output) until a new reference is set
     */
    public void stopMotor() {
        try {
            setpointLock.lock();
            controlMode = ControlMode.Stopped;
        } finally {
            setpointLock.unlock();
        }
        stopMotorOutput();
    }

    /**
     * sets the motor idle mode
     *
     * @param brake true if the motor should brake when idle, false if the motor should coast when idle
     */
    public abstract void setMotorIdleMode(boolean brake);

    /**
     * actually sets the motor output to 0
     */
    protected abstract void stopMotorOutput();

    /**
     * sets the voltage output of the controller
     * equivalent to {@link #setReference(double, ControlMode)} with ControlMode.Voltage
     *
     * @param voltage the voltage output of the controller
     */
    public void setVoltage(double voltage) {
        setReference(voltage, ControlMode.Voltage);
    }

    /**
     * sets the voltage output of the controller
     * equivalent to {@link #setReference(double, ControlMode)} with ControlMode.Voltage
     *
     * @param voltage the voltage output of the controller
     */
    public void setVoltage(Measure<Voltage> voltage) {
        setVoltage(voltage.baseUnitMagnitude());
    }

    /**
     * sets the duty cycle output of the controller
     * equivalent to {@link #setReference(double, ControlMode)} with ControlMode.DutyCycle
     *
     * @param dutyCycle the duty cycle output of the controller (from -1 to 1)
     */
    public void setDutyCycle(double dutyCycle) {
        setReference(dutyCycle, ControlMode.DutyCycle);
    }

    /**
     * sets the pid gains of the controller
     *
     * @param gains the pid gains of the controller
     */
    public void setPIDF(PIDFGains gains) {
        if (gains == null) {
            throw new IllegalArgumentException("Gains cannot be null");
        }

        pidController = gains.createPIDController();
        setPIDFMotor(gains);

        feedForward = (measurement) -> gains.getF();
    }

    protected abstract void setPIDFMotor(PIDFGains gains);

    public abstract void setCurrentLimits(double currentLimit, double currentThreshold);

    /**
     * enable position wrapping
     * use this only when using position or profiled position
     * the array should be the minimum value first then the maximum
     * units are the units of the mesasrments
     * @param minMax the array
     */
    public void enablePositionWrapping(Double[] minMax){
        if(minMax == null){
            wrappingMinMax = null;
            return;
        }
        else if(minMax.length == 2){
            wrappingMinMax = minMax.clone();
            return;
        }
        else throw new IllegalArgumentException("minmax must be null or length of 2");        
    }

    /**
     * enable position wrapping
     * use this only when using position or profile position
     * units are the units of the messurements
     * @param minimum the minimum value
     * @param maximum the maximum value
     */
    public void enablePositionWrapping(double minimum, double maximum){
        enablePositionWrapping(new Double[]{minimum, maximum});
    }

    /**
     * disables the position wrapping
     */
    public void disablePositionWrapping(){
        enablePositionWrapping(null);
    }

    /**
     * @return if position wrapping is enabled
     */
    public boolean isPositionWrappingEnabled(){
        return wrappingMinMax != null;
    }
    /**
     * sets the pid gains and feed forward of the controller
     *
     * @param gains       the pid gains of the controller (F is ignored)
     * @param feedForward the function that calculates the feed forward of the controller based on the measurement
     */
    public void setPIDF(PIDFGains gains, Function<Double, Double> feedForward) {

        if (gains == null) {
            throw new IllegalArgumentException("Gains cannot be null");
        }

        if (feedForward == null) {
            throw new IllegalArgumentException("Feed forward cannot be null");
        }

        pidController = gains.createPIDController();

        setPIDFMotor(gains);

        this.feedForward = feedForward;
    }

    /**
     * sets the measurements of the controller
     *
     * @param measurements the measurements of the controller
     *                     these are the position, velocity, and acceleration of the controlled value
     */
    public void setMeasurements(MarinersMeasurements measurements) {
        if (measurements == null) {
            throw new IllegalArgumentException("Measurements cannot be null");
        }

        this.measurements = measurements;
    }

    public MarinersMeasurements getMeasurements() {
        return measurements;
    }

    /**
     * sets the max and min output of the controller in volts
     *
     * @param maxMinOutput the max and min output of the controller in volts
     */
    public void setMaxMinOutput(double[] maxMinOutput) {
        if (maxMinOutput == null) {
            throw new IllegalArgumentException("Max min output cannot be null");
        }

        setMaxMinOutput(maxMinOutput[0], maxMinOutput[1]);
    }

    /**
     * sets the max and min output of the controller in volts
     *
     * @param max the max output of the controller in volts
     * @param min the min output of the controller in volts
     */
    public void setMaxMinOutput(double max, double min) {
        this.maxMinOutput = new double[]{max, -Math.abs(min)};
        setMaxMinOutputMotor(max, min);
    }

    protected abstract void setMaxMinOutputMotor(double max, double min);

    /**
     * sets the profile of the controller
     *
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
     *
     * @param constraints the constraints of the profile (needs to be appropriately set for the control mode)
     *                    normally, this would be the max velocity and acceleration of the controlled value
     *                    but if used profiled velocity control, this would be the max acceleration and jerk
     */
    public void setProfile(TrapezoidProfile.Constraints constraints) {
        if (constraints == null) {
            throw new IllegalArgumentException("Constraints cannot be null");
        }

        profile = new TrapezoidProfile(constraints);
    }

    /**
     * sets the profile of the controller
     *
     * @param first_derivative  the max value of the first derivative of the controlled value (for example, if using position control, this would be velocity)
     * @param second_derivative the max value of the second derivative of the controlled value (for example, if using position control, this would be acceleration)
     */
    public void setProfile(double first_derivative, double second_derivative) {
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(first_derivative, second_derivative));
    }

    /**
     * sets the deadband of the motor in duty cycle
     * if the motor output is less than this value, the motor will not put out any power
     *
     * @param deadBand the deadband of the motor in duty cycle
     */
    public void setMotorDeadBandDutyCycle(double deadBand) {
        motorVoltageDeadBand = deadBand;
        setMotorDeadBandDutyCycleMotor(deadBand);
    }

    /**
     * sends the deadband to the motor
     *
     * @param deadBand the deadband of the motor in duty cycle
     */
    protected abstract void setMotorDeadBandDutyCycleMotor(double deadBand);

    /**
     * sets the deadband of the motor in voltage
     * if the motor output is less than this value, the motor will not put out any power
     * motors work in duty cycle, so this will convert the voltage to duty cycle (12 volts is 1)
     *
     * @param deadBand the deadband of the motor in voltage
     */
    public void setMotorDeadBandVoltage(double deadBand) {
        setMotorDeadBandDutyCycle(deadBand / 12);
    }

    /**
     * is the motor inverted
     * true if counterclockwise is positive, false if clockwise is positive
     *
     * @param inverted true if the motor is inverted, false if the motor is not inverted
     */
    public abstract void setMotorInverted(boolean inverted);

    /**
     * resets the motor encoder
     * (only works if the measurements are based on the encoder)
     */
    public abstract void resetMotorEncoder();

    /**
     * sets the motor encoder position
     * (only works if the measurements are based on the encoder)
     * the value is multiplied by the gear ratio
     *
     * @param position the position of the motor encoder (default is rotations)
     */
    public abstract void setMotorEncoderPosition(double position);


    /**
     * creates the controller without any pid control or feed forward
     *
     * @param name     the name of the motor
     * @param location the location of the controller where it is running
     */
    protected MarinersController(String name, ControllerLocation location) {
        if (name == null) {
            throw new IllegalArgumentException("Name cannot be null");
        }

        if (name.isBlank()) {
            throw new IllegalArgumentException("Name cannot be blank");
        }

        if (location == null) {
            throw new IllegalArgumentException("Location cannot be null");
        }

        this.name = name;
        this.location = location;

        this.RUN_HZ = ControllerMaster.getInstance().addController(this, location);
    }

    /**
     * creates the controller with pid control and static feed forward
     *
     * @param name     the name of the motor
     * @param location the location of the controller where it is running
     * @param gains    the pid gains of the controller
     */
    protected MarinersController(String name, ControllerLocation location, PIDFGains gains) {
        this(name, location);

        setPIDF(gains);
    }

    /**
     * creates the controller with pid control and static feed forward
     *
     * @param name     the name of the motor
     * @param location the location of the controller where it is running
     * @param gains    the pid gains of the controller
     * @param profile  the profile of the controller
     */
    protected MarinersController(String name, ControllerLocation location, PIDFGains gains, TrapezoidProfile profile) {
        this(name, location);

        setPIDF(gains);
        setProfile(profile);
    }

    /**
     * creates the controller with pid control and feed forward
     *
     * @param name        the name of the motor
     * @param location    the location of the controller where it is running
     * @param gains       the pid gains of the controller
     * @param feedForward the function that calculates the feed forward of the controller based on the measurement
     */
    protected MarinersController(String name, ControllerLocation location, PIDFGains gains, Function<Double, Double> feedForward) {
        this(name, location);

        setPIDF(gains, feedForward);
    }

    /**
     * creates the controller with pid control and feed forward
     *
     * @param name        the name of the motor
     * @param location    the location of the controller where it is running
     * @param gains       the pid gains of the controller
     * @param profile     the profile of the controller
     * @param feedForward the function that calculates the feed forward of the controller based on the measurement
     */
    protected MarinersController(String name, ControllerLocation location, PIDFGains gains, TrapezoidProfile profile, Function<Double, Double> feedForward) {
        this(name, location);

        setPIDF(gains, feedForward);
        setProfile(profile);
    }

    /**
     * creates the controller with pid control and static feed forward
     *
     * @param name         the name of the motor
     * @param location     the location of the controller where it is running
     * @param gains        the pid gains of the controller
     * @param maxMinOutput the max and min output of the controller in volts
     */
    protected MarinersController(String name, ControllerLocation location, PIDFGains gains, double[] maxMinOutput) {
        this(name, location);

        setPIDF(gains);
        setMaxMinOutput(maxMinOutput);
    }

    /**
     * creates the controller with pid control and static feed forward
     *
     * @param name         the name of the motor
     * @param location     the location of the controller where it is running
     * @param gains        the pid gains of the controller
     * @param profile      the profile of the controller
     * @param maxMinOutput the max and min output of the controller in volts
     */
    protected MarinersController(String name, ControllerLocation location, PIDFGains gains, TrapezoidProfile profile, double[] maxMinOutput) {
        this(name, location);

        setPIDF(gains);
        setProfile(profile);
        setMaxMinOutput(maxMinOutput);
    }

    /**
     * creates the controller with pid control and feed forward
     *
     * @param name         the name of the motor
     * @param location     the location of the controller where it is running
     * @param gains        the pid gains of the controller
     * @param feedForward  the function that calculates the feed forward of the controller based on the measurement
     * @param maxMinOutput the max and min output of the controller in volts
     */
    protected MarinersController(String name, ControllerLocation location, PIDFGains gains, Function<Double, Double> feedForward, double[] maxMinOutput) {
        this(name, location);

        setPIDF(gains, feedForward);
        setMaxMinOutput(maxMinOutput);
    }

    /**
     * creates the controller with pid control and feed forward
     *
     * @param name         the name of the motor
     * @param location     the location of the controller where it is running
     * @param gains        the pid gains of the controller
     * @param profile      the profile of the controller
     * @param feedForward  the function that calculates the feed forward of the controller based on the measurement
     * @param maxMinOutput the max and min output of the controller in volts
     */
    protected MarinersController(String name, ControllerLocation location, PIDFGains gains, TrapezoidProfile profile, Function<Double, Double> feedForward, double[] maxMinOutput) {
        this(name, location);

        setPIDF(gains, feedForward);
        setProfile(profile);
        setMaxMinOutput(maxMinOutput);
    }

}
