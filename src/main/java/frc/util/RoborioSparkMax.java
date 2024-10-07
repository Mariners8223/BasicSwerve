package frc.util;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class RoborioSparkMax extends RoborioController {

    public enum ControlMode{
        DutyCycle,
        Voltage,
        Position,
        Velocity,
    }


    private final CANSparkMax spark;
    private final ProfiledPIDController[] controllers = new ProfiledPIDController[3];
    private final Double[] feedforward = new Double[3];

    private final AtomicReference<Double>[] setpoints = new AtomicReference[3];

    private ControlMode controlMode = ControlMode.DutyCycle;

    private int currentController = 0;

    private Supplier<Double> positionSupplier;
    private Supplier<Double> velocitySupplier;

    private double maxOutput = 12;
    private double minOutput = -12;


    public RoborioSparkMax(int id, CANSparkLowLevel.MotorType type) {
        spark = new CANSparkMax(0, type);
        spark.restoreFactoryDefaults();

        for (int i = 0; i < controllers.length; i++) {
            controllers[i] = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        }

        Arrays.fill(feedforward, 0.0);

        for (int i = 0; i < setpoints.length; i++) {
            setpoints[i] = new AtomicReference<Double>(0.0);
        }

        var encoder = spark.getEncoder();
        positionSupplier = encoder::getPosition;
        velocitySupplier = encoder::getVelocity;

        addRunnable(this);
    }

    public CANSparkMax getSpark() {
        return spark;
    }

    private void checkIndex(int index){
        if(index < 0 || index >= controllers.length)
            throw new IndexOutOfBoundsException("controller " + index + " does not exist");
    }

    public void setMaxOutput(double maxOutput){
        this.maxOutput = maxOutput;
    }

    public void setMinOutput(double minOutput){
        this.minOutput = minOutput;
    }

    public void setReferenceDevice(RelativeEncoder referenceDevice){
        setReferenceDevice(referenceDevice::getPosition, referenceDevice::getVelocity);
    }

    public void setReferenceDevice(AbsoluteEncoder referenceDevice){
        setReferenceDevice(referenceDevice::getPosition, referenceDevice::getVelocity);
    }

    public void setReferenceDevice(Supplier<Double> positionSupplier, Supplier<Double> velocitySupplier){
        this.positionSupplier = positionSupplier;
        this.velocitySupplier = velocitySupplier;
    }

    public void setReference(double reference, ControlMode mode){
        controlMode = mode;
        setpoints[0].set(reference);
    }

    public void setReference(double reference, ControlMode mode, int index){
        checkIndex(index);

        controlMode = mode;
        setpoints[index].set(reference);
    }

    public ProfiledPIDController getPIDController(){
        return controllers[0];
    }

    public ProfiledPIDController getPIDController(int index){
        checkIndex(index);

        return controllers[index];
    }

    public void setPIDController(ProfiledPIDController controller){
        controllers[0] = controller;
    }

    public void setPIDController(ProfiledPIDController controller, int index){
        checkIndex(index);

        controllers[index] = controller;
    }

    public void setKp(double kp){
        controllers[0].setP(kp);
    }

    public void setKp(double kp, int index){
        checkIndex(index);

        controllers[index].setP(kp);
    }

    public void setKi(double ki){
        controllers[0].setI(ki);
    }

    public void setKi(double ki, int index){
        checkIndex(index);

        controllers[index].setI(ki);
    }

    public void setKd(double kd){
        controllers[0].setD(kd);
    }

    public void setKd(double kd, int index){
        checkIndex(index);

        controllers[index].setD(kd);
    }

    public void setKf(double feedforward){
        this.feedforward[0] = feedforward;
    }

    public void setKf(double feedforward, int index){
        checkIndex(index);

        this.feedforward[index] = feedforward;
    }


    @Override
    protected void run(){
        ProfiledPIDController controller = controllers[currentController];

        double setpoint = setpoints[currentController].get();

        if(controlMode == ControlMode.DutyCycle){
            spark.set(MathUtil.clamp(setpoint, minOutput / 12, maxOutput / 12));
            return;
        }
        else if(controlMode == ControlMode.Voltage){
            spark.setVoltage(MathUtil.clamp(setpoint, minOutput, maxOutput));
            return;
        }

        double reference = switch (controlMode) {
            case Position -> positionSupplier.get();
            case Velocity -> velocitySupplier.get();
            default -> 0;
        };

        double output = controller.calculate(reference, setpoint);

        output += feedforward[currentController] * setpoint;

        spark.setVoltage(MathUtil.clamp(output, minOutput, maxOutput));
    }

    @Override
    protected void update(){
        currentController = (currentController + 1) % controllers.length;
    }
}
