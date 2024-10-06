package frc.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public abstract class RoborioController {

    private static final double DEFAULT_PERIOD = 0.02;

    private static final ArrayList<RoborioController> motors = new ArrayList<>();

    protected static final Notifier notifier = new Notifier(() -> motors.forEach(RoborioController::run));

    protected void addRunnable(RoborioController motor) {
        motors.add(motor);
    }

    protected abstract void run();

    private static final SubsystemBase subsystem = new SubsystemBase() {
        @Override
        public void periodic() {
            motors.forEach(RoborioController::update);
        }
    };

    protected abstract void update();

    public static void start() {
        notifier.startPeriodic(DEFAULT_PERIOD);
        subsystem.setName("RoborioControllers");
    }

}
