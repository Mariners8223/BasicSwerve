package frc.util;

import edu.wpi.first.wpilibj.Notifier;

import java.util.ArrayList;

public abstract class RoborioController {

    private static final double DEFAULT_PERIOD = 0.02;

    private static final ArrayList<Runnable> runnables = new ArrayList<>();

    protected static final Notifier notifier = new Notifier(() -> runnables.forEach(Runnable::run));

    protected void addRunnable(Runnable runnable) {
        runnables.add(runnable);
    }

    public static void start() {
        notifier.startPeriodic(DEFAULT_PERIOD);
    }

}
