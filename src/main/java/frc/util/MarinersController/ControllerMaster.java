package frc.util.MarinersController;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class ControllerMaster extends SubsystemBase {

    private static ControllerMaster instance;

    private final ArrayList<BaseController> controllers = new ArrayList<>();

    public static ControllerMaster getInstance(){
        if(instance == null){
            instance = new ControllerMaster();
        }
        return instance;
    }

    private ControllerMaster(){
        Notifier notifier = new Notifier(this::run);

        notifier.startPeriodic((double) 1 / BaseController.RUN_HZ);
    }

    public void addController(BaseController controller){
        controllers.add(controller);
    }

    @Override
    public void periodic() {
        controllers.forEach(BaseController::update);
    }

    private void run(){
        controllers.forEach(BaseController::run);
    }
}
