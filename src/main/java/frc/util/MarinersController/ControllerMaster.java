package frc.util.MarinersController;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class ControllerMaster extends SubsystemBase {

    private static ControllerMaster instance;

    private final ArrayList<BaseController> controllers = new ArrayList<>();

    private final ArrayList<Notifier> notifiers = new ArrayList<>();

    public static ControllerMaster getInstance(){
        if(instance == null){
            instance = new ControllerMaster();
        }
        return instance;
    }

    private ControllerMaster(){

    }

    public void addController(BaseController controller){
        controllers.add(controller);
        Notifier notifier = new Notifier(controller::runController);
        notifier.setName(controller.name + " Notifier");
        notifiers.add(notifier);
        notifier.startPeriodic(1.0 / BaseController.RUN_HZ);
    }

    public void stopLoop(){
        for(Notifier notifier : notifiers){
            notifier.stop();
        }
    }

    @Override
    public void periodic() {
        for(BaseController controller : controllers){
            controller.update();
        }
    }
}
