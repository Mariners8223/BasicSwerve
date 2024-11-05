package frc.util.MarinersController;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class ControllerMaster extends SubsystemBase {

    private static ControllerMaster instance;

    private final ArrayList<BaseController> controllers = new ArrayList<>();

    private final ArrayList<Notifier> notifiers = new ArrayList<>();

    public static final double ON_RIO_CONTROLLER_HZ = 50;

    public static final double MOTOR_CONTROLLER_HZ = 1000;

    public static ControllerMaster getInstance(){
        if(instance == null){
            instance = new ControllerMaster();
        }
        return instance;
    }

    private ControllerMaster(){

    }

    public void addController(BaseController controller, BaseController.ControllerLocation location){
        controllers.add(controller);
        Notifier notifier = new Notifier(controller::runController);
        notifier.setName(controller.name + " Notifier");
        notifiers.add(notifier);
        notifier.startPeriodic(1 / switch (location){
            case MOTOR -> MOTOR_CONTROLLER_HZ;
            case RIO -> ON_RIO_CONTROLLER_HZ;
        });
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
