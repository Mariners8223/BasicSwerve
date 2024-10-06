package frc.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class RoborioSpark extends RoborioController {
    private final CANSparkMax spark;
    ProfiledPIDController pidController;




    public RoborioSpark(int id, CANSparkLowLevel.MotorType type) {
        spark = new CANSparkMax(0, type);




        addRunnable(this::run);
    }

    public CANSparkMax getSpark() {
        return spark;
    }


    private void run(){
        spark.set(0.5);
    }
}
