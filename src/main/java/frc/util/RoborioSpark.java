package frc.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class RoborioSpark extends RoborioController {
    private final CANSparkMax spark;




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
