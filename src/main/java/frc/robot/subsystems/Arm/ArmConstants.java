package frc.robot.subsystems.Arm;

public class ArmConstants {
    public enum ArmPos{
        HOME_POSITION(0.0, 0),
        COLLECT_FLOOR_POSITION(0.0, 0),
        COLLECT_SOURCE_POSITION(0.0, 0),
        SHOOT_SUBWOFFER_POSITION(0.0, 0),
        FREE_POSITION(0.0, 0),
        
        UNKNOWN(100, 100);

        private final double alpha;
        private final double beta;
        ArmPos(double alpha, double beta){
            this.alpha = alpha;
            this.beta = beta;
        }

        public double getAlpha(){
            return alpha;
        }

        public double getBeta(){
            return beta;
        }

    }

    public static final double ARM_POSITION_TOLERANCE = 5 / 360;
    
}
