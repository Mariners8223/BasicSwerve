package frc.robot.subsystems.Arm;

import frc.util.PIDFGains;

public class ArmConstants {
    public enum ArmPos{
        HOME_POSITION(0.0, 0),
        COLLECT_FLOOR_POSITION(0.0, 0),
        COLLECT_SOURCE_POSITION(0.0, 0),
        SHOOT_SUBWOFFER_POSITION(0.0, 0),
        FREE_POSITION(0.0, 0),
        AIM_PSITION(0,0),
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

    public static final int ccountsPerRev = 0;
    public static final int limitSwitchPort = 0;
    
    public class AlphaConstants{
        public static final int MOTOR_ID = 0;
        public static final boolean IS_INVERTED = false;

        public static final PIDFGains PID = new PIDFGains(0, 0, 0, 0, ARM_POSITION_TOLERANCE);
    }
    
    public class BetaConstants{
        public static final int MOTOR_ID = 0;
        public static final boolean IS_INVERTED = false;

        public static final PIDFGains PID = new PIDFGains(0, 0, 0, 0, ARM_POSITION_TOLERANCE);
        public static final double GEAR_RATIO = 0;
    }
}
