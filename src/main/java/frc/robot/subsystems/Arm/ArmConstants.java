package frc.robot.subsystems.Arm;

import frc.util.PIDFGains;

public class ArmConstants {
    public enum ArmPosition{
        HOME_POSITION(0.0, 0),
        COLLECT_FLOOR_POSITION(0.0, 0),
        COLLECT_SOURCE_POSITION(0.0, 0),
        SHOOT_SUBWOFFER_POSITION(0.0, 0),
        FREE_POSITION(0.0, 0),
        AIM_POSITION(0,0),
        AMP_POSITION(0,0),
        UNKNOWN(100, 100);

        private final double alpha;
        private final double beta;
        ArmPosition(double alpha, double beta){
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

    public static final double ARM_POSITION_TOLERANCE = 5 * Math.PI / 180;
    public static final int LIMIT_SWITCH_PORT = 5;
    public static final double RELATIVE_ENCODER = 0.8819624;
    
    public class AlphaConstants{
        public static final int MOTOR_ID = 12;
        public static final boolean IS_INVERTED = false;

        public static final PIDFGains PID = new PIDFGains(0, 0, 0, 0, ARM_POSITION_TOLERANCE);
        public static final double MIN_OUTPUT_RANGE = -0.1;
        public static final double MAX_OUTPUT_RANGE = 0.2;
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final double SECONDARY_CURRENT_LIMIT = 50;
        public static final float FORWARD_SOFT_LIMIT = 0;
        public static final float REVERSE_SOFT_LIMIT = 0;
        public static final double GEAR_RATIO = 9*3*2; //TODO: check the ratio
    }
    
    public class BetaConstants{
        public static final int MOTOR_ID = 13;
        public static final boolean IS_INVERTED = false;

        public static final PIDFGains PID = new PIDFGains(0, 0, 0, 0, ARM_POSITION_TOLERANCE);
        public static final double MIN_OUTPUT_RANGE = -0.1;
        public static final double MAX_OUTPUT_RANGE = 0.1;
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final double SECONDARY_CURRENT_LIMIT = 50;
        public static final float FORWARD_SOFT_LIMIT = 0;
        public static final float REVERSE_SOFT_LIMIT = 0;
        public static final double GEAR_RATIO = 30 / 12 * 9 *3; //TODO: check the ratio
    }
}
