package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Translation3d;
import frc.util.PIDFGains;

public class ArmConstants {
    public enum ArmPosition{
        HOME_POSITION(0.004, 0.04),
        COLLECT_FLOOR_POSITION(-0.04, 0.4),
        COLLECT_SOURCE_POSITION(0.135, 0.04),
        SHOOT_SUBWOFFER_POSITION(0.135, 0.04),
        FREE_POSITION(0.137, 0.04),
        AIM_POSITION(0,0.04),
        AMP_POSITION(0.177,0.266),
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

    public static final double ARM_POSITION_TOLERANCE = 5 / 360;
    public static final int LIMIT_SWITCH_PORT = 5;
    public static final double RELATIVE_ENCODER_OFFSET = 0.8819624;
    public static final double LIMIT_SWITCH_OFFSET = -0.4698357899983724 + 0.5;

    public static final Translation3d ALPHA_DISTANCE_FROM_CENTER = new Translation3d(12.5, 0, 25);
    public static final double DISTANCE_BETWEEN_PIVOTS = 31;

    
    public class AlphaConstants{
        public static final int MOTOR_ID = 12;
        public static final boolean IS_INVERTED = false;

        public static final PIDFGains PID = new PIDFGains(0.1, 0, 0, 0.0028, ArmConstants.ARM_POSITION_TOLERANCE,0);
        public static final double MIN_OUTPUT_RANGE = -0.1;
        public static final double MAX_OUTPUT_RANGE = 0.2;
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final double SECONDARY_CURRENT_LIMIT = 50;
        public static final double FORWARD_SOFT_LIMIT = -0.04;
        public static final double REVERSE_SOFT_LIMIT = 0.25;
        public static final double GEAR_RATIO = 9 * 5 * 2; 
    }
    
    public class BetaConstants{
        public static final int MOTOR_ID = 13;
        public static final boolean IS_INVERTED = false;

        public static final PIDFGains PID = new PIDFGains(0.1, 0, 0, 0.005, ArmConstants.ARM_POSITION_TOLERANCE,0);
        public static final double MIN_OUTPUT_RANGE = 0.3;
        public static final double MAX_OUTPUT_RANGE = 0.3;
        public static final int SMART_CURRENT_LIMIT = 35;
        public static final double SECONDARY_CURRENT_LIMIT = 50;
        public static final double FORWARD_SOFT_LIMIT = 0.5;
        public static final double REVERSE_SOFT_LIMIT = LIMIT_SWITCH_OFFSET;
        public static final double GEAR_RATIO = 45 * 32 / 12; //TODO: check the ratio  32 / 12 * 9 * 5
    }
}
