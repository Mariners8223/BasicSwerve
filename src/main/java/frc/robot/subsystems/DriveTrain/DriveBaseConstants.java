package frc.robot.subsystems.DriveTrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.DriveTrain.SwerveModules.CompBotConstants;
import frc.robot.subsystems.DriveTrain.SwerveModules.DevBotConstants;
import frc.util.PIDFGains;

public class DriveBaseConstants {

    public static final double DISTANCE_BETWEEN_WHEELS = 0.58; // the distance between each wheel in meters
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[]{
            new Translation2d(DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(DISTANCE_BETWEEN_WHEELS / 2, -DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS / 2, -DISTANCE_BETWEEN_WHEELS / 2)};


    public static final class PathPlanner {
        public static final boolean PLAN_PATH_TO_STARTING_POINT = true; //if pathplanner should plan a path to the starting point if the robot is not there
        public static final boolean DYNAMIC_RE_PLANNING = true; //if pathplanner should replan the path if the robot is beyond the tolerance or if the spike is too big
        public static final double PATH_ERROR_TOLERANCE = 0.1; //the max error in position before pathPlanner replans the path in meters
        public static final double PATH_ERROR_SPIKE_TOLERANCE = 1; //the max position spike before path planner replans the path in meters


        public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
                PATH_ERROR_TOLERANCE,
                PATH_ERROR_SPIKE_TOLERANCE,
                DISTANCE_BETWEEN_WHEELS,
                Constants.ROBOT_TYPE == RobotType.COMPETITION ?
                        CompBotConstants.DRIVE_MOTOR_MODEL :
                        DevBotConstants.DRIVE_MOTOR_MODEL,
                DISTANCE_BETWEEN_WHEELS,
                PIGEON_ID);

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
                20,
                6.883,
                MODULE_CONFIG,
                MODULE_TRANSLATIONS);

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                Constants.ROBOT_TYPE == RobotType.DEVELOPMENT ?
                        DevBotConstants.MAX_WHEEL_LINEAR_VELOCITY :
                        CompBotConstants.MAX_WHEEL_LINEAR_VELOCITY,
                10,
                10,
                20); //the constraints for pathPlanner

        public static final PIDFGains THETA_PID = new PIDFGains(1.4574, 0, 0); //the pid gains for the PID Controller of the robot angle, units are radians per second
        public static final PIDFGains XY_PID = new PIDFGains(5.5, 0.055, 0.05); //the pid gains for the pid controller of the robot's velocity, units are meters per second
    }

    public static final int PIGEON_ID = 2;
}
