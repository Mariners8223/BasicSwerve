package frc.robot.subsystems.DriveTrain;

import com.pathplanner.lib.path.PathConstraints;

import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.DriveTrain.SwerveModules.CompBotConstants;
import frc.robot.subsystems.DriveTrain.SwerveModules.DevBotConstants;
import frc.util.PIDFGains;

public class DriveBaseConstants {
        public static final class PathPlanner{
            public static final boolean PLAN_PATH_TO_STARTING_POINT = true; //if pathplanner should plan a path to the starting point if the robot is not there
            public static final boolean DYNAMIC_RE_PLANNING = true; //if pathplanner should replan the path if the robot is beyond the tolerance or if the spike is too big
            public static final double PATH_ERROR_TOLERANCE = 0.1; //the max error in position before pathPlanner replans the path in meters
            public static final double PATH_ERROR_SPIKE_TOLERANCE = 1; //the max position spike before path planner replans the path in meters

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

        public static final int PIGEON_ID = 2; //the id of the pigeon IMU //TODO change this to the correct ID
}
