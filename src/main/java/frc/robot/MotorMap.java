package frc.robot;

import frc.robot.Constants.RobotType;

public class MotorMap {
    public static class DriveBase{
        public static int[][] MODULES = new int[][]{
                //Front Left
            {2, 3, Constants.ROBOT_TYPE == RobotType.DEVELOPMENT ? 7 : 3}, //drive, steer, absEncoder
                //Front Right
            {4, 5, Constants.ROBOT_TYPE == RobotType.DEVELOPMENT ? 8 : 0}, //drive, steer, absEncoder
                //Back Left
            {6, 7, Constants.ROBOT_TYPE == RobotType.DEVELOPMENT ? 6 : 1}, //drive, steer, absEncoder
                //Back Right
            {8, 9, Constants.ROBOT_TYPE == RobotType.DEVELOPMENT ? 9 : 2} //drive, steer, absEncoder
        };
    }
}
