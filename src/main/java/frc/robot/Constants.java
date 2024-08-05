// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleConstants;

import frc.util.PIDFGains;

import java.util.Map;

/** Add your docs here. */
public class Constants {
    public enum RobotType{
        COMPETITION,
        DEVELOPMENT,
        REPLAY
    }

    public static final RobotType robotType = RobotType.COMPETITION; //the type of robot the code is running on


    public static final class DriveTrain{

        public static final class PathPlanner{
            public static final boolean planPathToStartingPointIfNotAtIt = true; //if pathplanner should plan a path to the starting point if the robot is not there
            public static final boolean enableDynamicRePlanning = true; //if pathplanner should replan the path if the robot is beyond the tolerance or if the spike is too big
            public static final double pathErrorTolerance = 0.1; //the max error in position before pathPlanner replans the path in meters
            public static final double pathErrorSpikeTolerance = 1; //the max position spike before path planner replans the path in meters

            public static final PathConstraints pathConstraints = new PathConstraints(
                    robotType == RobotType.DEVELOPMENT ?
                            SwerveModuleConstants.DEVBOT.maxDriveVelocityMetersPerSecond :
                            SwerveModuleConstants.COMPBOT.maxDriveVelocityMetersPerSecond,
                    10,
                    10,
                    20); //the constraints for pathPlanner

            public static final PIDFGains thetaPID = new PIDFGains(1.4574, 0, 0); //the pid gains for the PID Controller of the robot angle, units are radians per second
            public static final PIDFGains XYPID = new PIDFGains(5.5, 0.055, 0.05); //the pid gains for the pid controller of the robot's velocity, units are meters per second
        }

        public static final Map<Integer, String> sparkMaxNames = Map.of(
                MotorMap.DriveBase.modules[SwerveModule.ModuleName.Front_Left.ordinal()][1], "Front_Left",
                MotorMap.DriveBase.modules[SwerveModule.ModuleName.Front_Right.ordinal()][1], "Front_Right",
                MotorMap.DriveBase.modules[SwerveModule.ModuleName.Back_Left.ordinal()][1], "Back_Left",
                MotorMap.DriveBase.modules[SwerveModule.ModuleName.Back_Right.ordinal()][1], "Back_Right"
        );
        
    } 
}
