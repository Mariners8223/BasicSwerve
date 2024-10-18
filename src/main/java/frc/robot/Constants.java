// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule;

import java.util.Map;

/**
 * Add your docs here.
 */
public class Constants {
    public enum RobotType {
        COMPETITION,
        DEVELOPMENT,
        REPLAY
    }

    public static final RobotType ROBOT_TYPE = RobotType.DEVELOPMENT; //the type of robot the code is running on

    public static final Map<Integer, String> SPARK_MAX_NAMES = Map.of(
            MotorMap.DriveBase.MODULES[SwerveModule.ModuleName.Front_Left.ordinal()][1], "Front_Left",
            MotorMap.DriveBase.MODULES[SwerveModule.ModuleName.Front_Right.ordinal()][1], "Front_Right",
            MotorMap.DriveBase.MODULES[SwerveModule.ModuleName.Back_Left.ordinal()][1], "Back_Left",
            MotorMap.DriveBase.MODULES[SwerveModule.ModuleName.Back_Right.ordinal()][1], "Back_Right"
    );
}
