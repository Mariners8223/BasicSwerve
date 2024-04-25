// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Translation2d;
import frc.util.PIDFGains;

import java.util.Map;

/** Add your docs here. */
public class Constants {
    public static final class DriveTrain{
        /**
         * the name of the swerve modules by order
         */
        public  enum ModuleName{
            Front_Left,
            Front_Right,
            Back_Left,
            Back_Right
        }

        public static final class Global{
            public static final double maxAcceleration = 40; //the max xy acceleration of the robot in meter per second squared //TODO find real value
            public static final double maxAccelerationRotation = 40; //the max rotation acceleration of the robot in omega radians per second squared //TODO find real value

            public static final double distanceBetweenWheels = 0.576; // the distance between each wheel in meters //TODO update based on new swerve maybe?

            public static final double maxRotationSpeed = 6.27; //the max speed the robot can rotate in (radians per second) //TODO find real value

            @SuppressWarnings("unused")
            public static final double RobotHeightFromGround = 0.155; //the height of the top of the frame from the ground in meters
        }

        public static final class Drive{
            // public static final PIDFGains driveMotorPID = new PIDFGains(0.4, 0.001, 0.001, 0.0, 0.22, 0); //the pid gains for the PID Controller of the drive motor, (units in rotations per second)
            public static final PIDFGains driveMotorPID = new PIDFGains(3.5037 * 6.75, 0.00, 0.00, 0.0, 0.22, 0); //the pid gains for the PID Controller of the drive motor, (units in rotations per second)

            public static final double freeWheelSpeedMetersPerSec = 3.75; //the max speed of the drive wheel in meters per second //TODO find real value

            public static final double driveMotorMaxAcceleration = 4; //the max Acceleration of the drive motor in rotations per second squared (only used by motion magic) //TODO find real value
            public static final double driveMotorMaxJerk = 4.4; //the max jerk of the drive motor in rotations per second cubed (only used by motion magic) //TODO find real value

            public static final double wheelRadiusMeters = 0.0508; //the radius of the drive wheel in meters
            public static final double wheelCircumferenceMeters = wheelRadiusMeters * 2 * Math.PI; //the circumference of the drive wheel in meters
 
            public static final double driveGearRatio = 6.75; //the gear ratio between the drive motor and the wheel (does not include any unit conversion only motor rotations to wheel rotations)

        }

        public static final class Steer{
            public static final double steerGearRatio = 12.5 * 3; //the gear ratio between the steer motor and the module itself
            // public static final double newGearRatio = steerGearRatio * 3; //this is for our old swerve
            // public static final PIDFGains steerMotorPID = new PIDFGains(0.4, 0, 0.1, 0, 0.0005, 0); //the pid gains for the PID Controller of the steer motor, units are in rotations //TODO needs tuning with SysID
            public static final PIDFGains steerMotorPID = new PIDFGains(14.042 * steerGearRatio, 0, 1.4 * steerGearRatio, 0, 0.1 * steerGearRatio, 0); //the pid gains for the PID Controller of the steer motor, units are in rotations //TODO needs tuning with SysID

            public static final double maxVelocity = 1; //the max velocity of the modules steer aspect in module rotations per minute (only used by smart Motion) //TODO find real value
            public static final double minVelocity = 0.1; //the min velocity of the modules steer aspect in module rotation per minute (only used by smart Motion) //TODO find real value
            public static final double maxAcceleration = 1; //the max acceleration of the modules steer aspect in module rotations per minute per second (only used by smart Motion) //TODO find real value

            public static final double front_left_absoluteEncoderZeroOffset = 0.59625; // the offset between the absolute encoder reading on the front left module, in degrees
            public static final double front_right_absoluteEncoderZeroOffset = 0.9368611111111111; // the offset between the absolute encoder on the front left module, in degrees
            public static final double back_left_absoluteEncoderZeroOffset = 0.2263055555555556; // the offset between the absolute encoder on the back left module, in degrees
            public static final double back_right_absoluteEncoderZeroOffset = 0.7502777777777778; // the offset between the absolute encoder on the back right module, in degrees

            // public static final double front_left_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets
            // public static final double front_right_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets
            // public static final double back_left_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets
            // public static final double back_right_absoluteEncoderZeroOffset = 0; // use this to calibrate zero offsets

        }

        public static final class PathPlanner{
            public static final boolean planPathToStartingPointIfNotAtIt = true; //if pathplanner should plan a path to the starting point if the robot is not there
            public static final boolean enableDynamicRePlanning = true; //if pathplanner should replan the path if the robot is beyond the tolerance or if the spike is too big
            public static final double pathErrorTolerance = 0.1; //the max error in position before pathPlanner replans the path in meters
            public static final double pathErrorSpikeTolerance = 1; //the max position spike before path planner replans the path in meters

            public static final PathConstraints pathConstraints = new PathConstraints(
                    Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec, Constants.DriveTrain.Global.maxAcceleration,
                    Constants.DriveTrain.Global.maxRotationSpeed, Constants.DriveTrain.Global.maxAccelerationRotation); //the constraints for pathPlanner


            public static final PIDFGains thetaPID = new PIDFGains(1.4574, 0, 0); //the pid gains for the PID Controller of the robot angle, units are radians per second
            public static final PIDFGains XYPID = new PIDFGains(5.5, 0.055, 0.05); //the pid gains for the pid controller of the robot's velocity, units are meters per second
        }

        public static class SwerveModule{
            public static final Translation2d[] moduleTranslations = new Translation2d[]
                {new Translation2d(Global.distanceBetweenWheels / 2, Global.distanceBetweenWheels / 2), new Translation2d(Global.distanceBetweenWheels / 2, -Global.distanceBetweenWheels / 2),
                 new Translation2d(-Global.distanceBetweenWheels / 2, Global.distanceBetweenWheels / 2), new Translation2d(-Global.distanceBetweenWheels / 2, -Global.distanceBetweenWheels / 2)};
            //^ places the translation of each module in the array in order of the enum (front left, front right, back left, back right)

            public static final double modulesThreadHz = 200; //the frequency of the modules thread in hertz

            public final ModuleName moduleName; //the name of the module (enum)

            public final int driveMotorID; // the CAN ID of the drive motor
            public final int steerMotorID; // the CAN ID of the steer motor
            public final int AbsEncoderID; // the CAN ID of the absolute encoder (this case CanCoder)

            public final boolean isSteerInverted; // if the steer motor output should be reversed
            public final boolean isDriveInverted; // if the drive motor output should be reversed
            public final boolean isAbsEncoderInverted; // if the reading of the absolute encoder should be inverted

            public final boolean shouldSteerMotorBeResetedByAbsEncoder; // if the steer motor should be reseted by the absolute encoder

            public final double absoluteEncoderZeroOffset; //the offset between the magnets zero and the modules zero in degrees

            public final Translation2d moduleTranslation; //the translation of the module relative to the center of the robot

            public SwerveModule(ModuleName moduleName, int driveMotorID, int steerMotorID, int AbsEncoderID, double absoluteEncoderZeroOffset, boolean shouldSteerMotorBeResetByAbsEncoder, boolean isSteerInverted, boolean isDriveInverted, boolean isAbsEncoderInverted){
                this.moduleName = moduleName;

                this.driveMotorID = driveMotorID;
                this.steerMotorID = steerMotorID;
                this.AbsEncoderID = AbsEncoderID;

                this.isDriveInverted = isDriveInverted;
                this.isSteerInverted = isSteerInverted;
                this.isAbsEncoderInverted = isAbsEncoderInverted;

                this.shouldSteerMotorBeResetedByAbsEncoder = shouldSteerMotorBeResetByAbsEncoder;

                this.absoluteEncoderZeroOffset = absoluteEncoderZeroOffset;

                this.moduleTranslation = moduleTranslations[moduleName.ordinal()];
            }
        }

        public static final SwerveModule front_left = new SwerveModule(ModuleName.Front_Left, 2, 3, 3, Steer.front_left_absoluteEncoderZeroOffset, false, false, false, true);
        //^the constants of the front left module
        public static final SwerveModule front_right = new SwerveModule(ModuleName.Front_Right, 4, 5, 0, Steer.front_right_absoluteEncoderZeroOffset, false,false, false, true);
        //^the constants of the front right module
        public static final SwerveModule back_left = new SwerveModule(ModuleName.Back_Left, 6, 7, 1, Steer.back_left_absoluteEncoderZeroOffset, false, false, false, true);
        //^the constants of the back left module
        public static final SwerveModule back_right = new SwerveModule(ModuleName.Back_Right, 8, 9, 2, Steer.back_right_absoluteEncoderZeroOffset, false, false, false, true);
        //^the constants of the back right module

        public static final Map<Integer, String> sparkMaxNames = Map.of(
            front_left.steerMotorID, front_left.moduleName.name(),
            front_right.steerMotorID, front_right.moduleName.name(),
            back_left.steerMotorID, back_left.moduleName.name(),
            back_right.steerMotorID, back_right.moduleName.name()
        );
        
    } 
}
