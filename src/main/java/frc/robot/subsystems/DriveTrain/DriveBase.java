// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleConstants;
import frc.util.FastGyros.GyroIO;
import frc.util.FastGyros.NavxIO;
import frc.util.FastGyros.SimGyroIO;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotType;

/**
 * The DriveBase class represents the drivetrain of the robot.
 * It controls the movement and positioning of the robot using swerve drive.
 */
@SuppressWarnings("unused")
public class DriveBase extends SubsystemBase {
    /**
     * the array of the modules themselves
     */
    private final SwerveModule[] modules = new SwerveModule[4];

    /**
     * the kinematics of the swerve drivetrain (how the modules are placed and calculating the kinematics)
     */
    private final SwerveDriveKinematics driveTrainKinematics = new SwerveDriveKinematics(SwerveModule.MODULE_TRANSLATIONS);

    /**
     * a variable representing the deltas of the modules (how much they have moved since the last update)
     */
    private final SwerveModulePosition[] moduleDeltas =
            new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()};

    /**
     * the pose estimator of the drive base (takes the odometry (which is wheel distance and gyro)
     * and calculates the pose of the robot and also can take vision measurements)
     */
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(driveTrainKinematics, new Rotation2d(), moduleDeltas, new Pose2d());

    /**
     * the gyro of the robot. it can be navx or pigeon 2
     */
    private final GyroIO gyro;

    /**
     * the inputs of the drive base (all the motor voltages, angles, speeds, etc.) to be logged
     */
    private final DriveBaseInputsAutoLogged inputs = new DriveBaseInputsAutoLogged();

    /**
     * the max speed the wheels can spin (drive motor at max speed)
     */
    public final double MAX_FREE_WHEEL_SPEED = Constants.ROBOT_TYPE == Constants.RobotType.DEVELOPMENT ?
            SwerveModuleConstants.DEVBOT.MAX_WHEEL_LINEAR_VELOCITY :
            SwerveModuleConstants.COMPBOT.MAX_WHEEL_LINEAR_VELOCITY; //the max speed the wheels can spin when the robot is not moving

    /**
     * the target states of the modules (the states the modules should be in)
     */
    private SwerveModuleState[] targetStates =
            new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};


    /**
     * the current pose of the robot (the position and rotation of the robot)
     */
    private Pose2d currentPose = new Pose2d();


    @AutoLog
    public static class DriveBaseInputs {
        protected double XspeedInput = 0; //the X speed input
        protected double YspeedInput = 0; //the Y speed input

        protected double rotationSpeedInput = 0;

        protected SwerveModuleState[] currentStates =
                new SwerveModuleState[]{new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()}; //the current states of the modules

        protected String activeCommand; //the active command of the robot
    }

    /**
     * Creates a new DriveBase.
     */
    public DriveBase() {
        modules[0] = new SwerveModule(SwerveModule.ModuleName.Front_Left);
        modules[1] = new SwerveModule(SwerveModule.ModuleName.Front_Right);
        modules[2] = new SwerveModule(SwerveModule.ModuleName.Back_Left);
        modules[3] = new SwerveModule(SwerveModule.ModuleName.Back_Right);

        if (RobotBase.isReal()) {
            gyro = new NavxIO(Constants.ROBOT_TYPE == RobotType.DEVELOPMENT);
            gyro.reset(new Pose2d());
        } else gyro = new SimGyroIO(() -> driveTrainKinematics.toTwist2d(moduleDeltas), this::getChassisSpeeds);

        for (int i = 0; i < 4; i++) modules[i].resetDriveEncoder();

        SmartDashboard.putData("Gyro", gyro);

        ReplanningConfig replanConfig =
                new ReplanningConfig(DriveBaseConstants.PathPlanner.PLAN_PATH_TO_STARTING_POINT,
                        DriveBaseConstants.PathPlanner.DYNAMIC_RE_PLANNING,
                        DriveBaseConstants.PathPlanner.PATH_ERROR_TOLERANCE,
                        DriveBaseConstants.PathPlanner.PATH_ERROR_SPIKE_TOLERANCE);
        // ^how pathplanner reacts to position error
        HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                DriveBaseConstants.PathPlanner.XY_PID.createPIDConstants(),
                DriveBaseConstants.PathPlanner.THETA_PID.createPIDConstants(),
                MAX_FREE_WHEEL_SPEED,
                Math.sqrt(Math.pow(SwerveModule.DISTANCE_BETWEEN_WHEELS, 2) * 2) / 2,
                replanConfig);
        //^creates path constraints for pathPlanner

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::reset,
                this::getChassisSpeeds,
                this::drive,
                pathFollowerConfig,
                () -> {
                    if (DriverStation.getAlliance().isPresent())
                        return DriverStation.getAlliance().get() == Alliance.Red;
                    else return false;
                },
                this);
        //^configures the auto builder for pathPlanner

        new Trigger(RobotState::isEnabled).whileTrue(new StartEndCommand(() -> // sets the modules to brake mode when the robot is enabled
                setModulesBrakeMode(true)
                , () ->
        {
            if (!DriverStation.isFMSAttached()) setModulesBrakeMode(false);
        }
        ).ignoringDisable(true));

        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
                this.setDefaultCommand(new DriveCommand(this, RobotContainer.driveController)),
                this::removeDefaultCommand).ignoringDisable(true));
    }


    /**
     * resets the robot to 0, 0 and a rotation of 0 (towards red alliance)
     */
    public Command resetOnlyDirection() {
        return new InstantCommand(() -> {
            if (DriverStation.getAlliance().isPresent()) if (DriverStation.getAlliance().get() == Alliance.Blue)
                currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d());
            else currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d(-Math.PI));
            else currentPose = new Pose2d(currentPose.getX(), currentPose.getY(), new Rotation2d());

            SwerveModulePosition[] positions = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++) positions[i] = modules[i].modulePeriodic();

            poseEstimator.resetPosition(new Rotation2d(), positions, currentPose);

            gyro.reset(currentPose);
        }).withName("Reset Only Direction").ignoringDisable(true);
    }

    public void setModulesBrakeMode(boolean isBrake) {
        for (int i = 0; i < 4; i++) {
            modules[i].setIdleMode(isBrake);
        }
    }

    /**
     * resets the robot to a new pose
     *
     * @param newPose the new pose the robot should be in
     */
    public void reset(Pose2d newPose) {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].modulePeriodic();

        poseEstimator.resetPosition(new Rotation2d(), positions, newPose);

        gyro.reset(newPose);
        currentPose = newPose;
        Logger.processInputs(getName(), inputs);
    }

    /**
     * gets the acceleration of the robot in the X direction
     *
     * @return the acceleration of the robot in the X direction m/s^2
     */
    public double getXAcceleration() {
        return gyro.getAccelerationX();
    }

    /**
     * gets the acceleration of the robot in the Y direction
     *
     * @return the acceleration of the robot in the Y direction m/s^2
     */
    public double getYAcceleration() {
        return gyro.getAccelerationY();
    }

    /**
     * returns the reported Rotation by the Navx
     *
     * @return Rotation2d reported by the Navx
     */
    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    /**
     * returns the current angle of the robot
     *
     * @return the angle of the robot (left is positive) IN DEGREES
     */
    public double getAngle() {
        return gyro.getAngleDegrees();
    }

    /**
     * gets the current chassisSpeeds of the robot
     *
     * @return the current chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return driveTrainKinematics.toChassisSpeeds(inputs.currentStates);
    }

    /**
     * gets the current absolute (field relative) speeds of the robot
     *
     * @return the current chassis speeds
     */
    public ChassisSpeeds getAbsoluteChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation2d());
    }

    /**
     * gets the current pose of the robot
     *
     * @return the current pose of the robot
     */
    public Pose2d getPose() {
        return currentPose;
    }

    /**
     * updates pose Estimator with vision measurements
     *
     * @param visionPose the pose of the robot from vision
     * @param timeStamp  the time stamp of the vision measurement
     */
    public void addVisionMeasurement(Pose2d visionPose, double timeStamp) {
        poseEstimator.addVisionMeasurement(visionPose, timeStamp);
    }

    /**
     * drives the robot relative to itself
     *
     * @param Xspeed        the X speed of the robot (forward is positive) m/s
     * @param Yspeed        the Y speed of the robot (left is positive) m/s
     * @param rotationSpeed the rotation of the robot (left is positive) rad/s
     */
    public void drive(double Xspeed, double Yspeed, double rotationSpeed, Translation2d centerOfRotation) {
        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(Xspeed, Yspeed, rotationSpeed, getRotation2d());

        targetStates = driveTrainKinematics.toSwerveModuleStates(fieldRelativeSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, MAX_FREE_WHEEL_SPEED);

        for (int i = 0; i < 4; i++) {
            targetStates[i] = modules[i].run(targetStates[i]);
        }

        inputs.XspeedInput = Xspeed;
        inputs.YspeedInput = Yspeed;
        inputs.rotationSpeedInput = rotationSpeed;
        Logger.processInputs(getName(), inputs);
    }

    /**
     * drives the robot relative to itself
     *
     * @param Xspeed        the X speed of the robot (forward is positive) m/s
     * @param Yspeed        the Y speed of the robot (left is positive) m/s
     * @param rotationSpeed the rotation of the robot (left is positive) rad/s
     */
    public void robotRelativeDrive(double Xspeed, double Yspeed, double rotationSpeed) {

        targetStates = driveTrainKinematics.toSwerveModuleStates(new ChassisSpeeds(Xspeed, Yspeed, rotationSpeed));
        SwerveDriveKinematics.desaturateWheelSpeeds(inputs.currentStates, MAX_FREE_WHEEL_SPEED);

        for (int i = 0; i < 4; i++) {
            targetStates[i] = modules[i].run(targetStates[i]);
        }

        inputs.XspeedInput = Xspeed;
        inputs.YspeedInput = Yspeed;
        inputs.rotationSpeedInput = rotationSpeed;
        Logger.processInputs(getName(), inputs);
    }

    /**
     * drives the robot without built in pid fixes
     *
     * @param chassisSpeeds the chassis speeds of the target
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        targetStates = driveTrainKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, MAX_FREE_WHEEL_SPEED);

        for (int i = 0; i < 4; i++) {
            targetStates[i] = modules[i].run(targetStates[i]);
        }

        inputs.XspeedInput = chassisSpeeds.vxMetersPerSecond;
        inputs.YspeedInput = chassisSpeeds.vyMetersPerSecond;
        inputs.rotationSpeedInput = chassisSpeeds.omegaRadiansPerSecond;
        Logger.processInputs(getName(), inputs);
    }

    public Command runModuleDriveCalibration() {
        return new InstantCommand(() -> {
            for (int i = 0; i < 4; i++) {
                modules[i].runDriveCalibration();
            }

            SmartDashboard.putNumber("drive P", 0);
            SmartDashboard.putNumber("drive I", 0);
            SmartDashboard.putNumber("drive D", 0);
            SmartDashboard.putNumber("drive setPoint", 0);

            SmartDashboard.putNumber("drive kS", 0);
        }).withName("Run Module Drive Calibration").ignoringDisable(true);
    }

    public Command stopModuleDriveCalibration() {
        return new InstantCommand(() -> {
            for (int i = 0; i < 4; i++) {
                modules[i].stopDriveCalibration();
            }
        }).withName("Stop Module Drive Calibration").ignoringDisable(true);
    }

    /**
     * path finds a path from the current pose to the target pose
     *
     * @param targetPose    the target pose
     * @param endVelocity   the velocity the robot should be in when it reaches the end of the path in m/s
     * @param rotationDelay the delay in meters before the robot starts rotation
     * @return a command that follows a path to the target pose
     */
    public Command findPath(Pose2d targetPose, double endVelocity, double rotationDelay) {
        return AutoBuilder.pathfindToPose(targetPose, DriveBaseConstants.PathPlanner.PATH_CONSTRAINTS, endVelocity, rotationDelay);
    }

    /**
     * path finds a path from the current pose to the target pose
     *
     * @param targetPose  the target pose
     * @param endVelocity the velocity the robot should be in when it reaches the end of the path in m/s
     * @return a command that follows a path to the target pose
     */
    public Command findPath(Pose2d targetPose, double endVelocity) {
        return AutoBuilder.pathfindToPose(targetPose, DriveBaseConstants.PathPlanner.PATH_CONSTRAINTS, endVelocity);
    }

    /**
     * path finds a path from the current pose to the target pose
     *
     * @param targetPose the target pose
     * @return a command that follows a path to the target pose
     */
    public Command findPath(Pose2d targetPose) {
        return AutoBuilder.pathfindToPose(targetPose, DriveBaseConstants.PathPlanner.PATH_CONSTRAINTS);
    }

    /**
     * path finds to a given path then follows that path
     *
     * @param targetPath the path to path finds to and follow
     * @return a command that path finds to a given path then follows that path
     */
    public Command pathFindToPathAndFollow(PathPlannerPath targetPath) {
        return AutoBuilder.pathfindThenFollowPath(targetPath, DriveBaseConstants.PathPlanner.PATH_CONSTRAINTS);
    }

    /**
     * path finds to a given path then follows that path
     *
     * @param targetPath    the path to path finds to and follow
     * @param rotationDelay the delay in meters in which the robot does not change it's holonomic angle
     * @return a command that path finds to a given path then follows that path
     */
    public Command pathFindToPathAndFollow(PathPlannerPath targetPath, double rotationDelay) {
        return AutoBuilder.pathfindThenFollowPath(targetPath, DriveBaseConstants.PathPlanner.PATH_CONSTRAINTS, rotationDelay);
    }

    SwerveModulePosition[] previousPositions = new SwerveModulePosition[]{
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()};

    SwerveModulePosition[] positions = new SwerveModulePosition[]{
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()};

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            inputs.currentStates[i] = modules[i].getCurrentState();
            positions[i] = modules[i].modulePeriodic();

            moduleDeltas[i] = new SwerveModulePosition(
                    positions[i].distanceMeters - previousPositions[i].distanceMeters,
                    positions[i].angle
            );

            previousPositions[i] = positions[i].copy();
        }

        gyro.update();
        poseEstimator.updateWithTime(Logger.getTimestamp(), gyro.getRotation2d(), positions);
        currentPose = poseEstimator.getEstimatedPosition();

        Logger.recordOutput("DriveBase/estimatedPose", currentPose);
        Logger.recordOutput("DriveBase/ChassisSpeeds", getChassisSpeeds());
        Logger.recordOutput("DriveBase/targetStates", targetStates);

        RobotContainer.field.setRobotPose(currentPose);

        inputs.activeCommand = this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None";

        Logger.processInputs(getName(), inputs);
    }
}
