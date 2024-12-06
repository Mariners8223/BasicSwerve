// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.util.BatteryVoltageLow;
import frc.util.LocalADStarAK;
import frc.util.MarinersController.*;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;    
    
    @Override
    public void robotInit() {
        new RobotContainer();

        Logger.recordMetadata("Robot Type", Constants.ROBOT_TYPE.name());

        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            //noinspection DataFlowIssue
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        if(isReal()){
            // Logger.addDataReceiver(new WPILOGWriter("/U/logs/AdvantageKit"));
            Logger.addDataReceiver(new NT4Publisher());
            switch (Constants.ROBOT_TYPE) {
                case DEVELOPMENT -> Logger.addDataReceiver(new WPILOGWriter("/media/logs"));
                case COMPETITION -> Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
                case REPLAY -> throw new IllegalArgumentException("Robot cannot be replay if it's real");
            }

        }
        else{
            if(Constants.ROBOT_TYPE == Constants.RobotType.REPLAY){
                String logPath = LogFileUtil.findReplayLog();
                ControllerMaster.getInstance().stopLoop();

                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                Logger.setReplaySource(new WPILOGReader(logPath));
            }
            Logger.addDataReceiver(new NT4Publisher());
        }

        SignalLogger.enableAutoLogging(false);
        DataLogManager.stop();

        Logger.start();

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((path) ->
                Logger.recordOutput("PathPlanner/ActivePath", path.toArray(new Pose2d[0])));

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
                Logger.recordOutput("PathPlanner/TargetPose", targetPose));

        PathfindingCommand.warmupCommand().schedule();


        ControllerMaster.getInstance();


        if(RobotController.getBatteryVoltage() <= 12){
            throw new BatteryVoltageLow("battery voltage to low, please replace batter");
        }
    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

        // SmartDashboard.putNumber("gryooo", Rotation2d.fromRadians(Math.atan(-RobotContainer.driveController.getRawAxis(1) / -RobotContainer.driveController.getRawAxis(0))).getDegrees());
        // SmartDashboard.putNumber("gryooo", new Rotation2d(-RobotContainer.driveController.getRawAxis(1), -RobotContainer.driveController.getRawAxis(0)).getDegrees());

    }
    
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {
    }
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void disabledExit() {}
    
    
    @Override
    public void autonomousInit()
    {
        autonomousCommand = RobotContainer.getAutoCommand();
        
        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    @SuppressWarnings("RedundantMethodOverride")
    @Override
    public void testExit() {}
}
