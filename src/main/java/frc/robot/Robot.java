// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule;
import frc.util.LocalADStarAK;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;
    
    String lastAutoName = "InstantCommand";
    
    
    @Override
    public void robotInit() {
        new RobotContainer();

        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
        Logger.registerURCL(URCL.startExternal(Constants.DriveTrain.sparkMaxNames));

        if(isReal()){
            Logger.addDataReceiver(new WPILOGWriter("/U/logs/AdvantageKit"));
            if(Constants.robotType == RobotType.DEVELOPMENT) Logger.addDataReceiver(new NT4Publisher());
            Logger.addDataReceiver(new NT4Publisher());

            DataLogManager.start("U/logs/dataLogManager");
            SignalLogger.setPath("U/logs/signalLogger");
            SignalLogger.enableAutoLogging(true);
        }
        else{
            if(Constants.robotType == Constants.RobotType.REPLAY){
                String logPath = LogFileUtil.findReplayLog();

                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                Logger.setReplaySource(new WPILOGReader(logPath));
            }
            // Logger.addDataReceiver(new WPILOGWriter("F:/downloads"));
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start();

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((path) ->
                Logger.recordOutput("PathPlanner/ActivePath", path.toArray(new Pose2d[0])));

        PathPlannerLogging.setLogTargetPoseCallback((targetPose) ->
                Logger.recordOutput("PathPlanner/TargetPose", targetPose));

        PathfindingCommand.warmupCommand().schedule();

        Notifier notifier = RobotContainer.driveBase.getNotifier();
        notifier.setName("DriveBaseNotifier");
        System.out.println("Starting DriveBase Notifier");
        notifier.startPeriodic(1 / SwerveModule.moduleThreadHz);
    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }
    
    
    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {
        // if(RobotContainer.getAutoCommand() != null && RobotContainer.getAutoCommand().getName() != lastAutoName){
        //     lastAutoName = RobotContainer.getAutoCommand().getName();
        //     RobotContainer.updateFieldFromAuto(lastAutoName);
        // }
    }
    
    
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
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
