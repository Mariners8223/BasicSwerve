// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.util.LocalADStarAK;
import frc.util.MarinersController.*;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersController.ControllerLocation;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot
{
    private Command autonomousCommand;    

    private MarinersController motor;
    
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

        PIDFGains gains = new PIDFGains(0.7, 0, 0, 0.7, 0, 0);

        TrapezoidProfile.Constraints constraints = new Constraints(3, 3);

        motor = new MarinersTalonFX("steer", ControllerLocation.MOTOR, 2, gains, 5.14);
//        motor = new MarinersSparkBase("steer", ControllerLocation.RIO, 4, true,
//                MarinersSparkBase.MotorType.SPARK_MAX, gains, 12.8);

        motor.setMaxMinOutput(3, -3);

//        CANcoder canCoder =  new CANcoder(3);
//
//        canCoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf));
//
//
//        canCoder.setPosition(canCoder.getAbsolutePosition().getValueAsDouble());
//
//        canCoder.getPosition().setUpdateFrequency(100);
//        canCoder.getVelocity().setUpdateFrequency(100);
//
//        motor.setMeasurements(new MarinersMeasurements(
//                () -> canCoder.getPosition().getValueAsDouble(),
//                () -> canCoder.getVelocity().getValueAsDouble(),
//                1
//        ));
//
//        motor.enablePositionWrapping(-0.5, 0.5);

        motor.setProfile(constraints);

        motor.resetMotorEncoder();
    }
    
    
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

        double value = SmartDashboard.getNumber("value", 0);

        motor.setReference(value, ControlMode.ProfiledVelocity);
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
