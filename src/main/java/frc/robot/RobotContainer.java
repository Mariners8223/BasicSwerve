// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveTrain.DriveBase;

public class RobotContainer{
    public static DriveBase driveBase;
    public static CommandPS5Controller driveController;

    public static Field2d field;
    public static LoggedDashboardChooser<Command> autoChooser;


    public RobotContainer()
    {
        driveController = new CommandPS5Controller(0);
        driveBase = new DriveBase();

        configureBindings();

        field = new Field2d();

        SmartDashboard.putData(field);

        configChooser();
    }

    private static final BooleanSupplier checkForPathChoiseUpdate = new BooleanSupplier() {
        private String lastAutoName = "InstantCommand"; 
        @Override
        public boolean getAsBoolean() {
            if(autoChooser.get() == null) return false;

            String currentAutoName = autoChooser.get().getName();

            try{ 
                return lastAutoName != currentAutoName;
            }
            finally{
                lastAutoName = currentAutoName;
            }
            
        };
    };

    private void configChooser(){
        List<String> namesOfAutos = AutoBuilder.getAllAutoNames();
        List<PathPlannerAuto> autosOfAutos = new ArrayList<>();

        autoChooser = new LoggedDashboardChooser<>("chooser");
        for (String autoName : namesOfAutos) {
            PathPlannerAuto auto = new PathPlannerAuto(autoName);
            autosOfAutos.add(auto);
        }

        autosOfAutos.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        SmartDashboard.putData("chooser", autoChooser.getSendableChooser());

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop).onTrue(new InstantCommand(() -> field.getObject("AutoPath").setPoses()).ignoringDisable(true));
        new Trigger(RobotState::isDisabled).and(checkForPathChoiseUpdate).onTrue(new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));
    }

    private static void updateFieldFromAuto(String autoName){
        List<Pose2d> poses = new ArrayList<>();
        boolean DoesExsit = false;
        for (String name : AutoBuilder.getAllAutoNames()) {
            if(name.equals(autoName)) DoesExsit = true;
        }
        if(!DoesExsit){
            field.getObject("AutoPath").setPoses();
            return;
        }
        PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) path = path.flipPath();
            path.getPathPoses().forEach(pose -> poses.add(pose));
        });
        field.getObject("AutoPath").setPoses(poses);
    }
    
    
    private void configureBindings() {
        driveController.options().onTrue(new InstantCommand(() -> driveBase.resetOnlyDirection()));
        driveController.cross().onTrue(new InstantCommand(() -> driveBase.runModuleDriveCalibration()));
        driveController.triangle().onTrue(new InstantCommand(() -> driveBase.stopModuleDriveCalibration()));
    }
    
    
    
    public static Command getAutoCommand()
    {
        return autoChooser.get();
    }
}
