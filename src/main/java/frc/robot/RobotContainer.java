// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import org.json.simple.parser.ParseException;
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
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControllerLocation;

public class RobotContainer{
    public static DriveBase driveBase;
    public static CommandPS5Controller driveController;

    public static Field2d field;
    public static LoggedDashboardChooser<Command> autoChooser;

    public static MarinersController motor;


    public RobotContainer()
    {
        driveController = new CommandPS5Controller(0);
        driveBase = new DriveBase();

        configureBindings();

        field = new Field2d();

        SmartDashboard.putData(field);

        configChooser();


        motor = new MarinersTalonFX("steer", ControllerLocation.MOTOR, 2);

        motor.setMotorIdleMode(false);

        motor.setMaxMinOutput(6, -6);

        SmartDashboard.putNumber("value", 0);

    }

    private static final BooleanSupplier checkForPathChoiceUpdate = new BooleanSupplier() {
        private String lastAutoName = "InstantCommand"; 
        @Override
        public boolean getAsBoolean() {
            if(autoChooser.get() == null) return false;

            String currentAutoName = autoChooser.get().getName();

            try{ 
                return !Objects.equals(lastAutoName, currentAutoName);
            }
            finally{
                lastAutoName = currentAutoName;
            }
            
        }
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
        new Trigger(RobotState::isDisabled).and(checkForPathChoiceUpdate).onTrue(new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));
    }

    private static void updateFieldFromAuto(String autoName){
        List<Pose2d> poses = new ArrayList<>();

        try {
            boolean invert =
                    DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
                path = invert ? path.flipPath() : path;

                poses.addAll(path.getPathPoses());
            });
        }
        catch (IOException | ParseException e){
            DriverStation.reportError("Error loading auto path", e.getStackTrace());
        }   

        field.getObject("AutoPath").setPoses(poses);
    }
    
    
    private void configureBindings() {
        // driveController.options().onTrue(driveBase.resetOnlyDirection());
        Supplier<Rotation2d> controllerAngle = () -> new Rotation2d(-RobotContainer.driveController.getRawAxis(1), -RobotContainer.driveController.getRawAxis(0));

        driveController.cross().whileTrue(driveBase.runSysIDQuasistatic(false, controllerAngle));
        driveController.square().whileTrue(driveBase.runSysIDQuasistatic(true, controllerAngle));

        driveController.circle().whileTrue(driveBase.runSysIDDynamic(false, controllerAngle));
        driveController.triangle().whileTrue(driveBase.runSysIDDynamic(true, controllerAngle));
    }
    
    
    
    public static Command getAutoCommand()
    {
        return autoChooser.get();
    }
}
