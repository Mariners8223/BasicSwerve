// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.CalibrateLimitSwitch;
import frc.robot.commands.Arm.MoveAlpha;
import frc.robot.commands.Arm.MoveArmToPosition;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Arm.ArmConstants.ArmPosition;
import frc.robot.subsystems.DriveTrain.DriveBase;

public class RobotContainer{
    public static DriveBase driveBase;
    public static Arm arm;
    public static CommandPS5Controller driveController;
    public static CommandPS5Controller armController;

    public static Field2d field;
    public static LoggedDashboardChooser<Command> autoChooser;


    public RobotContainer()
    {
        driveController = new CommandPS5Controller(0);
        armController = new CommandPS5Controller(1);
        driveBase = new DriveBase();
        arm = new Arm();

        configureBindings();
        // configureArmBindings();
        calibration();

        field = new Field2d();

        SmartDashboard.putData(field);

        configChooser();
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
        catch (RuntimeException ignored){
        }

        field.getObject("AutoPath").setPoses(poses);
    }
    
    
    private void configureBindings() {
        driveController.options().onTrue(driveBase.resetOnlyDirection());
        driveController.cross().onTrue(driveBase.runModuleDriveCalibration());
        driveController.triangle().onTrue(driveBase.stopModuleDriveCalibration());
    }

    private static void configureArmBindings() {
        Command moveToHome = MoveArmToPosition.getCommand(arm, ArmPosition.HOME_POSITION);

        armController.cross().whileTrue(MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.COLLECT_FLOOR_POSITION))
                .whileFalse(moveToHome);

        armController.circle().whileTrue(MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.COLLECT_SOURCE_POSITION))
                .whileFalse(moveToHome);

        armController.triangle().whileTrue(MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.AMP_POSITION))
                .whileFalse(moveToHome);

        armController.circle().whileTrue(MoveArmToPosition.getCommand(arm, ArmConstants.ArmPosition.FREE_POSITION))
                .whileFalse(moveToHome); 
    }

    private void calibration(){
        armController.touchpad().onTrue(CalibrateLimitSwitch.getCommand(arm));

        armController.options().onTrue(new MoveAlpha(arm, ArmConstants.ArmPosition.FREE_POSITION.getAlpha()));
    }
    
    
    
    public static Command getAutoCommand()
    {
        return autoChooser.get();
    }
}
