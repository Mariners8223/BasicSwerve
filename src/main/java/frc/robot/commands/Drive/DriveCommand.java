package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.DriveBase;

public class DriveCommand extends Command {

    private final DriveBase driveBase;
    private final GenericHID controller;

    private final int LEFT_X_AXIS = 1;
    private final int LEFT_Y_AXIS = 0;
    private final int RIGHT_X_AXIS = 2;
    private final int R2_AXIS = 4;

    public DriveCommand(DriveBase driveBase, GenericHID controller) {
        this.driveBase = driveBase;
        this.controller = controller;
        addRequirements(this.driveBase);
        setName("DriveCommand");
    }

    @Override
    public void initialize() {
        driveBase.drive(0, 0, 0);
    }


    @Override
    public void execute() {
        //calculates a value from 1 to the max wheel speed based on the R2 axis
        double R2Axis = (1 - (0.5 + controller.getRawAxis(R2_AXIS) / 2)) * (driveBase.maxFreeWheelSpeed - 1) + 1;

        //sets the value of the 3 axis we need (accounting for drift)
        double leftX = Math.abs(controller.getRawAxis(LEFT_X_AXIS)) > 0.1 ? controller.getRawAxis(LEFT_X_AXIS) : 0;
        double leftY = Math.abs(controller.getRawAxis(LEFT_Y_AXIS)) > 0.1 ? controller.getRawAxis(LEFT_Y_AXIS) : 0;
        double rightX = Math.abs(controller.getRawAxis(RIGHT_X_AXIS)) > 0.1 ? -controller.getRawAxis(RIGHT_X_AXIS) : 0;

        //drives the robot with the values
        driveBase.drive(
                leftX * R2Axis,
                leftY * R2Axis,
                rightX * R2Axis
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0, 0, 0);
    }
}
