package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.util.controllers.CommandController;


public class DriveCommand extends Command {
    private final DriveBase driveBase;
    private final CommandController controller;

    public DriveCommand(DriveBase driveBase, CommandController controller) {
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
        //calculates a value from 0 to the max wheel speed based on the R2 axis
        double R2Axis = 1 + (driveBase.maxFreeWheelSpeed - 1) * (1 - (0.5 + controller.getR2Axis() / 2));

        //sets the value of the 3 axis we need (accounting for drift)
        double leftX = Math.abs(controller.getLeftX()) > 0.1 ? -controller.getLeftX() : 0;
        double leftY = Math.abs(controller.getLeftY()) > 0.1 ? -controller.getLeftY() : 0;
        double rightX = Math.abs(controller.getRightX()) > 0.1 ? -controller.getRightX() : 0;

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
