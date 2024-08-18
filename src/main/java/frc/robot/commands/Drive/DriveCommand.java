package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.DriveTrain.DriveBase;

import static frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModule.distanceBetweenWheels;


public class DriveCommand extends Command {

    private final DriveBase driveBase;
    private final CommandPS5Controller controller;

    private static final Translation2d[] possibleCenterOfRotations = {
            new Translation2d(distanceBetweenWheels / 2, 0),
            new Translation2d(distanceBetweenWheels / 2, -distanceBetweenWheels / 2),
            new Translation2d(0, -distanceBetweenWheels / 2),
            new Translation2d(-distanceBetweenWheels / 2, -distanceBetweenWheels / 2),
            new Translation2d(-distanceBetweenWheels / 2, 0),
            new Translation2d(-distanceBetweenWheels, distanceBetweenWheels / 2),
            new Translation2d(0, distanceBetweenWheels / 2),
            new Translation2d(distanceBetweenWheels / 2, distanceBetweenWheels / 2)
    };

    public DriveCommand(DriveBase driveBase, CommandPS5Controller controller) {
        this.driveBase = driveBase;
        this.controller = controller;
        addRequirements(this.driveBase);
        setName("DriveCommand");
    }

    @Override
    public void initialize() {
        driveBase.drive(0, 0, 0, new Translation2d());
    }

    private static double deadBand(double value) {
        return Math.abs(value) > 0.1 ? value : 0;
    }

    
    int currentAngleFieldRelative = -1; //the pov itself angle
    int currentAngleRobotRelative = -1; //the selected angle on the robot axis
    int previousAngleFieldRelative = -1;
    int previousAngleRobotRelative = -1;
    int timer = 0;

    private Translation2d calculateCenterOfRotation(int angle) {
        if (angle != -1) {
            if (angle == currentAngleFieldRelative) {
                if (angle != previousAngleFieldRelative) {
                    timer++;
                    if (timer >= 25) {
                        timer = 0;
                        previousAngleFieldRelative = currentAngleFieldRelative;
                        previousAngleRobotRelative = currentAngleRobotRelative;
                    }
                }
            } else {
                if (angle == previousAngleFieldRelative) {
                    currentAngleRobotRelative = previousAngleRobotRelative;
                    currentAngleFieldRelative = previousAngleFieldRelative;
                } else {
                    currentAngleFieldRelative = angle;
                    currentAngleRobotRelative = convertPOVToRobotRelative(angle);
                }
                timer = 0;
            }
        } else {
            if (timer == 0) {
                currentAngleFieldRelative = -1;
                currentAngleRobotRelative = -1;
            } else if (timer >= 25) {
                previousAngleFieldRelative = -1;
                previousAngleRobotRelative = -1;
                timer = 0;
            }
            timer++;
        }
        return findCenterOfRotation(currentAngleRobotRelative);
    }

    private Translation2d findCenterOfRotation(int angle) {
        if (angle == -1) {
            return new Translation2d();
        } else {
            return possibleCenterOfRotations[(int) (angle / 45) % 8];
        }
    }

    /**
     * Converts the POV angle to a robot relative angle
     * <p>
     * function is:
     * f(angle) = (Math.round((360 - angle + inputModules(theta, 0, 360)) / 45) * 45)
     *
     * @param angle the pov angle
     * @return the robot relative angle snapped to nearest 45 degrees
     */
    private int convertPOVToRobotRelative(double angle) {
        double theta = MathUtil.inputModulus(driveBase.getAngle(), 0, 360);
        double alpha = 360 - angle;
        double beta = alpha + theta;

        beta = Math.round(beta / 45) * 45;

        return (int) beta;
    }


    @Override
    public void execute() {
        //calculates a value from 1 to the max wheel speed based on the R2 axis
        double R2Axis = (1 - (0.5 + controller.getR2Axis() / 2)) * (driveBase.maxFreeWheelSpeed - 1) + 1;

        double povAngle = controller.getHID().getPOV();

        Translation2d centerOfRotation = calculateCenterOfRotation((int) povAngle);

        //sets the value of the 3 axis we need (accounting for drift)
        double leftX = -deadBand(controller.getLeftX());
        double leftY = -deadBand(controller.getLeftY());
        double rightX = deadBand(controller.getRightX());

        //drives the robot with the values
        driveBase.drive(
                leftX * R2Axis,
                leftY * R2Axis,
                rightX * R2Axis,
                centerOfRotation
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0, 0, 0, new Translation2d());
    }
}
