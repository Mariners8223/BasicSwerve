package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class DriveBaseSYSID {
    private final SysIdRoutine steerMotorsRoutine;
    private final SysIdRoutine driveMotorsRoutine;

    private final SysIdRoutine ThetaRoutine;

    public DriveBaseSYSID(DriveBase driveBase, CommandPS5Controller controller) {

        Supplier<Rotation2d> controllerAngle = () -> new Rotation2d(-controller.getRawAxis(1), -controller.getRawAxis(0));

        steerMotorsRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, //set if needed
                        null, //set if needed
                        null, //set if needed
                        (state) -> Logger.recordOutput("SysIDStates/Steer", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        driveBase::runSYSIDSteer,
                        null, //should be null (use advantage kit)
                        driveBase
                ));

        SwerveModuleState[] driveRoutineStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            driveRoutineStates[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        }

        driveMotorsRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, //set if needed
                        null, //set if needed
                        null, //set if needed
                        (state) -> Logger.recordOutput("SysIDStates/Drive", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            Rotation2d angle = controllerAngle.get();

                            for (int i = 0; i < 4; i++) {
                                driveRoutineStates[i].angle = angle;
                                driveRoutineStates[i].speedMetersPerSecond = voltage.baseUnitMagnitude();
                            }

                            driveBase.driveSysID(driveRoutineStates);
                        },
                        null, //should be null (use advantage kit)
                        driveBase
                ));

        SwerveModuleState[] ThetaRoutineStates = new SwerveModuleState[4];

        ThetaRoutineStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(135)); //front left
        ThetaRoutineStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(45)); //front right
        ThetaRoutineStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(225)); //back left
        ThetaRoutineStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(315)); //back right

        ThetaRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, //set if needed
                        null, //set if needed
                        null, //set if needed
                        (state) -> Logger.recordOutput("SysIDStates/Theta", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            for (int i = 0; i < 4; i++) {
                                ThetaRoutineStates[i].speedMetersPerSecond = voltage.baseUnitMagnitude();
                            }

                            driveBase.driveSysID(ThetaRoutineStates);
                        },
                        null, //should be null (use advantage kit)
                        driveBase
                ));
    }

    /**
     * Get the steer motors routine for the given direction
     * @param direction the direction to get the routine for
     * @return the command to run the routine
     */
    public Command getSteerMotorsRoutineDynamic(SysIdRoutine.Direction direction) {
        return steerMotorsRoutine.dynamic(direction);
    }

    /**
     * Get the drive motors routine for the given direction
     * the routine will run the drive motors at the given voltage and steer the robot with the controller (field oriented)
     * @param direction the direction to get the routine for
     * @return the command to run the routine
     */
    public Command getDriveMotorsRoutineDynamic(SysIdRoutine.Direction direction) {
        return driveMotorsRoutine.dynamic(direction);
    }

    /**
     * Get the theta routine for the given direction
     * the routine will run the robot in the theta direction with the given voltage (positive is counter-clockwise)
     * @param direction the direction to get the routine for
     * @return the command to run the routine
     */
    public Command getThetaRoutineDynamic(SysIdRoutine.Direction direction) {
        return ThetaRoutine.dynamic(direction);
    }

    /**
     * Get the steer motors routine for the given direction
     * the routine will run the steer motors at a quasistatic voltage
     * @param direction the direction to get the routine for
     * @return the command to run the routine
     */
    public Command getSteerMotorsRoutineQuasistatic(SysIdRoutine.Direction direction) {
        return steerMotorsRoutine.quasistatic(direction);
    }

    /**
     * Get the drive motors routine for the given direction
     * the routine will run the drive motors at a quasistatic voltage and steer the robot with the controller (field oriented)
     * @param direction the direction to get the routine for
     * @return the command to run the routine
     */
    public Command getDriveMotorsRoutineQuasistatic(SysIdRoutine.Direction direction) {
        return driveMotorsRoutine.quasistatic(direction);
    }

    /**
     * Get the theta routine for the given direction
     * the routine will run the robot in the theta direction at a quasistatic voltage (positive is counter-clockwise)
     * @param direction the direction to get the routine for
     * @return the command to run the routine
     */
    public Command getThetaRoutineQuasistatic(SysIdRoutine.Direction direction) {
        return ThetaRoutine.quasistatic(direction);
    }
}
