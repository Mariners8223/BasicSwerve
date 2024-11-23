package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import frc.util.PIDFGains;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ROBOT_TYPE;

public class SwerveModule {

    /**
     * the name of the swerve modules by order
     */
    public enum ModuleName {
        Front_Left,
        Front_Right,
        Back_Left,
        Back_Right
    }

    public static final double MODULE_THREAD_HZ = 50;
    public static final double DISTANCE_BETWEEN_WHEELS = 0.576; // the distance between each wheel in meters
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[]{
            new Translation2d(DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(DISTANCE_BETWEEN_WHEELS / 2, -DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS / 2, -DISTANCE_BETWEEN_WHEELS / 2)};

    private final String moduleName;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private boolean runningDriveCalibration = false;

    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(ModuleName name) {
        this.moduleName = name.toString();

        if (RobotBase.isReal()) {
            io = switch (ROBOT_TYPE) {
                case DEVELOPMENT -> new SwerveModuleIODevBot(name);
                case COMPETITION -> new SwerveModuleIOCompBot(name);
                case REPLAY -> throw new IllegalArgumentException("Robot cannot be replay if it's real");
            };
        } else {
            io = ROBOT_TYPE == Constants.RobotType.REPLAY ? new SwerveModuleIOSIM.SwerveModuleIOReplay() : new SwerveModuleIOSIM();

        }

    }

    public SwerveModulePosition modulePeriodic() {
        io.updateInputs(inputs);

        if (!runningDriveCalibration) {
//            targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
            targetState.speedMetersPerSecond *= Math.cos(targetState.angle.getRadians() - inputs.currentState.angle.getRadians());

            io.setDriveMotorReference(targetState.speedMetersPerSecond);
            io.setSteerMotorReference(targetState.angle.getRotations());
        } else {

            double driveKp = SmartDashboard.getNumber("drive kP", 0);
            double driveKi = SmartDashboard.getNumber("drive kI", 0);
            double driveKd = SmartDashboard.getNumber("drive kD", 0);
            double driveKf = SmartDashboard.getNumber("drive kF", 0);

            io.setDriveMotorPID(new PIDFGains(driveKp, driveKi, driveKd, driveKf, 0, 0));

            double driveReference = SmartDashboard.getNumber("drive setPoint", 0);

            io.setDriveMotorReference(driveReference);

            io.setSteerMotorReference(targetState.angle.getRotations());
        }

        Logger.processInputs("SwerveModule/" + moduleName, inputs);

        return new SwerveModulePosition(inputs.drivePositionMeters, inputs.currentState.angle);
    }

    public SwerveModuleState run(SwerveModuleState targetState) {
        targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
        targetState.speedMetersPerSecond *= inputs.currentState.angle.minus(targetState.angle).getCos();

        this.targetState = targetState;

        return targetState;
    }
    
    public SwerveModuleState getCurrentState() {
        return inputs.currentState;
    }

    public void runDriveCalibration() {
        runningDriveCalibration = true;
    }

    public void stopDriveCalibration() {
        runningDriveCalibration = false;
    }

    public void setIdleMode(boolean isBrakeMode) {
        io.setIdleMode(isBrakeMode);
    }

    public void resetDriveEncoder() {
        io.resetDriveEncoder();
    }
}
