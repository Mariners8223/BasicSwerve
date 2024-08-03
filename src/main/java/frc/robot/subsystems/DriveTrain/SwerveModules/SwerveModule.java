package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.robotType;

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

    public static final double moduleThreadHz = 50;
    public static final double distanceBetweenWheels = 0.576; // the distance between each wheel in meters
    public static final Translation2d[] moduleTranslations = new Translation2d[]
            {new Translation2d(distanceBetweenWheels / 2, distanceBetweenWheels / 2), new Translation2d(distanceBetweenWheels / 2, -distanceBetweenWheels / 2),
                    new Translation2d(-distanceBetweenWheels / 2, distanceBetweenWheels / 2), new Translation2d(-distanceBetweenWheels / 2, -distanceBetweenWheels / 2)};

    private final String moduleName;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private final ProfiledPIDController drivePIDController;
    public final SimpleMotorFeedforward driveFeedforward;
    private final PIDController steerPIDController;

    private boolean isRunningSysID = false;

    private SwerveModuleState targetState = new SwerveModuleState();

    private double driveMotorVoltageOutput = 0;
    private double steerMotorVoltageOutput = 0;

    private final Runnable driveMotorSetRunnable = new Runnable() {
        @Override
        public void run() {
            io.setDriveMotorVoltage(driveMotorVoltageOutput);
        }
    };

    private final Runnable steerMotorSetRunnable = new Runnable() {
        @Override
        public void run() {
            io.setSteerMotorVoltage(steerMotorVoltageOutput);
        }
    };

    public SwerveModule(ModuleName name) {
        this.moduleName = name.toString();
        SwerveModuleConstants constants =
                robotType == Constants.RobotType.DEVELOPMENT ? SwerveModuleConstants.DEVBOT : SwerveModuleConstants.COMPBOT;

        drivePIDController = constants.driveMotorPID.createProfiledPIDController();
        driveFeedforward = new SimpleMotorFeedforward(0, constants.driveMotorPID.getF());
        steerPIDController = constants.steerMotorPID.createPIDController();

        if (RobotBase.isReal()) {
            io = switch (robotType) {
                case DEVELOPMENT -> new SwerveModuleIODevBot(name);
                case COMPETITION -> new SwerveModuleIOCompBot(name);
                case REPLAY -> throw new IllegalArgumentException("Robot cannot be replay if it's real");
            };
        } else {
            io = robotType == Constants.RobotType.REPLAY ? new SwerveModuleIOSIM.SwerveModuleIOReplay() : new SwerveModuleIOSIM();

            SmartDashboard.putData(drivePIDController);
            SmartDashboard.putData(steerPIDController);

        }

    }

    public SwerveModulePosition modulePeriodic() {
        io.updateInputs(inputs);

        if (!isRunningSysID) {
            targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
            targetState.speedMetersPerSecond *= Math.cos(targetState.angle.getRadians() - inputs.currentState.angle.getRadians());

            driveMotorVoltageOutput = drivePIDController.calculate(inputs.currentState.speedMetersPerSecond, targetState.speedMetersPerSecond) + driveFeedforward.calculate(targetState.speedMetersPerSecond);
            steerMotorVoltageOutput = steerPIDController.calculate(inputs.currentState.angle.getRadians(), targetState.angle.getRadians());
            steerMotorVoltageOutput = steerPIDController.atSetpoint() ? 0 : steerMotorVoltageOutput;

            driveMotorSetRunnable.run();
            steerMotorSetRunnable.run();
        } else {
            if (targetState != null) {
                double steerOutPut = steerPIDController.calculate(inputs.currentState.angle.getRadians(), targetState.angle.getRadians());

                io.setSteerMotorVoltage(steerPIDController.atSetpoint() ? 0 : steerOutPut);
            }
        }
        Logger.processInputs("SwerveModule/" + moduleName, inputs);

        return new SwerveModulePosition(inputs.drivePositionMeters, inputs.currentState.angle);
    }

    public SwerveModuleState getCurrentState() {
        return inputs.currentState;
    }

    public SwerveModuleState run(SwerveModuleState targetState) {
        isRunningSysID = false;
        targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
        targetState.speedMetersPerSecond *= inputs.currentState.angle.minus(targetState.angle).getCos();

        this.targetState = targetState;

        return targetState;
    }

    public void runSysIDSteer(Measure<Voltage> steerVoltage) {
        isRunningSysID = true;
        io.setSteerMotorVoltage(steerVoltage.in(Volts));
        targetState = null;
    }

    public void runSysIDDrive(Measure<Voltage> driveVoltage, Rotation2d angle) {
        isRunningSysID = true;
        io.setDriveMotorVoltage(driveVoltage.in(Volts));
        targetState.angle = angle;
    }

    public void setIdleMode(boolean isBrakeMode) {
        io.setIdleMode(isBrakeMode);
    }

    public void resetDriveEncoder() {
        io.resetDriveEncoder();
    }
}
