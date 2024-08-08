package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableRegistry;
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
    public static final Translation2d[] moduleTranslations = new Translation2d[]{
            new Translation2d(distanceBetweenWheels / 2, distanceBetweenWheels / 2),
            new Translation2d(distanceBetweenWheels / 2, -distanceBetweenWheels / 2),
            new Translation2d(-distanceBetweenWheels / 2, distanceBetweenWheels / 2),
            new Translation2d(-distanceBetweenWheels / 2, -distanceBetweenWheels / 2)};

    private final String moduleName;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private final ProfiledPIDController drivePIDController;
    public SimpleMotorFeedforward driveFeedforward;
    private final PIDController steerPIDController;

    private boolean runningCalibration = false;
    private boolean runningDriveCalibration = false;

    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(ModuleName name) {
        this.moduleName = name.toString();
        SwerveModuleConstants constants =
                robotType == Constants.RobotType.DEVELOPMENT ? SwerveModuleConstants.DEVBOT : SwerveModuleConstants.COMPBOT;

        drivePIDController = constants.driveMotorPID[name.ordinal()].createProfiledPIDController();
        driveFeedforward = new SimpleMotorFeedforward(0, constants.driveMotorPID[name.ordinal()].getF());
        steerPIDController = constants.steerMotorPID[name.ordinal()].createPIDController();

        if (RobotBase.isReal()) {
            io = switch (robotType) {
                case DEVELOPMENT -> new SwerveModuleIODevBot(name);
                case COMPETITION -> new SwerveModuleIOCompBot(name);
                case REPLAY -> throw new IllegalArgumentException("Robot cannot be replay if it's real");
            };
        } else {
            io = robotType == Constants.RobotType.REPLAY ? new SwerveModuleIOSIM.SwerveModuleIOReplay() : new SwerveModuleIOSIM();

        }

    }

    public SwerveModulePosition modulePeriodic() {
        io.updateInputs(inputs);

        if (!runningCalibration) {
            targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
            targetState.speedMetersPerSecond *= Math.cos(targetState.angle.getRadians() - inputs.currentState.angle.getRadians());

            double driveMotorVoltageOutput =
                    drivePIDController.calculate(inputs.currentState.speedMetersPerSecond, targetState.speedMetersPerSecond);

            driveMotorVoltageOutput += driveFeedforward.calculate(targetState.speedMetersPerSecond);

            double steerMotorVoltageOutput =
                    steerPIDController.calculate(inputs.currentState.angle.getRadians(), targetState.angle.getRadians());
            steerMotorVoltageOutput = steerPIDController.atSetpoint() ? 0 : steerMotorVoltageOutput;

            io.setDriveMotorVoltage(driveMotorVoltageOutput);
            io.setSteerMotorVoltage(steerMotorVoltageOutput);

            io.run();
        } else {
            if (runningDriveCalibration) {
                drivePIDController.setP(SmartDashboard.getNumber("drive P", drivePIDController.getP()));
                drivePIDController.setI(SmartDashboard.getNumber("drive I", drivePIDController.getI()));
                drivePIDController.setD(SmartDashboard.getNumber("drive D", drivePIDController.getD()));

                drivePIDController.setGoal(SmartDashboard.getNumber("drive setPoint", 0));

                driveFeedforward = new SimpleMotorFeedforward(0, SmartDashboard.getNumber("drive kS", driveFeedforward.ks));

                double driveOut = drivePIDController.calculate(inputs.currentState.speedMetersPerSecond);

                driveOut += driveFeedforward.calculate(drivePIDController.getGoal().position);
                driveOut = MathUtil.clamp(driveOut, -12, 12);

                io.setDriveMotorVoltage(driveOut);

                double steerOut = steerPIDController.calculate(inputs.currentState.angle.getRadians(), targetState.angle.getRadians());

                steerOut = steerPIDController.atSetpoint() ? 0 : steerOut;

                io.setSteerMotorVoltage(steerOut);
            } else {
                double steerOut = steerPIDController.calculate(inputs.currentState.angle.getRadians());


                steerOut = MathUtil.clamp(steerOut, -12, 12);


                io.setSteerMotorVoltage(steerOut);
                io.setDriveMotorVoltage(0);
            }

            io.run();
        }
        Logger.processInputs("SwerveModule/" + moduleName, inputs);

        return new SwerveModulePosition(inputs.drivePositionMeters, inputs.currentState.angle);
    }

    public SwerveModuleState getCurrentState() {
        return inputs.currentState;
    }

    public SwerveModuleState run(SwerveModuleState targetState) {
        targetState = SwerveModuleState.optimize(targetState, inputs.currentState.angle);
        targetState.speedMetersPerSecond *= inputs.currentState.angle.minus(targetState.angle).getCos();

        this.targetState = targetState;

        return targetState;
    }

    public void runSteerCalibration() {
        runningCalibration = true;

        SendableRegistry.setName(steerPIDController, moduleName + " steer Controller");

        SmartDashboard.putData(steerPIDController);
    }

    public void stopSteerCalibration() {
        runningCalibration = false;
    }

    public void runDriveCalibration(){
        runningDriveCalibration = true;
        runningCalibration = true;
    }

    public void stopDriveCalibration(){
        runningCalibration = false;
        runningDriveCalibration = false;
    }

    public void setIdleMode(boolean isBrakeMode) {
        io.setIdleMode(isBrakeMode);
    }

    public void resetDriveEncoder() {
        io.resetDriveEncoder();
    }
}
