package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModuleIOSIM extends SwerveModuleIO {
    private final DCMotorSim driveMotor;
    private final DCMotorSim steerMotor;

    private final PIDController driveMotorPIDController;
    private final PIDController steerMotorPIDController;

    private final String name;

    private final double DRIVE_GEAR_RATIO;
    private final double STEER_GEAR_RATIO;
    private final double WHEEL_RADIUS_METERS;


    public SwerveModuleIOSIM(SwerveModule.ModuleName name) {
        DCMotor driveMotorModel;
        DCMotor steerMotorModel;

        double DRIVE_KV, DRIVE_KA, STEER_KV, STEER_KA;

        if (Constants.ROBOT_TYPE == Constants.RobotType.DEVELOPMENT) {
            DRIVE_GEAR_RATIO = DevBotConstants.DRIVE_GEAR_RATIO;
            STEER_GEAR_RATIO = DevBotConstants.STEER_GEAR_RATIO;
            WHEEL_RADIUS_METERS = DevBotConstants.WHEEL_RADIUS_METERS;

            DevBotConstants constants = DevBotConstants.values()[name.ordinal()];

            driveMotorPIDController = constants.DRIVE_MOTOR_PID.createPIDController();
            steerMotorPIDController = constants.STEER_MOTOR_PID.createPIDController();

            driveMotorModel = DCMotor.getKrakenX60(1);
            steerMotorModel = DCMotor.getNEO(1);

            DRIVE_KV = constants.DRIVE_KV;
            DRIVE_KA = constants.DRIVE_KA;
            STEER_KV = constants.STEER_KV;
            STEER_KA = constants.STEER_KA;
        }
        else {
            DRIVE_GEAR_RATIO = CompBotConstants.DRIVE_GEAR_RATIO;
            STEER_GEAR_RATIO = CompBotConstants.STEER_GEAR_RATIO;
            WHEEL_RADIUS_METERS = CompBotConstants.WHEEL_RADIUS_METERS;

            CompBotConstants constants = CompBotConstants.values()[name.ordinal()];

            driveMotorPIDController = constants.DRIVE_MOTOR_PID.createPIDController();
            steerMotorPIDController = constants.STEER_MOTOR_PID.createPIDController();

            driveMotorModel = DCMotor.getFalcon500(1);
            steerMotorModel = DCMotor.getNEO(1);

            DRIVE_KV = constants.DRIVE_KV;
            DRIVE_KA = constants.DRIVE_KA;
            STEER_KV = constants.STEER_KV;
            STEER_KA = constants.STEER_KA;
        }

        driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DRIVE_KV, DRIVE_KA),
                driveMotorModel.withReduction(DRIVE_GEAR_RATIO), 0);

        steerMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(STEER_KV, STEER_KA),
                steerMotorModel.withReduction(STEER_GEAR_RATIO), 0);

        this.name = name.name();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        driveMotor.update(1 / SwerveModule.MODULE_THREAD_HZ);
        steerMotor.update(1 / SwerveModule.MODULE_THREAD_HZ);

        inputs.currentState.speedMetersPerSecond =
                (driveMotor.getAngularVelocityRadPerSec() /DRIVE_GEAR_RATIO) * WHEEL_RADIUS_METERS;

        inputs.currentState.angle = Rotation2d.fromRadians(steerMotor.getAngularPositionRad() / STEER_GEAR_RATIO);

        inputs.drivePositionMeters =
                (driveMotor.getAngularPositionRad() / DRIVE_GEAR_RATIO) * WHEEL_RADIUS_METERS;
    }

    @Override
    public void setDriveMotorReference(double reference) {
        double driveMotorVelocity = driveMotor.getAngularVelocityRPM() / 60; //turn to rotations per second

        double driveMotorReferenceNativeUnits =
                (reference / WHEEL_RADIUS_METERS) * DRIVE_GEAR_RATIO;

        double driveMotorVoltage = driveMotorPIDController.calculate(driveMotorVelocity, driveMotorReferenceNativeUnits);

        driveMotor.setInputVoltage(driveMotorVoltage * RobotController.getBatteryVoltage());
    }

    @Override
    public void setSteerMotorReference(double reference) {
        double steerMotorPosition = steerMotor.getAngularPositionRotations();

        double steerMotorReferenceNativeUnits = reference * STEER_GEAR_RATIO;

        double steerMotorVoltage = steerMotorPIDController.calculate(steerMotorPosition, steerMotorReferenceNativeUnits);

        steerMotor.setInputVoltage(steerMotorVoltage * RobotController.getBatteryVoltage());
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        DriverStation.reportWarning("dummy this is a simulation", false);
    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.setState(0, 0);
    }

    @Override
    void startDriveCalibration() {
        SmartDashboard.putData(name + " Drive Motor PID", driveMotorPIDController);
    }

    @Override
    void endDriveCalibration() {

    }

    @Override
    void startSteerCalibration() {
        SmartDashboard.putData(name + " Steer Motor PID", steerMotorPIDController);
    }

    @Override
    void endSteerCalibration() {

    }


    /**
     * A fake class for replaying data
     */
    public static class SwerveModuleIOReplay extends SwerveModuleIO {
        @Override
        public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        }

        @Override
        public void setDriveMotorReference(double reference) {

        }

        @Override
        public void setSteerMotorReference(double reference) {

        }

        @Override
        public void setIdleMode(boolean isBrakeMode) {
        }

        @Override
        public void resetDriveEncoder() {
        }

        @Override
        void startDriveCalibration() {

        }

        @Override
        void endDriveCalibration() {

        }

        @Override
        void startSteerCalibration() {

        }

        @Override
        void endSteerCalibration() {

        }
    }
}
