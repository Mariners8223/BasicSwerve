package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
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


    public SwerveModuleIOSIM(String name) {

        if (Constants.ROBOT_TYPE == Constants.RobotType.DEVELOPMENT) {
            DRIVE_GEAR_RATIO = DevBotConstants.DRIVE_GEAR_RATIO;
            STEER_GEAR_RATIO = DevBotConstants.STEER_GEAR_RATIO;
            WHEEL_RADIUS_METERS = DevBotConstants.WHEEL_RADIUS_METERS;

            DevBotConstants constants = DevBotConstants.valueOf(name);

            driveMotorPIDController = constants.DRIVE_MOTOR_PID.createPIDController();
            steerMotorPIDController = constants.STEER_MOTOR_PID.createPIDController();
        }
        else {
            DRIVE_GEAR_RATIO = CompBotConstants.DRIVE_GEAR_RATIO;
            STEER_GEAR_RATIO = CompBotConstants.STEER_GEAR_RATIO;
            WHEEL_RADIUS_METERS = CompBotConstants.WHEEL_RADIUS_METERS;

            CompBotConstants constants = CompBotConstants.valueOf(name);

            driveMotorPIDController = constants.DRIVE_MOTOR_PID.createPIDController();
            steerMotorPIDController = constants.STEER_MOTOR_PID.createPIDController();
        }

        driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.025 / DRIVE_GEAR_RATIO);

        steerMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.004 / STEER_GEAR_RATIO);



        this.name = name;
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
