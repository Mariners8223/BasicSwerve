package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSimMotor;
import frc.util.PIDFGains;

public class SwerveModuleIOSIM extends SwerveModuleIO {
    private final MarinersController driveMotor;
    private final MarinersController steerMotor;

    private final double DRIVE_KA, WHEEL_CIRCUMFERENCE_METERS;

    public SwerveModuleIOSIM(SwerveModule.ModuleName name) {
        DCMotor driveMotorModel;
        DCMotor steerMotorModel;

        PIDFGains driveMotorPIDController, steerMotorPIDController;

        double DRIVE_KS, DRIVE_KV, STEER_KV, STEER_KA, DRIVE_GEAR_RATIO, STEER_GEAR_RATIO, WHEEL_RADIUS_METERS;

        if (Constants.ROBOT_TYPE == Constants.RobotType.DEVELOPMENT) {
            DRIVE_GEAR_RATIO = DevBotConstants.DRIVE_GEAR_RATIO;
            STEER_GEAR_RATIO = DevBotConstants.STEER_GEAR_RATIO;
            WHEEL_CIRCUMFERENCE_METERS = DevBotConstants.WHEEL_CIRCUMFERENCE_METERS;
            WHEEL_RADIUS_METERS = DevBotConstants.WHEEL_RADIUS_METERS;

            DevBotConstants constants = DevBotConstants.values()[name.ordinal()];

            driveMotorPIDController = constants.DRIVE_MOTOR_PID;
            steerMotorPIDController = constants.STEER_MOTOR_PID;

            driveMotorModel = DCMotor.getKrakenX60(1);
            steerMotorModel = DCMotor.getNEO(1);

            DRIVE_KS = constants.DRIVE_KS;
            DRIVE_KV = constants.DRIVE_KV;
            DRIVE_KA = constants.DRIVE_KA;

            STEER_KV = constants.STEER_KV;
            STEER_KA = constants.STEER_KA;
        }
        else {
            DRIVE_GEAR_RATIO = CompBotConstants.DRIVE_GEAR_RATIO;
            STEER_GEAR_RATIO = CompBotConstants.STEER_GEAR_RATIO;
            WHEEL_CIRCUMFERENCE_METERS = CompBotConstants.WHEEL_CIRCUMFERENCE_METERS;
            WHEEL_RADIUS_METERS = CompBotConstants.WHEEL_RADIUS_METERS;

            CompBotConstants constants = CompBotConstants.values()[name.ordinal()];

            driveMotorPIDController = constants.DRIVE_MOTOR_PID;
            steerMotorPIDController = constants.STEER_MOTOR_PID;

            driveMotorModel = DCMotor.getFalcon500(1);
            steerMotorModel = DCMotor.getNEO(1);

            DRIVE_KS = constants.DRIVE_KS;
            DRIVE_KV = constants.DRIVE_KV;
            DRIVE_KA = constants.DRIVE_KA;

            STEER_KV = constants.STEER_KV;
            STEER_KA = constants.STEER_KA;
        }

        driveMotor = new MarinersSimMotor(name.name() + " Drive Motor", driveMotorModel,
                DRIVE_KV * WHEEL_RADIUS_METERS, DRIVE_KA * WHEEL_RADIUS_METERS, DRIVE_GEAR_RATIO);

        driveMotor.setPIDF(driveMotorPIDController);

        driveMotor.setStaticFeedForward(DRIVE_KS);

        driveMotor.setFeedForward(DRIVE_KV * WHEEL_CIRCUMFERENCE_METERS);

//        steerMotor = new MarinersSimMotor(name.name() + " Steer Motor", steerMotorModel,
//                STEER_KV, STEER_KA, STEER_GEAR_RATIO);
        steerMotor = new MarinersSimMotor(name.name() + " Steer Motor", steerMotorModel,
                STEER_GEAR_RATIO, 0.01);

        steerMotor.setPIDF(steerMotorPIDController);

        steerMotor.enablePositionWrapping(-0.5, 0.5);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentState.speedMetersPerSecond = driveMotor.getVelocity() * WHEEL_CIRCUMFERENCE_METERS;

        inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getPosition());

        inputs.drivePositionMeters = driveMotor.getPosition() * WHEEL_CIRCUMFERENCE_METERS;
    }

    @Override
    public void setDriveMotorReference(double reference) {
        driveMotor.setReference(reference / WHEEL_CIRCUMFERENCE_METERS, MarinersController.ControlMode.Velocity);
    }

    @Override
    public void setDriveMotorReference(double reference, double accelerationFeedForward) {
        driveMotor.setReference(reference / WHEEL_CIRCUMFERENCE_METERS, MarinersController.ControlMode.Velocity, accelerationFeedForward * DRIVE_KA);
    }

    @Override
    public void setDriveMotorVoltage(Voltage voltage) {
        driveMotor.setVoltage(voltage);
    }

    @Override
    public void setSteerMotorReference(double reference) {
        steerMotor.setReference(reference, MarinersController.ControlMode.Position);
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        DriverStation.reportWarning("dummy this is a simulation", false);
    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.resetMotorEncoder();
    }

    @Override
    void startDriveCalibration() {
        driveMotor.startPIDTuning();
    }

    @Override
    void endDriveCalibration() {
        driveMotor.startPIDTuning();
    }

    @Override
    void startSteerCalibration() {
        steerMotor.startPIDTuning();
    }

    @Override
    void endSteerCalibration() {
        steerMotor.startPIDTuning();
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
        public void setDriveMotorReference(double reference, double accelerationFeedForward) {

        }

        @Override
        public void setDriveMotorVoltage(Voltage voltage) {

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
