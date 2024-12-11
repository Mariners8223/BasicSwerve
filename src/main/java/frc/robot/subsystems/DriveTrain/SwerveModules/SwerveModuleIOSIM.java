package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSimMotor;
import frc.util.PIDFGains;

public class SwerveModuleIOSIM extends SwerveModuleIO {
    private final MarinersController driveMotor;
    private final MarinersController steerMotor;


    public SwerveModuleIOSIM(SwerveModule.ModuleName name) {
        double DRIVE_GEAR_RATIO;
        double WHEEL_CIRCUMFERENCE_METERS;

        double STEER_GEAR_RATIO;

        PIDFGains drivePIDF;
        PIDFGains steerPIDF;

        if (Constants.ROBOT_TYPE == Constants.RobotType.DEVELOPMENT) {
            DRIVE_GEAR_RATIO = DevBotConstants.DRIVE_GEAR_RATIO;
            STEER_GEAR_RATIO = DevBotConstants.STEER_GEAR_RATIO;
            WHEEL_CIRCUMFERENCE_METERS = DevBotConstants.WHEEL_CIRCUMFERENCE_METERS;

            DevBotConstants constants = DevBotConstants.values()[name.ordinal()];

            drivePIDF = constants.DRIVE_MOTOR_PID;
            steerPIDF = constants.STEER_MOTOR_PID;
        }
        else {
            DRIVE_GEAR_RATIO = CompBotConstants.DRIVE_GEAR_RATIO;
            STEER_GEAR_RATIO = CompBotConstants.STEER_GEAR_RATIO;
            WHEEL_CIRCUMFERENCE_METERS = CompBotConstants.WHEEL_CIRCUMFERENCE_METERS;

            CompBotConstants constants = CompBotConstants.values()[name.ordinal()];

            drivePIDF = constants.DRIVE_MOTOR_PID;
            steerPIDF = constants.STEER_MOTOR_PID;
        }

        driveMotor = new MarinersSimMotor(name.name() + " Drive Motor",
                DCMotor.getKrakenX60(1), DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE_METERS, 1);

        driveMotor.setPIDF(drivePIDF);

        steerMotor = new MarinersSimMotor(name.name() + " Steer Motor",
                DCMotor.getNEO(1), STEER_GEAR_RATIO, 0.01);

        steerMotor.setPIDF(steerPIDF);

        steerMotor.enablePositionWrapping(-0.5, 0.5);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentState.speedMetersPerSecond = driveMotor.getVelocity();

        inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getPosition());

        inputs.drivePositionMeters = driveMotor.getPosition();
    }

    @Override
    public void setDriveMotorReference(double reference) {
        driveMotor.setReference(reference, MarinersController.ControlMode.Velocity);
    }

    @Override
    public void setDriveMotorVoltage(double voltage) {
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
        driveMotor.stopPIDTuning();
    }

    @Override
    void startSteerCalibration() {
        steerMotor.startPIDTuning();
    }

    @Override
    void endSteerCalibration() {
        steerMotor.stopPIDTuning();
    }

    @Override
    MarinersController getDriveMotorController() {
        return driveMotor;
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
        public void setDriveMotorVoltage(double voltage) {

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

        @Override
        MarinersController getDriveMotorController() {
            return null;
        }
    }
}
