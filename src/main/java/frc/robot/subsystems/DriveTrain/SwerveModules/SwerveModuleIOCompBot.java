package frc.robot.subsystems.DriveTrain.SwerveModules;


import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersMeasurements;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersTalonFX;

public class SwerveModuleIOCompBot extends SwerveModuleIO {
    private final MarinersController driveMotor;
    private final MarinersController steerMotor;

    private final double DRIVE_KA;

    public SwerveModuleIOCompBot(SwerveModule.ModuleName name) {

        CompBotConstants constants = CompBotConstants.values()[name.ordinal()];

        DutyCycleEncoder absEncoder = configDutyCycleEncoder(constants.ABSOLUTE_ENCODER_ID, constants.ABSOLUTE_ZERO_OFFSET);

        this.DRIVE_KA = constants.DRIVE_KA;


        driveMotor = new MarinersTalonFX(
                name.name() + " Drive Motor",
                MarinersController.ControllerLocation.MOTOR,
                constants.DRIVE_MOTOR_ID,
                constants.DRIVE_MOTOR_PID,
                CompBotConstants.DRIVE_GEAR_RATIO / CompBotConstants.WHEEL_CIRCUMFERENCE_METERS);

        steerMotor = new MarinersSparkBase(
                name.name() + " Steer Motor",
                MarinersController.ControllerLocation.RIO,
                constants.STEER_MOTOR_ID,
                true,
                MarinersSparkBase.MotorType.SPARK_MAX,
                constants.STEER_MOTOR_PID);

        steerMotor.setMeasurements(
                new MarinersMeasurements(
                        absEncoder::get,
                        1
                )
        );

    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentState = new SwerveModuleState(driveMotor.getVelocity(), Rotation2d.fromRotations(steerMotor.getPosition()));

        inputs.drivePositionMeters = driveMotor.getPosition();
    }

    @Override
    public void setDriveMotorReference(double reference) {
        driveMotor.setReference(reference, MarinersController.ControlMode.Velocity);
    }

    @Override
    public void setDriveMotorReference(double reference, double acceleration) {
        driveMotor.setReference(reference, MarinersController.ControlMode.Velocity,
                acceleration * DRIVE_KA);
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
    public void setSteerMotorVoltage(double voltage) {
        steerMotor.setVoltage(voltage);
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        driveMotor.setMotorIdleMode(isBrakeMode);
        steerMotor.setMotorIdleMode(isBrakeMode);
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
}
