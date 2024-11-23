package frc.robot.subsystems.DriveTrain.SwerveModules;


import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.MotorMap;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersMeasurements;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.PIDFGains;

public class SwerveModuleIOCompBot extends SwerveModuleIO {
    private final MarinersController driveMotor;
    private final MarinersController steerMotor;

    public SwerveModuleIOCompBot(SwerveModule.ModuleName name) {
        int driveMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][0];
        int steerMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][1];
        int absEncoderID = MotorMap.DriveBase.MODULES[name.ordinal()][2];

        double zeroOffset = constants.ABSOLUTE_ZERO_OFFSETS[name.ordinal()];

        DutyCycleEncoder absEncoder = configDutyCycleEncoder(absEncoderID, zeroOffset);


        driveMotor = new MarinersTalonFX(
                name.name() + " Drive Motor",
                MarinersController.ControllerLocation.MOTOR,
                driveMotorID,
                constants.DRIVE_MOTOR_PID[name.ordinal()],
                constants.DRIVE_GEAR_RATIO / constants.WHEEL_CIRCUMFERENCE_METERS);

        steerMotor = new MarinersSparkBase(
                name.name() + " Steer Motor",
                MarinersController.ControllerLocation.RIO,
                steerMotorID,
                true,
                MarinersSparkBase.MotorType.SPARK_MAX,
                constants.STEER_MOTOR_PID[name.ordinal()]);

        double absEncoderMultiplier = constants.ABSOLUTE_ENCODER_INVERTED ? -1 : 1;

        steerMotor.setMeasurements(
                new MarinersMeasurements(
                        () -> absEncoder.get() * absEncoderMultiplier,
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
    public void setSteerMotorReference(double reference) {
        steerMotor.setReference(reference, MarinersController.ControlMode.Position);
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
