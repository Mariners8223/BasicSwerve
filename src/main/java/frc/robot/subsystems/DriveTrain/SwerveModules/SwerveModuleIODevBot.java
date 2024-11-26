package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersMeasurements;
import frc.util.MarinersController.MarinersTalonFX;

public class SwerveModuleIODevBot extends SwerveModuleIO {
    private final MarinersController driveMotor;
    private final MarinersController steerMotor;
    private final CANcoder absEncoder;


    public SwerveModuleIODevBot(SwerveModule.ModuleName name) {

        DevBotConstants constants = DevBotConstants.values()[name.ordinal()];

        driveMotor = new MarinersTalonFX(
                name.name() + " Drive Motor",
                MarinersController.ControllerLocation.MOTOR,
                constants.DRIVE_MOTOR_ID,
                constants.DRIVE_MOTOR_PID,
                DevBotConstants.DRIVE_GEAR_RATIO / DevBotConstants.WHEEL_CIRCUMFERENCE_METERS);

        driveMotor.setMotorInverted(constants.DRIVE_INVERTED);


        driveMotor.setCurrentLimits(50, 70);

        // driveMotor.setProfile(40,40);

        driveMotor.setMotorDeadBandDutyCycle(0.05);

        steerMotor = new MarinersSparkBase(
                name.name() + " Steer Motor",
                MarinersController.ControllerLocation.RIO,
                constants.STEER_MOTOR_ID,
                true,
                MarinersSparkBase.MotorType.SPARK_MAX,
                constants.STEER_MOTOR_PID);


        steerMotor.setCurrentLimits(30, 50);

        steerMotor.setProfile(40, 80);

        steerMotor.setMotorDeadBandVoltage(0.3);

        absEncoder = configCANCoder(constants.ABSOLUTE_ENCODER_ID, constants.ABSOLUTE_ZERO_OFFSET, (int) steerMotor.RUN_HZ);

        steerMotor.setMeasurements(
                new MarinersMeasurements(
                        () -> absEncoder.getPosition().getValueAsDouble(),
                        () -> absEncoder.getVelocity().getValueAsDouble(),
                        1
                )
        );

        steerMotor.enablePositionWrapping(-0.5, 0.5);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getPosition());
        inputs.currentState.speedMetersPerSecond = driveMotor.getVelocity();

        inputs.drivePositionMeters = driveMotor.getPosition();

        inputs.absPosition = absEncoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public void setDriveMotorReference(double reference) {
        driveMotor.setReference(reference, MarinersController.ControlMode.Velocity);
    }

    @Override
    public void setSteerMotorReference(double reference) {
        steerMotor.setReference(reference, MarinersController.ControlMode.ProfiledPosition);
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


}
