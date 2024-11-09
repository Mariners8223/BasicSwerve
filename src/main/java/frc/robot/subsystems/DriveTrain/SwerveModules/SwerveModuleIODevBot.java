package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.MotorMap;
import frc.util.MarinersController.BaseController;
import frc.util.MarinersController.MarinerSparkBase;
import frc.util.MarinersController.MarinersMeasurements;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.PIDFGains;

public class SwerveModuleIODevBot extends SwerveModuleIO {
    private final BaseController driveMotor;
    private final BaseController steerMotor;
    private final CANcoder absEncoder;

    private final VelocityDutyCycle driveMotorVelocityDutyCycle =
            new VelocityDutyCycle(0).withEnableFOC(false);


    public SwerveModuleIODevBot(SwerveModule.ModuleName name) {
        int driveMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][0];
        int steerMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][1];
        int absEncoderID = MotorMap.DriveBase.MODULES[name.ordinal()][2];

        double zeroOffset = constants.ABSOLUTE_ZERO_OFFSETS[name.ordinal()];


        driveMotor = new MarinersTalonFX(
                name.name() + " Drive Motor",
                BaseController.ControllerLocation.MOTOR,
                driveMotorID,
                constants.DRIVE_MOTOR_PID[name.ordinal()],
                constants.DRIVE_GEAR_RATIO * constants.WHEEL_CIRCUMFERENCE_METERS);

        steerMotor = new MarinerSparkBase(
                name.name() + " Steer Motor",
                BaseController.ControllerLocation.RIO,
                steerMotorID,
                true,
                MarinerSparkBase.MotorType.SPARK_MAX,
                constants.STEER_MOTOR_PID[name.ordinal()]);

        absEncoder = configCANCoder(absEncoderID, zeroOffset, (int)steerMotor.RUN_HZ);

        steerMotor.setMeasurements(
                new MarinersMeasurements(
                        absEncoder.getPosition()::getValueAsDouble,
                        absEncoder.getVelocity()::getValueAsDouble,
                        1
                )
        );
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getMeasurements().getPosition());
        inputs.currentState.speedMetersPerSecond = driveMotor.getMeasurements().getVelocity();

        inputs.drivePositionMeters = driveMotor.getMeasurements().getPosition();
    }

    @Override
    protected void sendInputsToMotors(double driveMotorReference, double steerMotorReference) {
        driveMotor.setReference(driveMotorReference, BaseController.ControlMode.ProfiledVelocity);

        steerMotor.setReference(steerMotorReference, BaseController.ControlMode.ProfiledPosition);
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
    void setDriveMotorPID(PIDFGains pidGains) {
        driveMotor.setPIDF(pidGains);
    }

}
