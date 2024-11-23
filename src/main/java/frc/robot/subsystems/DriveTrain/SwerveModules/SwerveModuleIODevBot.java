package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.MotorMap;
import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersMeasurements;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.PIDFGains;

public class SwerveModuleIODevBot extends SwerveModuleIO {
    private final MarinersController driveMotor;
    private final MarinersController steerMotor;

    private final VelocityDutyCycle driveMotorVelocityDutyCycle =
            new VelocityDutyCycle(0).withEnableFOC(false);


    public SwerveModuleIODevBot(SwerveModule.ModuleName name) {
        int driveMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][0];
        int steerMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][1];
        int absEncoderID = MotorMap.DriveBase.MODULES[name.ordinal()][2];

        double zeroOffset = constants.ABSOLUTE_ZERO_OFFSETS[name.ordinal()];


        driveMotor = new MarinersTalonFX(
                name.name() + " Drive Motor",
                MarinersController.ControllerLocation.MOTOR,
                driveMotorID,
                constants.DRIVE_MOTOR_PID[name.ordinal()],
                constants.DRIVE_GEAR_RATIO * constants.WHEEL_CIRCUMFERENCE_METERS);

        steerMotor = new MarinersSparkBase(
                name.name() + " Steer Motor",
                MarinersController.ControllerLocation.RIO,
                steerMotorID,
                true,
                MarinersSparkBase.MotorType.SPARK_MAX,
                constants.STEER_MOTOR_PID[name.ordinal()]);

        CANcoder absEncoder = configCANCoder(absEncoderID, zeroOffset, (int) steerMotor.RUN_HZ);

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
    void setDriveMotorPID(PIDFGains pidGains) {
        driveMotor.setPIDF(pidGains);
    }

}
