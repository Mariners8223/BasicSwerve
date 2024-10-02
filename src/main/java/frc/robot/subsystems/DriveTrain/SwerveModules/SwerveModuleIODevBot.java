package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.MotorMap;
import frc.util.PIDFGains;

public class SwerveModuleIODevBot extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final DutyCycleEncoder absEncoder;
    private final int absEncoderMultiplier = constants.ABSOLUTE_ENCODER_INVERTED ? -1 : 1;

    private final VelocityDutyCycle driveMotorVelocityDutyCycle =
            new VelocityDutyCycle(0).withEnableFOC(false);


    public SwerveModuleIODevBot(SwerveModule.ModuleName name) {
        int driveMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][0];
        int steerMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][1];
        int absEncoderID = MotorMap.DriveBase.MODULES[name.ordinal()][2];

        double zeroOffset = constants.ABSOLUTE_ZERO_OFFSETS[name.ordinal()];

        absEncoder = configDutyCycleEncoder(absEncoderID, zeroOffset);

        driveMotor = configTalonFX(getTalonFXConfiguration(name), driveMotorID);
        steerMotor = configCanSparkMax(steerMotorID, absEncoder.get() * absEncoderMultiplier, name);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentState.angle = Rotation2d.fromRotations(absEncoder.get() * absEncoderMultiplier);
        // inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() / DevBotConstants.steerGearRatio);

        inputs.currentState.speedMetersPerSecond =
                (driveMotor.getVelocity().getValueAsDouble() / constants.DRIVE_GEAR_RATIO) * constants.WHEEL_CIRCUMFERENCE_METERS;

        inputs.driveMotorRPM = driveMotor.getVelocity().getValueAsDouble() * 60;

        inputs.absEncoderPosition = (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * absEncoderMultiplier;

        inputs.steerVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(steerMotor.getEncoder().getVelocity() / constants.STEER_GEAR_RATIO);

        inputs.drivePositionMeters =
                (driveMotor.getPosition().getValueAsDouble() / constants.DRIVE_GEAR_RATIO) * constants.WHEEL_CIRCUMFERENCE_METERS;

        inputs.driveMotorAppliedOutput = driveMotor.getDutyCycle().getValueAsDouble();
        inputs.steerMotorAppliedOutput = steerMotor.getAppliedOutput();

        inputs.driveMotorAppliedVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.steerMotorAppliedVoltage = inputs.steerMotorAppliedOutput * steerMotor.getBusVoltage();

        inputs.driveMotorTemperature = driveMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    protected void sendInputsToMotors(double driveMotorReference, double steerMotorReference) {
        double driveMotorOut = (driveMotorReference / constants.WHEEL_CIRCUMFERENCE_METERS) * constants.DRIVE_GEAR_RATIO;
        double steerMotorOut = steerMotorReference * constants.STEER_GEAR_RATIO;

        driveMotor.setControl(driveMotorVelocityDutyCycle.withVelocity(driveMotorOut));
        steerMotor.getPIDController().setReference(steerMotorOut, CANSparkBase.ControlType.kPosition);
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        driveMotor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        steerMotor.setIdleMode(isBrakeMode ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.setPosition(0);
    }

    @Override
    void setDriveMotorPID(PIDFGains pidGains) {
        Slot0Configs config = new Slot0Configs();

        config.kP = pidGains.getP();
        config.kI = pidGains.getI();
        config.kD = pidGains.getD();
        config.kV = pidGains.getF();

        driveMotor.getConfigurator().apply(config);
    }

}
