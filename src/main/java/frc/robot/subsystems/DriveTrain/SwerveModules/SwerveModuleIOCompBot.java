package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.MotorMap;
import frc.util.PIDFGains;

public class SwerveModuleIOCompBot extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final DutyCycleEncoder absEncoder;
    private final int absEncoderMultiplier = constants.isAbsEncoderInverted ? -1 : 1;

    private final VelocityDutyCycle driveMotorVelocityDutyCycle =
            new VelocityDutyCycle(0).withEnableFOC(false);


    public SwerveModuleIOCompBot(SwerveModule.ModuleName name) {
        int driveMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][0];
        int steerMotorID = MotorMap.DriveBase.MODULES[name.ordinal()][1];
        int absEncoderID = MotorMap.DriveBase.MODULES[name.ordinal()][2];

        double zeroOffset = constants.abs_zeroOffsets[name.ordinal()];

        absEncoder = configDutyCycleEncoder(absEncoderID, zeroOffset);

        driveMotor = configTalonFX(getTalonFXConfiguration(name), driveMotorID);
        steerMotor = configCanSparkMax(steerMotorID, absEncoder.get() * absEncoderMultiplier, name);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        inputs.currentState.angle = Rotation2d.fromRotations(absEncoder.get() * absEncoderMultiplier);
        // inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() / DevBotConstants.steerGearRatio);

        inputs.currentState.speedMetersPerSecond =
                (driveMotor.getVelocity().getValueAsDouble() / constants.driveGearRatio) * constants.wheelCircumferenceMeters;

        inputs.driveMotorRPM = driveMotor.getVelocity().getValueAsDouble() * 60;

        inputs.absEncoderPosition = (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * absEncoderMultiplier;

        inputs.steerVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(steerMotor.getEncoder().getVelocity() / constants.steerGearRatio);

        inputs.drivePositionMeters =
                (driveMotor.getPosition().getValueAsDouble() / constants.driveGearRatio) * constants.wheelCircumferenceMeters;

        inputs.driveMotorAppliedOutput = driveMotor.getDutyCycle().getValueAsDouble();
        inputs.steerMotorAppliedOutput = steerMotor.getAppliedOutput();

        inputs.driveMotorAppliedVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.steerMotorAppliedVoltage = inputs.steerMotorAppliedOutput * steerMotor.getBusVoltage();

        inputs.driveMotorTemperature = driveMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    protected void sendInputsToMotors(double driveMotorReference, double steerMotorReference) {
        double driveMotorOut = (driveMotorReference / constants.wheelCircumferenceMeters) * constants.driveGearRatio;
        double steerMotorOut = steerMotorReference * constants.steerGearRatio;

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

    @Override
    void setSteerMotorPID(PIDFGains pidGains) {
        SparkPIDController pidController = steerMotor.getPIDController();

        pidController.setP(pidGains.getP());
        pidController.setI(pidGains.getI());
        pidController.setD(pidGains.getD());
        pidController.setFF(pidGains.getF());
    }

}
