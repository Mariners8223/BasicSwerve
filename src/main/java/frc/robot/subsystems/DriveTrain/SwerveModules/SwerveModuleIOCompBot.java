package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.MotorMap;

public class SwerveModuleIOCompBot extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final DutyCycleEncoder absEncoder;
    private final int absEncoderMultiplier = constants.isAbsEncoderInverted ? -1 : 1;


    public SwerveModuleIOCompBot(SwerveModule.ModuleName name) {
        int driveMotorID = MotorMap.DriveBase.modules[name.ordinal()][0];
        int steerMotorID = MotorMap.DriveBase.modules[name.ordinal()][1];
        int absEncoderID = MotorMap.DriveBase.modules[name.ordinal()][2];

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

        inputs.absEncoderPosition = (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * absEncoderMultiplier;

        inputs.steerVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(steerMotor.getEncoder().getVelocity() / constants.steerGearRatio);
        
        inputs.drivePositionMeters =
                (driveMotor.getPosition().getValueAsDouble() / constants.driveGearRatio) * constants.wheelCircumferenceMeters;

        inputs.driveMotorAppliedVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.steerMotorAppliedVoltage = steerMotor.getAppliedOutput() * steerMotor.getBusVoltage();
    }

    @Override
    protected void sendInputsToMotors(double driveMotorVoltageInput, double steerMotorVoltageInput) {
        driveMotor.setVoltage(driveMotorVoltageInput);
        steerMotor.setVoltage(steerMotorVoltageInput);
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

}
