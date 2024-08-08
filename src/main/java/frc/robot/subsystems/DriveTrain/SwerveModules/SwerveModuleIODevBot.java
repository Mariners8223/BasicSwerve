package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.MotorMap;

public class SwerveModuleIODevBot extends SwerveModuleIO {

    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final CANcoder absEncoder;

    public SwerveModuleIODevBot(SwerveModule.ModuleName name) {
        int driveMotorID = MotorMap.DriveBase.modules[name.ordinal()][0];
        int steerMotorID = MotorMap.DriveBase.modules[name.ordinal()][1];
        int absEncoderID = MotorMap.DriveBase.modules[name.ordinal()][2];

        double zeroOffset = constants.abs_zeroOffsets[name.ordinal()];

        absEncoder = configCANCoder(absEncoderID, zeroOffset);

        driveMotor = configTalonFX(getTalonFXConfiguration(name), driveMotorID);
        steerMotor = configCanSparkMax(steerMotorID, absEncoder.getAbsolutePosition().getValueAsDouble(), name);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        // inputs.currentState.angle = Rotation2d.fromRotations(absEncoder.getPosition().getValueAsDouble());
        inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() / constants.steerGearRatio);

        inputs.currentState.speedMetersPerSecond =
                (driveMotor.getVelocity().getValueAsDouble() / constants.driveGearRatio) * constants.wheelCircumferenceMeters;

        inputs.absEncoderPosition = (absEncoder.getAbsolutePosition().getValueAsDouble());

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
