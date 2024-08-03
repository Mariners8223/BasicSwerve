package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.Constants.robotType;

/* TODO: make actual interface and create a logger subclass */
public abstract class SwerveModuleIO {
    @AutoLog
    static class SwerveModuleIOInputs {
        public SwerveModuleState currentState = new SwerveModuleState();

        public double steerVelocityRadPerSec = 0.0;
        public double drivePositionMeters = 0.0;

        public double steerMotorAppliedVoltage = 0.0;
        public double driveMotorAppliedVoltage = 0.0;

        public double absEncoderPosition = 0.0;
    }

    protected final SwerveModuleConstants constants =
            robotType ==Constants.RobotType.DEVELOPMENT ? SwerveModuleConstants.DEVBOT : SwerveModuleConstants.COMPBOT;

    /**
     * Updates the inputs of the module
     */
    abstract void updateInputs(SwerveModuleIOInputsAutoLogged inputs);

    /**
     * sets the voltage for the drive motor
     *
     * @param voltage the voltage to set the drive motor to
     */
    abstract void setDriveMotorVoltage(double voltage);

    /**
     * sets the voltage for the steer motor
     *
     * @param voltage the voltage to set the steer motor to
     */
    abstract void setSteerMotorVoltage(double voltage);

    /**
     * sets the idle mode of the module
     *
     * @param isBrakeMode true for brake mode, false for coast mode
     */
    abstract void setIdleMode(boolean isBrakeMode);

    /**
     * resets the drive encoder
     */
    abstract void resetDriveEncoder();

}
