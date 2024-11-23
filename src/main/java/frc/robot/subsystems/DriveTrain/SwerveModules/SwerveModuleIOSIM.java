package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.util.PIDFGains;

public class SwerveModuleIOSIM extends SwerveModuleIO {
    private final DCMotorSim driveMotor;
    private final DCMotorSim steerMotor;

    private double driveMotorVoltage = 0;
    private double steerMotorVoltage = 0;

    private PIDController driveMotorPIDController;
    private final PIDController steerMotorPIDController;

    public SwerveModuleIOSIM() {
        driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.025 / constants.DRIVE_GEAR_RATIO);

        steerMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.004 / constants.STEER_GEAR_RATIO);

        driveMotorPIDController = constants.DRIVE_MOTOR_PID[0].createPIDController();
        steerMotorPIDController = constants.STEER_MOTOR_PID[0].createPIDController();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        driveMotor.update(1 / SwerveModule.MODULE_THREAD_HZ);
        steerMotor.update(1 / SwerveModule.MODULE_THREAD_HZ);

        inputs.currentState.speedMetersPerSecond =
                (driveMotor.getAngularVelocityRadPerSec() / constants.DRIVE_GEAR_RATIO) * constants.WHEEL_RADIUS_METERS;

        inputs.currentState.angle = Rotation2d.fromRadians(steerMotor.getAngularPositionRad() / constants.STEER_GEAR_RATIO);

        inputs.drivePositionMeters =
                (driveMotor.getAngularPositionRad() / constants.DRIVE_GEAR_RATIO) * constants.WHEEL_RADIUS_METERS;
    }

    @Override
    public void setDriveMotorReference(double reference) {
        double driveMotorVelocity = driveMotor.getAngularVelocityRPM() / 60; //turn to rotations per second

        double driveMotorReferenceNativeUnits =
                (reference / constants.WHEEL_CIRCUMFERENCE_METERS) * constants.DRIVE_GEAR_RATIO;

        driveMotorVoltage = driveMotorPIDController.calculate(driveMotorVelocity, driveMotorReferenceNativeUnits);

        driveMotor.setInputVoltage(driveMotorVoltage * RobotController.getBatteryVoltage());
    }

    @Override
    public void setSteerMotorReference(double reference) {
        double steerMotorPosition = steerMotor.getAngularPositionRotations();

        double steerMotorReferenceNativeUnits = reference * constants.STEER_GEAR_RATIO;

        steerMotorVoltage = steerMotorPIDController.calculate(steerMotorPosition, steerMotorReferenceNativeUnits);

        steerMotor.setInputVoltage(steerMotorVoltage * RobotController.getBatteryVoltage());
    }

    @Override
    public void setIdleMode(boolean isBrakeMode) {
        DriverStation.reportWarning("dummy this is a simulation", false);
    }

    @Override
    public void resetDriveEncoder() {
        driveMotor.setState(0, 0);
    }

    @Override
    void setDriveMotorPID(PIDFGains pidGains) {
        driveMotorPIDController = pidGains.createPIDController();
    }



    /**
     * A fake class for replaying data
     */
    public static class SwerveModuleIOReplay extends SwerveModuleIO {
        @Override
        public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
        }

        @Override
        public void setDriveMotorReference(double reference) {

        }

        @Override
        public void setSteerMotorReference(double reference) {

        }

        @Override
        public void setIdleMode(boolean isBrakeMode) {
        }

        @Override
        public void resetDriveEncoder() {
        }

        @Override
        void setDriveMotorPID(PIDFGains pidGains) {

        }
    }
}
