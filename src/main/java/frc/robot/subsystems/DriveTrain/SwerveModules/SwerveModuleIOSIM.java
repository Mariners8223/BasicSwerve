package frc.robot.subsystems.DriveTrain.SwerveModules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.DriveTrain.SwerveModules.SwerveModuleIODevBot.DevBotConstants;

public class SwerveModuleIOSIM implements SwerveModuleIO{
  private final DCMotorSim driveMotor;
  private final DCMotorSim steerMotor;

  private final double driveMotorGearRatio = Constants.robotType == RobotType.DEVELOPMENT ? DevBotConstants.driveGearRatio : SwerveModuleIOCompBot.CompBotConstants.driveGearRatio;
  private final double driveWheelRadiusMeters = Constants.robotType == RobotType.DEVELOPMENT ? DevBotConstants.wheelRadiusMeters : SwerveModuleIOCompBot.CompBotConstants.wheelRadiusMeters;
  private final double steerMotorGearRatio = Constants.robotType == RobotType.DEVELOPMENT ? DevBotConstants.steerGearRatio : SwerveModuleIOCompBot.CompBotConstants.steerGearRatio;

  private double driveMotorVoltage = 0;
  private double steerMotorVoltage = 0;

  public SwerveModuleIOSIM(){
    driveMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.25);
    if (Constants.robotType == Constants.RobotType.DEVELOPMENT) {
      steerMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 0.25);
    } else {
      steerMotor = new DCMotorSim(DCMotor.getNeo550(1), 1, 0.25);
    }
  }

  @Override
  public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    driveMotor.update(1 / SwerveModule.moduleThreadHz);
    steerMotor.update(1 / SwerveModule.moduleThreadHz);

    inputs.currentState.speedMetersPerSecond = (driveMotor.getAngularVelocityRadPerSec() / driveMotorGearRatio) * driveWheelRadiusMeters;
    inputs.currentState.angle = Rotation2d.fromRadians(steerMotor.getAngularPositionRad() / steerMotorGearRatio);

    inputs.steerVelocityRadPerSec = steerMotor.getAngularVelocityRadPerSec() / steerMotorGearRatio;
    inputs.drivePositionMeters = driveMotor.getAngularPositionRad() * driveWheelRadiusMeters;

    inputs.driveMotorAppliedVoltage = driveMotorVoltage;
    inputs.steerMotorAppliedVoltage = steerMotorVoltage;
  }

  @Override
  public void setDriveMotorVoltage(double voltage) {
    driveMotorVoltage = MathUtil.clamp(voltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    driveMotor.setInputVoltage(voltage);
  }

  @Override
  public void setSteerMotorVoltage(double voltage) {
    steerMotorVoltage = MathUtil.clamp(voltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    steerMotor.setInputVoltage(voltage);
  }

  @Override
  public void setIdleMode(boolean isBrakeMode) {
    DriverStation.reportWarning("dummy this is a simulation", false);
  }

  @Override
  public void resetDriveEncoder() {
    driveMotor.setState(0, 0);
  }
}
