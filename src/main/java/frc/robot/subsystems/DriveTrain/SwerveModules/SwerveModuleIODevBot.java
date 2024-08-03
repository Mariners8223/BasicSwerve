package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.MotorMap;
import frc.util.PIDFGains;

public class SwerveModuleIODevBot extends SwerveModuleIO{

  private final TalonFX driveMotor;
  private final CANSparkMax steerMotor;
  private final CANcoder absEncoder;

  public SwerveModuleIODevBot(SwerveModule.ModuleName name){
    int driveMotorID = MotorMap.DriveBase.modules[name.ordinal()][0];
    int steerMotorID = MotorMap.DriveBase.modules[name.ordinal()][1];
    int absEncoderID = MotorMap.DriveBase.modules[name.ordinal()][2];

    double zeroOffset = constants.abs_zeroOffsets[name.ordinal()];

    absEncoder = configCANCoder(absEncoderID, zeroOffset);

    driveMotor = configTalonFX(getTalonFXConfiguration(), driveMotorID);
    steerMotor = configCanSparkMax(steerMotorID);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    // inputs.currentState.angle = Rotation2d.fromRotations(absEncoder.getPosition().getValueAsDouble());
    inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() / constants.steerGearRatio);

    inputs.currentState.speedMetersPerSecond = (driveMotor.getVelocity().getValueAsDouble() / constants.driveGearRatio) * constants.wheelCircumferenceMeters;

    inputs.absEncoderPosition = (absEncoder.getAbsolutePosition().getValueAsDouble());

    inputs.steerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(steerMotor.getEncoder().getVelocity() / constants.steerGearRatio);
    inputs.drivePositionMeters = (driveMotor.getPosition().getValueAsDouble() / constants.driveGearRatio) * constants.wheelCircumferenceMeters;

    inputs.driveMotorAppliedVoltage = driveMotor.getMotorVoltage().getValueAsDouble();
    inputs.steerMotorAppliedVoltage = steerMotor.getAppliedOutput() * steerMotor.getBusVoltage();
  }

  @Override
  public void setDriveMotorVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void setSteerMotorVoltage(double voltage) {
    steerMotor.setVoltage(voltage);
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


  private CANcoder configCANCoder(int absEncoderID, double absoluteEncoderZeroOffset){
    CANcoder canCoder = new CANcoder(absEncoderID);

    canCoder.getPosition().setUpdateFrequency(SwerveModule.moduleThreadHz);
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.FutureProofConfigs = false;

    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = constants.isAbsEncoderInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = -absoluteEncoderZeroOffset;

    canCoder.getPosition().setUpdateFrequency(SwerveModule.moduleThreadHz);
    canCoder.setPosition(canCoder.getAbsolutePosition().getValueAsDouble());

    canCoder.getConfigurator().apply(config);
    canCoder.optimizeBusUtilization();

    return canCoder;
  }

  private TalonFX configTalonFX(TalonFXConfiguration config, int driveMotorID){
    TalonFX talonFX = new TalonFX(driveMotorID); //creates a new talon fx

    talonFX.getConfigurator().apply(config); //apply the given config

    talonFX.getPosition().setUpdateFrequency(SwerveModule.moduleThreadHz); //position is needed more for destroy

    talonFX.getVelocity().setUpdateFrequency(SwerveModule.moduleThreadHz); //sets as default
    talonFX.getMotorVoltage().setUpdateFrequency(50); //sets as default
    talonFX.getSupplyCurrent().setUpdateFrequency(50); //sets as default
    talonFX.getStatorCurrent().setUpdateFrequency(50); //sets as default
    talonFX.getDeviceTemp().setUpdateFrequency(50);

    talonFX.optimizeBusUtilization(); //optimizes canbus util

    talonFX.setPosition(0);

    return talonFX; //returns the ready talon fx
  }

  /**
   * creates a config for the talonFX
   * @return the new config
   */
  private TalonFXConfiguration getTalonFXConfiguration(){
    TalonFXConfiguration config = new TalonFXConfiguration(); //creates a new talonFX config

    config.FutureProofConfigs = false; //disables future proofing
    config.Audio.AllowMusicDurDisable = false;

    config.MotorOutput.Inverted = constants.isDriveInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive; //sets the inverted value

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.CurrentLimits.SupplyCurrentThreshold = 60;
    config.CurrentLimits.SupplyTimeThreshold = 0.1;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; //sets it to coast (changed when the robot is enabled)

    config.Slot0.kP = constants.driveMotorPID.getP(); //sets the P
    config.Slot0.kI = constants.driveMotorPID.getI(); //sets the I
    config.Slot0.kD = constants.driveMotorPID.getD(); //sets the D
    config.Slot0.kS = constants.driveMotorPID.getF(); //sets the feedForward

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; //just in case sets the built-in sensor
    config.Feedback.SensorToMechanismRatio = 1; //changes the units to m/s

    return config; //returns the new config
  }

  /**
   * use this to config the steer motor at the start of the program
   */
  private CANSparkMax configCanSparkMax(int steerMotorID){
    CANSparkMax sparkMax = new CANSparkMax(steerMotorID, CANSparkLowLevel.MotorType.kBrushless);

    sparkMax.restoreFactoryDefaults();

    sparkMax.enableVoltageCompensation(12); //sets voltage compensation to 12V
    sparkMax.setInverted(constants.isSteerInverted); //sets if the motor is inverted or not

    sparkMax.setIdleMode(CANSparkBase.IdleMode.kCoast); //sets the idle mode to coat (automatically goes to brakes once the robot is enabled)

    sparkMax.getPIDController().setP(constants.steerMotorPID.getP()); //sets the P for the PID Controller
    sparkMax.getPIDController().setI(constants.steerMotorPID.getI()); //sets the I for the PID Controller
    sparkMax.getPIDController().setD(constants.steerMotorPID.getD()); //sets the D for the PID Controller
    sparkMax.getPIDController().setIZone(constants.steerMotorPID.getIZone()); //sets the IZone for the PID Controller

    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, (int)((1 / SwerveModule.moduleThreadHz) * 1000)); //sets the status 0 frame to 10ms
    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, (int)((1 / SwerveModule.moduleThreadHz) * 1000)); //sets the status 0 frame to 10ms
    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, (int)((1 / SwerveModule.moduleThreadHz) * 1000));

    sparkMax.getEncoder().setPositionConversionFactor(1); //sets the gear ratio for the module
    sparkMax.getEncoder().setVelocityConversionFactor(1); //sets the velocity to rad per sec of the module

    sparkMax.getEncoder().setPosition(absEncoder.getPosition().getValueAsDouble() * constants.steerGearRatio); //sets the position of the motor to the absolute encoder

    sparkMax.setSmartCurrentLimit(35); //sets the current limit of the motor (thanks noga for reminding m)
    sparkMax.setSecondaryCurrentLimit(60);
    sparkMax.burnFlash(); //sometimes work

    return sparkMax;
  }
}
