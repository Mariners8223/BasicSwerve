package frc.robot.subsystems.DriveTrain.SwerveModules;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.MotorMap;
import frc.util.PIDFGains;

public class SwerveModuleIOCompBot implements SwerveModuleIO{
  public static class CompBotConstants{
    public static final double driveGearRatio =  6.75;
    public static final double steerGearRatio = 12.5 * 3;

    public static final double wheelRadiusMeters = 0.0508;
    public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters;

    public static final double maxDriveVelocityMetersPerSecond = 2;

    public static final boolean isDriveInverted = false;
    public static final boolean isSteerInverted = false;
    public static final boolean isAbsEncoderInverted = false;

    public static final double front_left_absoluteEncoderZeroOffset = 214.65;//212.8; // the offset between the absolute encoder reading on the front left module, in degrees
    public static final double front_right_absoluteEncoderZeroOffset = 337.27;//253.1; // the offset between the absolute encoder on the front left module, in degrees
    public static final double back_left_absoluteEncoderZeroOffset = 81.47;//320.2; // the offset between the absolute encoder on the back left module, in degrees
    public static final double back_right_absoluteEncoderZeroOffset = 270.1;//205;

    public static final double front_left_zeroOffset = 0.59625; // the offset between the absolute encoder reading on the front left module, in degrees
    public static final double front_right_zeroOffset = 0.9368; // the offset between the absolute encoder on the front left module, in degrees
    public static final double back_left_zeroOffset = 0.226; // the offset between the absolute encoder on the back left module, in degrees
    public static final double back_right_zeroOffset = 0.750; // the offset between the absolute encoder on the back right module, in degrees

    public static final PIDFGains driveMotorPID = new PIDFGains(2.89, 0.00, 0, 1.2, 0.1, 0, 1 / SwerveModule.moduleThreadHz, 3, 100);
    public static final PIDFGains steerMotorPID = new PIDFGains(10, 0, 0.5, 0, 0.1, 0, 1 / SwerveModule.moduleThreadHz);
  }

  private final TalonFX driveMotor;
  private final CANSparkMax steerMotor;
  private final DutyCycleEncoder absEncoder;
  private final int absEncoderMultiplier = CompBotConstants.isAbsEncoderInverted ? -1 : 1;


  public SwerveModuleIOCompBot(SwerveModule.ModuleName name){
    int driveMotorID = MotorMap.DriveBase.modules[name.ordinal()][0];
    int steerMotorID = MotorMap.DriveBase.modules[name.ordinal()][1];
    int absEncoderID = MotorMap.DriveBase.modules[name.ordinal()][2];

    double zeroOffset = switch (name){
      case Front_Left -> CompBotConstants.front_left_zeroOffset;
      case Front_Right -> CompBotConstants.front_right_zeroOffset;
      case Back_Left -> CompBotConstants.back_left_zeroOffset;
      case Back_Right -> CompBotConstants.back_right_zeroOffset;
    };

    absEncoder = configDutyCycleEncoder(absEncoderID, zeroOffset);

    driveMotor = configTalonFX(getTalonFXConfiguration(), driveMotorID);
    steerMotor = configCanSparkMax(steerMotorID);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputsAutoLogged inputs) {
    steerMotor.getEncoder().setPosition(absEncoder.get() * CompBotConstants.steerGearRatio * absEncoderMultiplier); //fix?

    inputs.currentState.angle = Rotation2d.fromRotations(steerMotor.getEncoder().getPosition() / CompBotConstants.steerGearRatio);
    inputs.currentState.speedMetersPerSecond = (driveMotor.getVelocity().getValueAsDouble() / CompBotConstants.driveGearRatio) * CompBotConstants.wheelCircumferenceMeters;

    inputs.absEncoderPosition = (absEncoder.getAbsolutePosition() - absEncoder.getPositionOffset()) * absEncoderMultiplier;

    inputs.steerVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(steerMotor.getEncoder().getVelocity() / CompBotConstants.steerGearRatio);
    inputs.drivePositionMeters = (driveMotor.getPosition().getValueAsDouble() / CompBotConstants.driveGearRatio) * CompBotConstants.wheelCircumferenceMeters;

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

  private DutyCycleEncoder configDutyCycleEncoder(int absEncoderID, double zeroOffset){
    DutyCycleEncoder encoder = new DutyCycleEncoder(absEncoderID);
    encoder.reset();
    encoder.setPositionOffset(zeroOffset);

    return encoder;
  }

  private TalonFX configTalonFX(TalonFXConfiguration config, int driveMotorID){
    TalonFX talonFX = new TalonFX(driveMotorID); //creates a new talon fx

    talonFX.getConfigurator().apply(config); //apply the given config

    talonFX.getPosition().setUpdateFrequency(SwerveModule.moduleThreadHz); //position is needed more for destroy

    talonFX.getVelocity().setUpdateFrequency(SwerveModule.moduleThreadHz); //sets as default
    talonFX.getMotorVoltage().setUpdateFrequency(SwerveModule.moduleThreadHz); //sets as default
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

    config.MotorOutput.Inverted = CompBotConstants.isDriveInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive; //sets the inverted value

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.CurrentLimits.SupplyCurrentThreshold = 60;
    config.CurrentLimits.SupplyTimeThreshold = 0.1;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; //sets it to coast (changed when the robot is enabled)

    config.Slot0.kP = CompBotConstants.driveMotorPID.getP(); //sets the P
    config.Slot0.kI = CompBotConstants.driveMotorPID.getI(); //sets the I
    config.Slot0.kD = CompBotConstants.driveMotorPID.getD(); //sets the D
    config.Slot0.kS = CompBotConstants.driveMotorPID.getF(); //sets the feedForward

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
    sparkMax.setInverted(CompBotConstants.isSteerInverted); //sets if the motor is inverted or not

    sparkMax.setIdleMode(CANSparkBase.IdleMode.kCoast); //sets the idle mode to coat (automatically goes to brakes once the robot is enabled)

    sparkMax.getPIDController().setP(CompBotConstants.steerMotorPID.getP()); //sets the P for the PID Controller
    sparkMax.getPIDController().setI(CompBotConstants.steerMotorPID.getI()); //sets the I for the PID Controller
    sparkMax.getPIDController().setD(CompBotConstants.steerMotorPID.getD()); //sets the D for the PID Controller
    sparkMax.getPIDController().setIZone(CompBotConstants.steerMotorPID.getIZone()); //sets the IZone for the PID Controller

    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, (int)((1 / SwerveModule.moduleThreadHz) * 1000)); //sets the status 0 frame to 10ms
    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, (int)((1 / SwerveModule.moduleThreadHz) * 1000)); //sets the status 0 frame to 10ms
    sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, (int)((1 / SwerveModule.moduleThreadHz) * 1000));

    sparkMax.getEncoder().setPositionConversionFactor(1); //sets the gear ratio for the module

    sparkMax.getEncoder().setPosition(absEncoder.get() * CompBotConstants.steerGearRatio * absEncoderMultiplier); //sets the position of the motor to the absolute encoder

    sparkMax.setSmartCurrentLimit(35); //sets the current limit of the motor (thanks noga for reminding m)
    sparkMax.setSecondaryCurrentLimit(60);
    sparkMax.burnFlash(); //sometimes work

    return sparkMax;
  }
}
