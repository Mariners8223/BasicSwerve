package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;

import static frc.robot.Constants.robotType;

public abstract class SwerveModuleIO implements Runnable{
    @AutoLog
    static class SwerveModuleIOInputs {
        public SwerveModuleState currentState = new SwerveModuleState();

        public double steerVelocityRadPerSec = 0.0;
        public double drivePositionMeters = 0.0;

        public double steerMotorAppliedVoltage = 0.0;
        public double driveMotorAppliedVoltage = 0.0;

        public double absEncoderPosition = 0.0;
    }

    /**
     * the constants of the module (depends on if it's a devbot or a compbot)
     */
    protected final SwerveModuleConstants constants =
            robotType == Constants.RobotType.DEVELOPMENT ? SwerveModuleConstants.DEVBOT : SwerveModuleConstants.COMPBOT;

    private double driveMotorVoltageInput = 0;
    private double steerMotorVoltageInput = 0;

    /**
     * Updates the inputs of the module
     */
    abstract void updateInputs(SwerveModuleIOInputsAutoLogged inputs);

    /**
     * sets the voltage input for the drive motor
     *
     * @param voltage the voltage to set the drive motor to
     */
    public void setDriveMotorVoltage(double voltage){
        this.driveMotorVoltageInput = voltage;
    }

    /**
     * sets the voltage input for the steer motor
     *
     * @param voltage the voltage to set the steer motor to
     */
    public void setSteerMotorVoltage(double voltage){
        this.steerMotorVoltageInput = voltage;
    }

    /**
     * sends the inputs set by {@link #setDriveMotorVoltage driveMotor} and {@link #setSteerMotorVoltage steerMotor} to the motors
     */
    @Override
    public void run() {
        sendInputsToMotors(driveMotorVoltageInput, steerMotorVoltageInput);
    }

    abstract protected void sendInputsToMotors(double driveMotorVoltage, double steerMotorVoltage);

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

    /**
     * configures the absolute encoder (duty cycle encoder)
     * @param absEncoderID the port of the abs encoder on the rio
     * @param zeroOffset the zero offset of the abs encoder
     * @return the configured abs encoder
     */
    protected DutyCycleEncoder configDutyCycleEncoder(int absEncoderID, double zeroOffset) {
        DutyCycleEncoder encoder = new DutyCycleEncoder(absEncoderID);
        encoder.reset();
        encoder.setPositionOffset(zeroOffset);

        return encoder;
    }

    /**
     * configures the absolute encoder (CANCoder)
     * @param absEncoderID the port of the abs encoder on the CAN bus
     * @param absoluteEncoderZeroOffset the zero offset of the abs encoder
     * @return the configured CANCoder
     */
    protected CANcoder configCANCoder(int absEncoderID, double absoluteEncoderZeroOffset) {
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

    /**
     * configures the TalonFX
     * @param config the config to apply to the talon fx
     * @param driveMotorID the CAN id of the drive motor
     * @return the configured talon fx
     */
    protected TalonFX configTalonFX(TalonFXConfiguration config, int driveMotorID) {
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
     * configures the TalonFX to swerve settings
     * @return the configured talon fx
     */
    protected TalonFXConfiguration getTalonFXConfiguration(SwerveModule.ModuleName name) {
        TalonFXConfiguration config = new TalonFXConfiguration(); //creates a new talonFX config

        config.FutureProofConfigs = false; //disables future proofing
        config.Audio.AllowMusicDurDisable = false;

        config.MotorOutput.Inverted = constants.isDriveInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive; //sets the inverted value

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentThreshold = 60;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; //sets it to coast (changed when the robot is enabled)

        config.Slot0.kP = constants.driveMotorPID[name.ordinal()].getP(); //sets the P
        config.Slot0.kI = constants.driveMotorPID[name.ordinal()].getI(); //sets the I
        config.Slot0.kD = constants.driveMotorPID[name.ordinal()].getD(); //sets the D
        config.Slot0.kS = constants.driveMotorPID[name.ordinal()].getF(); //sets the feedForward

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; //just in case sets the built-in sensor
        config.Feedback.SensorToMechanismRatio = 1; //changes the units to m/s

        return config; //returns the new config
    }

    /**
     * configures the steer motor to swerve settings
     * @param steerMotorID the CAN id of the steer motor
     * @param absEncoderPosition the position of the absolute encoder (in rotations of the encoder)
     * @return the configured steer motor
     */
    protected CANSparkMax configCanSparkMax(int steerMotorID, double absEncoderPosition, SwerveModule.ModuleName name) {
        CANSparkMax sparkMax = new CANSparkMax(steerMotorID, CANSparkLowLevel.MotorType.kBrushless);

        sparkMax.restoreFactoryDefaults();

        sparkMax.enableVoltageCompensation(12); //sets voltage compensation to 12V
        sparkMax.setInverted(constants.isSteerInverted); //sets if the motor is inverted or not

        sparkMax.setIdleMode(CANSparkBase.IdleMode.kCoast); //sets the idle mode to coat (automatically goes to brakes once the robot is enabled)

        sparkMax.getPIDController().setP(constants.steerMotorPID[name.ordinal()].getP()); //sets the P for the PID Controller
        sparkMax.getPIDController().setI(constants.steerMotorPID[name.ordinal()].getI()); //sets the I for the PID Controller
        sparkMax.getPIDController().setD(constants.steerMotorPID[name.ordinal()].getD()); //sets the D for the PID Controller
        sparkMax.getPIDController().setIZone(constants.steerMotorPID[name.ordinal()].getIZone()); //sets the IZone for the PID Controller

        sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, (int) (1000 / SwerveModule.moduleThreadHz)); //sets the status 0 frame to 10ms
        sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, (int) (1000 / SwerveModule.moduleThreadHz)); //sets the status 0 frame to 10ms
        sparkMax.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, (int) (1000 / SwerveModule.moduleThreadHz));

        sparkMax.getEncoder().setPositionConversionFactor(1); //sets the gear ratio for the module
        sparkMax.getEncoder().setVelocityConversionFactor(1); //sets the velocity to rad per sec of the module

        sparkMax.getEncoder().setPosition(absEncoderPosition * constants.steerGearRatio); //sets the position of the motor to the absolute encoder

        sparkMax.setSmartCurrentLimit(35); //sets the current limit of the motor (thanks noga for reminding m)
        sparkMax.setSecondaryCurrentLimit(60);
        sparkMax.burnFlash(); //sometimes work

        return sparkMax;
    }
}
