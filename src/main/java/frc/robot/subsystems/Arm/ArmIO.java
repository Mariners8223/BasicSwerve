package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkLowLevel;

public interface ArmIO {
    @AutoLog
    public class ArmInputs{
        public double motorAlphaPosition;
        public double absAlphaEncoderPosition;

        public double motorBetaPosition;
        public boolean betaLimitSwitch;
    }

    CANSparkMax  alphaMotor= new CANSparkMax(ArmConstants.alphaMotorID, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax  betaMotor= new CANSparkMax(ArmConstants.betaMotorID, CANSparkLowLevel.MotorType.kBrushless);
    SparkAbsoluteEncoder encoder_Abs = alphaMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    RelativeEncoder encoder_Relative = alphaMotor.getAlternateEncoder(ArmConstants.ccountsPerRev);
    DigitalInput limitSwitch = new DigitalInput(ArmConstants.limitSwitchPort);


    public void setAlphaTargetRotation(double AlphaTarget);
    public void setBetaTargetRotation(double BetaTarget);
    public void resetBetaEncoder();

    public void update(ArmInputsAutoLogged inputs);
}
