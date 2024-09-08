// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmConstants.*;
import frc.robot.subsystems.Arm.ArmConstants.ArmPos;


public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  ArmIO io;
  ArmInputsAutoLogged inputs;

  public void MoveAlpha(double AlphaTarget){
    io.setAlphaTargetRotation(AlphaTarget);
  }

  public void MoveBeta(double BetaTarget){
    io.setBetaTargetRotation(BetaTarget);
  }

  public double GetAlphaPos() { return inputs.motorAlphaPosition; }
  public double GetBetaPos() { return inputs.motorBetaPosition; }

  public ArmPos getCurrentPos(){
    double alpha = GetAlphaPos();
    double beta = GetBetaPos();

    ArmPos[] positions = ArmPos.values();

    for (ArmPos armPos : positions) {
      if(Math.abs(alpha - armPos.getAlpha()) < ArmConstants.ARM_POSITION_TOLERANCE &&
      Math.abs(beta - armPos.getBeta()) < ArmConstants.ARM_POSITION_TOLERANCE)
      return armPos;
    }

 
    return ArmPos.UNKNOWN;
  }
  
  // public void TargetArmPos_Alpha(ArmPos armPos){ //souldn't work 
  //   double currentAlphaPos = io.GetCurrentAlphaPos();
  //   // double currentBetaPos = io.GetCurrentBetaPos();
  //   double wantedPos = armPos.getPosValue();
  //   // boolean workWithAlpha = (wanttedPos != ArmPos.CollectFloorPos_Beta) ? true : false;
  //   if(currentAlphaPos != wantedPos){
  //     if(currentAlphaPos == ArmPos.CollectFloorPos_Alpha.getPosValue()){
  //       //here sould be a sequence
  //     }
  //     else if(wantedPos == ArmPos.CollectFloorPos_Alpha.getPosValue()){
  //       //here sould be a sequence
  //     }
  //   }
  // }

  @Override
  public void periodic(){
    io.update(inputs);
  }
}
