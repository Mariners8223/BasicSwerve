// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmCoanstants.*;


public class Arm extends SubsystemBase {
  /** Creates a new arm. */
  ArmIO io;

  public void MoveAlpha(double AlphaTarget){
    io.setAlphaTargetRotation(AlphaTarget);
  }

  public void MoveBeta(double BetaTarget){
    io.setBetaTargetRotation(BetaTarget);
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
}
