// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter_Intake;

import edu.wpi.first.units.Time;

/** Add your docs here. */
public class ShooterIntakeConstants {
    public enum PresetSpeeds{
        SPEED1(4000),
        SPEED2(3000),
        SPEED3(-2000),
        SPEED4(1000),
        SPEED5(500); //TODO find real speed
      
    
        public final double RPM;
    
        private PresetSpeeds(double RPM){
          this.RPM = RPM;
        }
      }
    public enum PresetsTimeSec
    {
        TimeShooter(5);

        public final double Sec;

        private PresetsTimeSec (double Sec)
        {
            this.Sec=Sec;
        }

      
    }
}
