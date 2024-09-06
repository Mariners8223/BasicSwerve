package frc.robot.subsystems.Arm;

public class ArmCoanstants {
    public enum ArmPos{
        HomePos(0.0),
        CollectFloorPos_Alpha(0.0),
        CollectFloorPos_Beta(0.0),  
        CollectSourcePos(0.0),
        ShootSubwooferPos(0.0),
        freePos(0.0);

        private double pos;
        ArmPos(double pos){
            this.pos = pos;
        }

        public double getPosValue(){
            return this.pos;
        }

    }
    
}
