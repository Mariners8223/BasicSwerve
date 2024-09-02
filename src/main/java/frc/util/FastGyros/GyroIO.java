package frc.util.FastGyros;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public interface GyroIO extends Sendable {
    double getAngleDegrees();

    Rotation2d getRotation2d();

    void update();

    double getYaw();

    double getPitch();

    double getRoll();

    double getVelocityX();

    double getVelocityY();

    double getAccelerationX();

    double getAccelerationY();

    void reset(Pose2d var1);

    void initSendable(SendableBuilder var1);

    public static final class DefaultImpls {
        // $FF: synthetic method
        public static void reset$default(GyroIO var0, Pose2d var1, int var2, Object var3) {
            if (var3 != null) {
                throw new UnsupportedOperationException("Super calls with default arguments not supported in this target, function: reset");
            } else {
                if ((var2 & 1) != 0) {
                    var1 = new Pose2d();
                }

                var0.reset(var1);
            }
        }
    }
}
