package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable limelight;

    public LimelightSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getTargetY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double[] getTargetPose() {
        if (!hasTarget()) {
            return null;
        }
        
        double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[0]);
        
        if (botpose.length < 6) {
            return null;
        }
        
        return new double[] {
            botpose[0],  // X (meters)
            botpose[1],  // Y (meters, positive left)
            botpose[5]   // Yaw (degrees, CCW positive)
        };
    }

    public void setLedMode(int mode) {
        limelight.getEntry("ledMode").setNumber(mode);
    }

    public void setPipeline(int pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline);
    }
}