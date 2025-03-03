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
        // Check if there's at least one target
        return limelight.getEntry("tv").getDouble(0) >= 1;
    }

    public double getTargetX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getTargetY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double[] getTargetPose() {
        // Remove the hasTarget() check here - rely on botpose validity
        double[] botpose = limelight.getEntry("botpose").getDoubleArray(new double[0]);
        
        // Require full 6-element pose data
        if (botpose.length < 6) return null;
        
        return new double[] {
            botpose[0],  // X (meters)
            botpose[1],  // Y (meters)
            botpose[5]   // Yaw (degrees)
        };
    }

    public void setLedMode(int mode) {
        limelight.getEntry("ledMode").setNumber(mode);
    }

    public void setPipeline(int pipeline) {
        System.out.println("Setting pipeline to " + pipeline + " at " + System.currentTimeMillis());
        limelight.getEntry("pipeline").setNumber(pipeline);
    }

    public double getCurrentPipeline() {
        double pipe = limelight.getEntry("getpipe").getDouble(-1);
        System.out.println("Current pipeline: " + pipe + " at " + System.currentTimeMillis());
        return pipe;
    }
}