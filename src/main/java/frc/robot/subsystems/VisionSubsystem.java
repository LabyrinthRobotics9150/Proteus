package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelight;

    public VisionSubsystem() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        setPipeline(0);
    }

    public void setPipeline(int pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline);
    }

    public double getTx() {
        return limelight.getEntry("tx").getDouble(0.0);
    }
    
    // Added method to get vertical offset (ty) if needed for future distance control
    public double getTy() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1;
    }
}
