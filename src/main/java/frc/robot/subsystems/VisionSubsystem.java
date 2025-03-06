package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LimelightHelpers.*;

public class VisionSubsystem extends SubsystemBase {
    private RawFiducial[] fiducials;
    private final String limelightName = VisionConstants.LIMELIGHT_NAME;

    public VisionSubsystem() {
        config();
    }

    public static class NoSuchTargetException extends RuntimeException {
        public NoSuchTargetException(String message) { super(message); }
    }

    public void config() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            VisionConstants.CAMERA_X_OFFSET,
            VisionConstants.CAMERA_Y_OFFSET,
            VisionConstants.CAMERA_Z_OFFSET,
            VisionConstants.CAMERA_ROLL,
            VisionConstants.CAMERA_PITCH,
            VisionConstants.CAMERA_YAW
        );
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, VisionConstants.ALLOWED_TAG_IDS);
    }

    @Override
    public void periodic() {
        fiducials = LimelightHelpers.getRawFiducials(limelightName);
        logDiagnostics();
    }

    private void logDiagnostics() {
        boolean connected = LimelightHelpers.getTV(limelightName);
        SmartDashboard.putBoolean("Limelight/Connected", connected);
        SmartDashboard.putNumber("Limelight/VisibleTags", fiducials != null ? fiducials.length : 0);
    }

    public RawFiducial getClosestFiducial() {
        validateFiducials();
        
        RawFiducial closest = fiducials[0];
        double minDistance = closest.distToRobot;
        
        for (RawFiducial fiducial : fiducials) {
            if (fiducial.distToRobot < minDistance) {
                closest = fiducial;
                minDistance = fiducial.distToRobot;
            }
        }
        return closest;
    }

    public RawFiducial getFiducialWithId(int id) {
        validateFiducials();
        
        for (RawFiducial fiducial : fiducials) {
            if (fiducial.id == id) return fiducial;
        }
        throw new NoSuchTargetException("Tag ID " + id + " not found");
    }

    private void validateFiducials() {
        if (fiducials == null || fiducials.length == 0) {
            throw new NoSuchTargetException("No AprilTags detected");
        }
    }

    // Simplified getters
    public double getTX() { return LimelightHelpers.getTX(limelightName); }
    public double getTY() { return LimelightHelpers.getTY(limelightName); }
    public double getTA() { return LimelightHelpers.getTA(limelightName); }
    public boolean getTV() { return LimelightHelpers.getTV(limelightName); }
}