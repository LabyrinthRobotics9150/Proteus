package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

/**
 * AutoAlignCommand that uses only 2D data from the Limelight.
 * It aligns the robot by first correcting rotation (using tx) and then driving
 * to a desired distance (estimated via the target area ta).
 */
public class AutoAlignCommand extends Command {
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final VisionSubsystem m_Limelight;
    
    // A simple PIDController subclass that allows setting tolerance.
    private static class PIDControllerConfigurable extends PIDController {
        public PIDControllerConfigurable(double kP, double kI, double kD) {
            super(kP, kI, kD);
        }
        public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
            super(kP, kI, kD);
            this.setTolerance(tolerance);
        }
    }
    
    // PID controllers (tuned for your robot) – note these values may need adjustment.
    private static PIDControllerConfigurable rotationalPidController =
        new PIDControllerConfigurable(0.15, 0.0, 0.005, 1.0);
    private static final PIDControllerConfigurable xPidController =
        new PIDControllerConfigurable(0.6, 0.0, 0.001, 0.05);
    private static final PIDControllerConfigurable yPidController =
        new PIDControllerConfigurable(0.6, 0.0, 0.001, 0.05);
    
    // Acceptable rotation error (in degrees) when the target is centered.
    private static final double ROTATION_ERROR_THRESHOLD_DEGREES = 1.0;
    
    // The SwerveRequest objects used to send commands to the drivetrain.
    private static final SwerveRequest.RobotCentric alignRequest =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    
    // For lateral (side-to-side) alignment, we allow an offset (in meters).
    // For central alignment, yoffset is zero.
    private double yoffset;
    
    // The alignment is done in two stages:
    // 1. ALIGN_ROTATION: Rotate the robot until the horizontal error (tx) is near zero.
    // 2. DRIVE_X: Drive forward/backward (with optional lateral adjustment) until the target is at the desired distance.
    private enum AlignStage {
        ALIGN_ROTATION,
        DRIVE_X
    }
    private AlignStage currentStage;
    
    // The desired distance (in meters) from the target.
    private static final double DESIRED_DISTANCE = 0.5;
    // A constant to convert the target area (ta) to an estimated distance.
    // (This is a simple heuristic – tune as necessary.)
    private static final double DISTANCE_CONSTANT = 1.0;
    
    /**
     * Constructor for central alignment.
     */
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        this.yoffset = 0.0;
        addRequirements(m_Limelight);
    }
    
    /**
     * Constructor that accepts a rightAlign flag to offset laterally.
     * For right alignment, a small negative offset is used; for left, a positive offset.
     */
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, boolean rightAlign) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        addRequirements(m_Limelight);
        this.yoffset = rightAlign ? -0.75 : 0.75;
    }
    
    @Override
    public void initialize() {
        rotationalPidController.reset();
        xPidController.reset();
        yPidController.reset();
        currentStage = AlignStage.ALIGN_ROTATION;
    }
    
    @Override
    public void execute() {
        // Retrieve 2D vision metrics.
        double tx = m_Limelight.getClosestTX();  // Horizontal offset (in degrees)
        double ta = m_Limelight.getClosestTA();  // Target area (used to estimate distance)
        
        // If no target is detected, stop the drivetrain.
        if (ta <= 0) {
            m_drivetrain.setControl(idleRequest);
            return;
        }
        
        // Estimate the distance to the target from the target area.
        // (This is a simple heuristic; in practice, you might use camera mounting parameters.)
        double estimatedDistance = DISTANCE_CONSTANT / Math.sqrt(ta);
        
        double outputX = 0.0;
        double outputY = 0.0;
        double outputRotation = 0.0;
        
        switch (currentStage) {
            case ALIGN_ROTATION:
                // Rotate the robot until the horizontal error (tx) is within a small threshold.
                double rotationError = tx;
                if (Math.abs(rotationError) < ROTATION_ERROR_THRESHOLD_DEGREES) {
                    outputRotation = 0.0;
                    currentStage = AlignStage.DRIVE_X;
                } else {
                    outputRotation = rotationalPidController.calculate(rotationError, 0.0);
                    double maxRotationRate = 0.5; // Limit the rotational speed
                    outputRotation = Math.max(-maxRotationRate, Math.min(maxRotationRate, outputRotation));
                }
                break;
            case DRIVE_X:
                // Use the estimated distance to drive forward/backward.
                // Optionally, correct lateral error using the horizontal offset (tx).
                double lateralError = estimatedDistance * Math.sin(Math.toRadians(tx));
                if (Math.abs(lateralError - yoffset) > 0.2) { // tolerance of ~20cm
                    outputY = yPidController.calculate(lateralError, yoffset);
                }
                if (Math.abs(estimatedDistance - DESIRED_DISTANCE) > 0.05) { // tolerance of ~5cm
                    outputX = -xPidController.calculate(estimatedDistance, DESIRED_DISTANCE);
                }
                // If the horizontal error increases, return to rotation alignment.
                if (Math.abs(tx) > ROTATION_ERROR_THRESHOLD_DEGREES) {
                    currentStage = AlignStage.ALIGN_ROTATION;
                }
                break;
        }
        
        m_drivetrain.setControl(
            alignRequest.withRotationalRate(outputRotation)
                        .withVelocityX(outputX)
                        .withVelocityY(outputY)
        );
        
        // Publish debug information.
        SmartDashboard.putString("Align Stage", currentStage.name());
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("Estimated Distance", estimatedDistance);
        SmartDashboard.putNumber("Lateral Error", estimatedDistance * Math.sin(Math.toRadians(tx)));
        SmartDashboard.putNumber("Output Rotation", outputRotation);
        SmartDashboard.putNumber("Output X", outputX);
        SmartDashboard.putNumber("Output Y", outputY);
    }
    
    @Override
    public boolean isFinished() {
        // Finish when all controllers have reached their setpoints.
        return rotationalPidController.atSetpoint() &&
               xPidController.atSetpoint() &&
               yPidController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(idleRequest);
    }
}
