package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

/**
 * A simple extension of PIDController that allows configurable tolerance.
 */
class PIDControllerConfigurable extends PIDController {
    public PIDControllerConfigurable(double kP, double kI, double kD) {
        super(kP, kI, kD);
    }
    
    public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
        super(kP, kI, kD);
        this.setTolerance(tolerance);
    }
}

/**
 * AutoAlignCommand now performs alignment in three sequential stages:
 * 1. ALIGN_Y: Correct lateral (left/right) error.
 * 2. ALIGN_ROTATION: Correct the robot's heading.
 * 3. DRIVE_X: Approach or back away to the designated distance.
 */
public class AutoAlignCommand extends Command {
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final VisionSubsystem m_Limelight;
    
    // PID controllers with updated gains for smoother, faster response.
    private static PIDControllerConfigurable rotationalPidController = 
        new PIDControllerConfigurable(0.15, 0.0, 0.005, 1.0);
    private static final PIDControllerConfigurable xPidController = 
        new PIDControllerConfigurable(0.6, 0.0, 0.001, 0.05);
    private static final PIDControllerConfigurable yPidController = 
        new PIDControllerConfigurable(0.6, 0.0, 0.001, 0.05);
    
    // Rotation error threshold (in degrees) for “good enough” heading.
    private static final double ROTATION_ERROR_THRESHOLD_DEGREES = 1.0;
    
    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    private static int tagID = -1;
    private double yoffset;
    
    // Allowed AprilTag IDs for auto-alignment.
    private static final int[] ALLOWED_TAG_IDS = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
    
    // State machine for alignment. The new order is:
    // 1. ALIGN_Y: Correct lateral offset.
    // 2. ALIGN_ROTATION: Correct heading.
    // 3. DRIVE_X: Drive to the desired distance.
    private enum AlignStage {
        ALIGN_Y,
        ALIGN_ROTATION,
        DRIVE_X
    }
    private AlignStage currentStage;
    
    // Constructor for central alignment.
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        yoffset = 0;
        addRequirements(m_Limelight);
    }
    
    // Constructor for left/right alignment (adjusts lateral setpoint via yoffset).
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, boolean rightAlign) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        addRequirements(m_Limelight);
        // For right alignment, set a small positive offset; for left alignment, a small negative offset.
        yoffset = rightAlign ? 0.01 : -0.01;
    }
    
    @Override
    public void initialize() {
        rotationalPidController.reset();
        xPidController.reset();
        yPidController.reset();
        try {
            tagID = m_Limelight.getClosestFiducial().id;
        } catch (VisionSubsystem.NoSuchTargetException e) {
            tagID = -1;
        }
        // Start with lateral alignment.
        currentStage = AlignStage.ALIGN_Y;
    }
    
    @Override
    public void execute() {
        RawFiducial fiducial;
        try {
            if (tagID != -1) {
                fiducial = m_Limelight.getFiducialWithId(tagID);
            } else {
                fiducial = m_Limelight.getClosestFiducial();
                tagID = fiducial.id;
            }
            boolean allowed = false;
            for (int id : ALLOWED_TAG_IDS) {
                if (fiducial.id == id) {
                    allowed = true;
                    break;
                }
            }
            if (!allowed) {
                m_drivetrain.setControl(idleRequest);
                return;
            }
        } catch (VisionSubsystem.NoSuchTargetException e) {
            m_drivetrain.setControl(idleRequest);
            return;
        }
        
        double outputX = 0.0;
        double outputY = 0.0;
        double outputRotation = 0.0;
        
        switch (currentStage) {
            case ALIGN_Y: {
                // Lateral (Y) alignment: Calculate lateral error.
                // Using distance * sin(angle) approximates the lateral offset.
                double lateralError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
                if (Math.abs(lateralError - yoffset) < 0.05) { // within 5cm tolerance
                    outputY = 0.0;
                    currentStage = AlignStage.ALIGN_ROTATION;
                } else {
                    outputY = yPidController.calculate(lateralError, yoffset);
                }
                break;
            }
            case ALIGN_ROTATION: {
                // Rotation alignment: Correct the robot's heading using the horizontal error.
                double rotationError = fiducial.txnc; // in degrees
                if (Math.abs(rotationError) < ROTATION_ERROR_THRESHOLD_DEGREES) {
                    outputRotation = 0.0;
                    currentStage = AlignStage.DRIVE_X;
                } else {
                    outputRotation = rotationalPidController.calculate(rotationError, 0.0);
                    double maxRotationRate = 0.5; // Limit maximum rotational rate
                    outputRotation = Math.max(-maxRotationRate, Math.min(maxRotationRate, outputRotation));
                }
                break;
            }
            case DRIVE_X: {
                // Drive forward/backward to achieve the desired distance.
                double desiredDistance = 0.6;
                if (Math.abs(fiducial.distToRobot - desiredDistance) < 0.05) { // within 5cm tolerance
                    outputX = 0.0;
                } else {
                    // Invert the output so that positive controller output moves the robot toward the target.
                    outputX = -xPidController.calculate(fiducial.distToRobot, desiredDistance);
                }
                // If the rotation error grows again, revert to rotational alignment.
                if (Math.abs(fiducial.txnc) > ROTATION_ERROR_THRESHOLD_DEGREES) {
                    currentStage = AlignStage.ALIGN_ROTATION;
                }
                break;
            }
        }
        
        // Send the computed control outputs to the drivetrain.
        m_drivetrain.setControl(
            alignRequest
                .withRotationalRate(outputRotation)
                .withVelocityX(outputX)
                .withVelocityY(outputY)
        );
        
        // Publish status information for debugging.
        SmartDashboard.putString("Align Stage", currentStage.name());
        SmartDashboard.putNumber("txnc", fiducial.txnc);
        SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
        SmartDashboard.putNumber("rotationalRate", outputRotation);
        SmartDashboard.putNumber("velocityX", outputX);
        SmartDashboard.putNumber("velocityY", outputY);
    }
    
    @Override
    public boolean isFinished() {
        // The command finishes when all controllers report they are at their setpoints.
        return xPidController.atSetpoint() && rotationalPidController.atSetpoint() && yPidController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(idleRequest);
    }
}
