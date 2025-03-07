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
 * Revised AutoAlignCommand now performs alignment in three stages:
 * 
 * 1. ALIGN_Y: Compute the lateral (Y) correction once (using radians internally).
 * 2. ALIGN_ROTATION: Compute the rotation correction once (using radians internally).
 * 3. DRIVE_X: Continuously drive forward/backward until the robot reaches the desired distance.
 * 
 * After the first two stages the computed outputs are “fixed” (i.e. not updated), so the command
 * only calculates the rotation and lateral corrections one time.
 */
public class AutoAlignCommand extends Command {
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final VisionSubsystem m_Limelight;
    
    // New (faster) PID gains and tolerances—all angle calculations are done in radians now.
    private static PIDControllerConfigurable rotationalPidController = 
        new PIDControllerConfigurable(0.3, 0.0, 0.01, Math.toRadians(1.0));
    private static final PIDControllerConfigurable xPidController = 
        new PIDControllerConfigurable(1.2, 0.0, 0.002, 0.05);
    private static final PIDControllerConfigurable yPidController = 
        new PIDControllerConfigurable(1.2, 0.0, 0.002, 0.05);
    
    // Using a robot-centric control request.
    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    
    // Keep track of the chosen AprilTag (if available)
    private static int tagID = -1;
    // A lateral offset (in meters) that can be set via the alternate constructor.
    private double yoffset;
    
    // Allowed AprilTag IDs for auto-alignment.
    private static final int[] ALLOWED_TAG_IDS = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
    
    // We now use a simple three‐stage state machine.
    private enum AlignStage {
        ALIGN_Y,
        ALIGN_ROTATION,
        DRIVE_X
    }
    private AlignStage currentStage;
    
    // These fields will store the one‐shot outputs.
    private Double fixedRotationOutput = null;
    private Double fixedLateralOutput = null;
    
    // Constructor for central alignment.
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        yoffset = 0;
        addRequirements(m_Limelight);
    }
    
    // Constructor for left/right alignment (sets a lateral offset).
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, boolean rightAlign) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        addRequirements(m_Limelight);
        // For right alignment, use a small positive offset; for left alignment, a small negative offset.
        yoffset = rightAlign ? -0.75 : 0.75;
    }
    
    @Override
    public void initialize() {
        rotationalPidController.reset();
        xPidController.reset();
        yPidController.reset();
        fixedRotationOutput = null;
        fixedLateralOutput = null;
        try {
            // Choose the closest fiducial as the target.
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
                // Compute lateral error:
                // Using the target’s horizontal angle (txnc) converted to radians, then
                // lateral error = distance * sin(angle).
                double lateralError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
                // Calculate the lateral output only once.
                if (fixedLateralOutput == null) {
                    fixedLateralOutput = yPidController.calculate(lateralError, yoffset);
                }
                outputY = fixedLateralOutput;
                // When the error is within tolerance, lock in the lateral correction.
                if (Math.abs(lateralError - yoffset) < 0.2) { // 20 cm tolerance
                    fixedLateralOutput = 0.0;
                    currentStage = AlignStage.ALIGN_ROTATION;
                }
                break;
            }
            case ALIGN_ROTATION: {
                // Compute the rotation error in radians.
                double rotationErrorRad = Units.degreesToRadians(fiducial.txnc);
                // Calculate the rotation output only once.
                if (fixedRotationOutput == null) {
                    fixedRotationOutput = rotationalPidController.calculate(rotationErrorRad, 0.0);
                }
                outputRotation = fixedRotationOutput;
                // Once the rotation error is within the (radian) tolerance, lock it in.
                if (Math.abs(rotationErrorRad) < Math.toRadians(1.0)) {
                    fixedRotationOutput = 0.0;
                    currentStage = AlignStage.DRIVE_X;
                }
                break;
            }
            case DRIVE_X: {
                // Drive forward/backward until the robot is at the desired distance.
                double desiredDistance = 0.5; // meters
                outputX = -xPidController.calculate(fiducial.distToRobot, desiredDistance);
                // Do not revert to earlier stages even if txnc changes.
                break;
            }
        }
        
        // Send the computed (and fixed, when appropriate) control outputs.
        m_drivetrain.setControl(
            alignRequest
                .withRotationalRate(outputRotation)
                .withVelocityX(outputX)
                .withVelocityY(outputY)
        );
        
        // For debugging: publish the current stage and errors.
        SmartDashboard.putString("Align Stage", currentStage.name());
        SmartDashboard.putNumber("txnc (deg)", fiducial.txnc);
        SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
        SmartDashboard.putNumber("Applied rotationalRate (rad/s)", outputRotation);
        SmartDashboard.putNumber("Applied velocityX", outputX);
        SmartDashboard.putNumber("Applied velocityY", outputY);
    }
    
    @Override
    public boolean isFinished() {
        // We consider the command finished once the x (distance) controller reaches its setpoint.
        return xPidController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(idleRequest);
    }
}
