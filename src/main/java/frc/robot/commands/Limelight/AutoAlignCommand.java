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

class PIDControllerConfigurable extends PIDController {
    public PIDControllerConfigurable(double kP, double kI, double kD) {
        super(kP, kI, kD);
    }
    
    public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
        super(kP, kI, kD);
        this.setTolerance(tolerance);
    }
}

public class AutoAlignCommand extends Command {
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final VisionSubsystem m_Limelight;
    
    // PID controllers remain the same as before.
    private static PIDControllerConfigurable rotationalPidController = 
        new PIDControllerConfigurable(0.2, 0.0, 0.005, 0.3);
    private static final PIDControllerConfigurable xPidController = 
        new PIDControllerConfigurable(0.8, 0.0, 0.001, 0.02);
    private static final PIDControllerConfigurable yPidController = 
        new PIDControllerConfigurable(0.8, 0.0, 0.001, 0.02);
    
    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    
    private static int tagID = -1;
    private double yoffset;
    private double desiredDistance = 0.3; // Closer target distance

    // Tolerance (in degrees) for acceptable rotation alignment.
    private static final double ROTATION_TOLERANCE = 1.0;
    
    // Allowed AprilTag IDs for auto-alignment.
    private static final int[] ALLOWED_TAG_IDS = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
    
    // Two alignment phases: first rotate, then drive (x and y) concurrently.
    private enum AlignStage { ROTATION, TRANSLATION }
    private AlignStage currentStage = AlignStage.ROTATION;
    
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
        yoffset = rightAlign ? 0.01 : -0.01;
    }
    
    @Override
    public void initialize() {
        rotationalPidController.reset();
        xPidController.reset();
        yPidController.reset();
        currentStage = AlignStage.ROTATION;
        try {
            tagID = m_Limelight.getClosestFiducial().id;
        } catch (VisionSubsystem.NoSuchTargetException e) {
            tagID = -1;
        }
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
        
        double rotationOutput = 0.0;
        double forwardOutput = 0.0;
        double lateralOutput = 0.0;
        
        // Phase 1: Rotation alignment.
        if (currentStage == AlignStage.ROTATION) {
            if (Math.abs(fiducial.txnc) > ROTATION_TOLERANCE) {
                rotationOutput = rotationalPidController.calculate(fiducial.txnc, 0.0);
                forwardOutput = 0.0;
                lateralOutput = 0.0;
            } else {
                // Once within tolerance, switch to translation phase.
                currentStage = AlignStage.TRANSLATION;
            }
        }
        
        // Phase 2: Translation (x and y) control.
        if (currentStage == AlignStage.TRANSLATION) {
            // Assume rotation remains good; do not further adjust rotation.
            rotationOutput = 0.0;
            forwardOutput = -xPidController.calculate(fiducial.distToRobot, desiredDistance);
            // For lateral alignment, use the vision's vertical offset (tync) as an approximation.
            lateralOutput = yPidController.calculate(fiducial.tync, yoffset);
        }
        
        // Apply the computed outputs.
        m_drivetrain.setControl(
            alignRequest
                .withRotationalRate(rotationOutput)
                .withVelocityX(forwardOutput)
                .withVelocityY(lateralOutput)
        );
        
        // Publish status for debugging.
        SmartDashboard.putString("AutoAlign_Phase", currentStage.name());
        SmartDashboard.putNumber("AutoAlign_txnc", fiducial.txnc);
        SmartDashboard.putNumber("AutoAlign_distToRobot", fiducial.distToRobot);
        SmartDashboard.putNumber("AutoAlign_rotationOutput", rotationOutput);
        SmartDashboard.putNumber("AutoAlign_forwardOutput", forwardOutput);
        SmartDashboard.putNumber("AutoAlign_lateralOutput", lateralOutput);
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes only when all controllers are at setpoint in the translation phase.
        return (currentStage == AlignStage.TRANSLATION) &&
               xPidController.atSetpoint() &&
               yPidController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(idleRequest);
    }
}
