package frc.robot.commands.Limelight;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
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
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_Limelight;

    private static PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(VisionConstants.ROTATE_P, VisionConstants.ROTATE_I, VisionConstants.ROTATE_D, VisionConstants.TOLERANCE);
    private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(VisionConstants.MOVE_P, VisionConstants.MOVE_I, VisionConstants.MOVE_D, VisionConstants.TOLERANCE);
    private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(VisionConstants.STRAFE_P, VisionConstants.STRAFE_I, VisionConstants.STRAFE_D, VisionConstants.TOLERANCE);

    private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    private static int tagID = -1;
    
    public double rotationalRate = 0;
    public double velocityX = 0;
    public double velocityY = 0;

    private boolean rotationAligned = false; // Tracks if rotation is aligned
    
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        addRequirements(m_Limelight);
    }

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int ID) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        if (ID < 0) throw new IllegalArgumentException("AprilTag ID cannot be negative");
        tagID = ID;
        addRequirements(m_Limelight);
    }

    @Override
    public void initialize() {
        rotationAligned = false; // Reset rotation state on command start
    }

    @Override
    public void execute() {
        rotationalPidController = new PIDControllerConfigurable(
            SmartDashboard.getNumber("Rotate P", 1), 
            VisionConstants.ROTATE_I, 
            SmartDashboard.getNumber("Rotate D", 0.0), 
            VisionConstants.TOLERANCE
        );

        try {
            RawFiducial fiducial;
            if (tagID == -1) {
                fiducial = m_Limelight.getFiducialWithId(m_Limelight.getClosestFiducial().id);
            } else {
                fiducial = m_Limelight.getFiducialWithId(tagID);
            }

            if (!rotationAligned) {
                // Phase 1: Align rotation first
                rotationalRate = rotationalPidController.calculate(fiducial.txnc, 0.0) 
                    * RotationsPerSecond.of(0.75).in(RadiansPerSecond) 
                    * -0.1;

                velocityX = 0;
                velocityY = 0;

                // Check if rotation is aligned
                if (rotationalPidController.atSetpoint()) {
                    rotationAligned = true;
                }
            } else {
                // Phase 2: Adjust X and Y while maintaining rotation
                velocityX = xPidController.calculate(fiducial.distToRobot, 1.0) 
                    * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) 
                    * 0.6;

                double yError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
                velocityY = yPidController.calculate(yError, 0.0) 
                    * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) 
                    * 0.6;

                // Continue adjusting rotation
                rotationalRate = rotationalPidController.calculate(fiducial.txnc, 0.0) 
                    * RotationsPerSecond.of(0.75).in(RadiansPerSecond) 
                    * -0.1;
            }

            // Apply movement with corrected rotational direction
            m_drivetrain.setControl(
                alignRequest
                    .withRotationalRate(rotationalRate) // Removed negative to fix direction
                    .withVelocityX(-velocityX)
                    .withVelocityY(velocityY)
            );

            // Update dashboard
            SmartDashboard.putNumber("txnc", fiducial.txnc);
            SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
            SmartDashboard.putNumber("rotationalRate", rotationalRate);
            SmartDashboard.putNumber("velocityX", velocityX);
            SmartDashboard.putNumber("velocityY", velocityY);

        } catch (VisionSubsystem.NoSuchTargetException e) {
            System.out.println("No AprilTag found");
            m_drivetrain.setControl(idleRequest);
        }
    }

    @Override
    public boolean isFinished() {
        // Check all controllers only after rotation is aligned
        return rotationAligned 
            && rotationalPidController.atSetpoint() 
            && xPidController.atSetpoint() 
            && yPidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> idleRequest);
    }
}