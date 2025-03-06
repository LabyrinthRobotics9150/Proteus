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
    public enum AlignmentDirection {
        LEFT,
        CENTER,
        RIGHT
    }

    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_Limelight;
    private final int tagID;
    private final AlignmentDirection alignmentDirection;

    private final PIDControllerConfigurable rotationalPidController;
    private final PIDControllerConfigurable xPidController;

    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    
    public double rotationalRate = 0;
    public double velocityX = 0;

    // Constructors
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
        this(drivetrain, limelight, -1, AlignmentDirection.CENTER);
    }

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int ID) {
        this(drivetrain, limelight, ID, AlignmentDirection.CENTER);
    }

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, AlignmentDirection direction) {
        this(drivetrain, limelight, -1, direction);
    }

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, int ID, AlignmentDirection direction) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        /* 
         * 
        if (ID < 0 && direction != AlignmentDirection.CENTER) {
            throw new IllegalArgumentException("When using alignment direction, tagID must be specified");
        }
        */
        this.tagID = ID;
        this.alignmentDirection = direction;
        
        // Initialize PID controllers
        rotationalPidController = new PIDControllerConfigurable(
            0.05000, 0.000000, 0.001000, 0.01);
        xPidController = new PIDControllerConfigurable(
            0.400000, 0.000000, 0.000600, 0.01);

        addRequirements(m_Limelight);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {    
        try {
            RawFiducial fiducial = (tagID == -1) ?
                m_Limelight.getClosestFiducial() :
                m_Limelight.getFiducialWithId(tagID);

            // Determine alignment setpoint
            double rotationalSetpoint = 0.0;
            switch(alignmentDirection) {
                case LEFT:
                    rotationalSetpoint = VisionConstants.LEFT_ALIGN_TX_SETPOINT;
                    break;
                case RIGHT:
                    rotationalSetpoint = VisionConstants.RIGHT_ALIGN_TX_SETPOINT;
                    break;
                case CENTER:
                default:
                    rotationalSetpoint = 0.0;
            }

            // Calculate control outputs
            rotationalRate = rotationalPidController.calculate(
                2 * fiducial.txnc, 
                rotationalSetpoint
            ) * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * -0.1;

            final double velocityX = xPidController.calculate(
                fiducial.distToRobot, 
                0.3
            ) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6;

            // Update dashboard
            SmartDashboard.putNumber("txnc", fiducial.txnc);
            SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
            SmartDashboard.putNumber("rotationalPidController", rotationalRate);
            SmartDashboard.putNumber("xPidController", velocityX);

            // Drive robot
            m_drivetrain.setControl(
                alignRequest
                    .withRotationalRate(-rotationalRate)
                    .withVelocityX(-velocityX)
            );

        } catch (VisionSubsystem.NoSuchTargetException nste) { 
            System.out.println("No apriltag found");
            m_drivetrain.setControl(
                alignRequest
                    .withRotationalRate(-1.5 * rotationalRate)
                    .withVelocityX(-1.5 * velocityX)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return rotationalPidController.atSetpoint() && xPidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> idleRequest);
    }
}