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

    // Initialize the PID controllers once.
    private static PIDControllerConfigurable rotationalPidController = 
        new PIDControllerConfigurable(VisionConstants.ROTATE_P, VisionConstants.ROTATE_I, VisionConstants.ROTATE_D, VisionConstants.TOLERANCE);
    private static final PIDControllerConfigurable xPidController = 
        new PIDControllerConfigurable(VisionConstants.MOVE_P, VisionConstants.MOVE_I, VisionConstants.MOVE_D, VisionConstants.TOLERANCE);
    private static final PIDControllerConfigurable yPidController = 
        new PIDControllerConfigurable(VisionConstants.STRAFE_P, VisionConstants.STRAFE_I, VisionConstants.STRAFE_D, VisionConstants.TOLERANCE);

    // Minimum output thresholds (tunable)
    private static final double MIN_ROTATIONAL_OUTPUT = 0.05; // Example value in radians per second
    private static final double MIN_VELOCITY_OUTPUT = 0.1;    // Example value in meters per second

    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    private static int tagID = -1;
    
    public double rotationalRate = 0;
    public double velocityX = 0;
    public double velocityY = 0;

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
        // Optionally reset controllers here if needed.
        rotationalPidController.reset();
        xPidController.reset();
        yPidController.reset();
    }

    @Override
    public void execute() {
        // Update rotational PID parameters from SmartDashboard without recreating the controller.
        rotationalPidController.setP(SmartDashboard.getNumber("Rotate P", VisionConstants.ROTATE_P));
        rotationalPidController.setD(SmartDashboard.getNumber("Rotate D", VisionConstants.ROTATE_D));
        // (Update I if needed)

        try {
            RawFiducial fiducial;
            if (tagID == -1) {
                fiducial = m_Limelight.getFiducialWithId(m_Limelight.getClosestFiducial().id);
            } else {
                fiducial = m_Limelight.getFiducialWithId(tagID);
            }

            // Calculate X velocity (forward/backward).
            double rawXOutput = xPidController.calculate(fiducial.distToRobot, 1.0);
            velocityX = rawXOutput 
                * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) 
                * 0.6;
            if (!xPidController.atSetpoint() && Math.abs(velocityX) < MIN_VELOCITY_OUTPUT) {
                velocityX = Math.copySign(MIN_VELOCITY_OUTPUT, velocityX);
            }

            // Calculate Y velocity (strafing).
            double yError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
            double rawYOutput = yPidController.calculate(yError, 0.0);
            velocityY = rawYOutput 
                * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) 
                * 0.6;
            if (!yPidController.atSetpoint() && Math.abs(velocityY) < MIN_VELOCITY_OUTPUT) {
                velocityY = Math.copySign(MIN_VELOCITY_OUTPUT, velocityY);
            }

            // Only compute rotational movement if both x and y have reached their setpoints.
            if (xPidController.atSetpoint() && yPidController.atSetpoint()) {
                double rawRotationalOutput = rotationalPidController.calculate(fiducial.txnc, 0.0);
                rotationalRate = rawRotationalOutput 
                    * RotationsPerSecond.of(0.75).in(RadiansPerSecond) 
                    * -2;
                if (!rotationalPidController.atSetpoint() && Math.abs(rotationalRate) < MIN_ROTATIONAL_OUTPUT) {
                    rotationalRate = Math.copySign(MIN_ROTATIONAL_OUTPUT, rotationalRate);
                }
            } else {
                rotationalRate = 0.0;
            }

            // Apply movement to the drivetrain.
            m_drivetrain.setControl(
                alignRequest
                    .withRotationalRate(-rotationalRate)  // Verify sign as needed.
                    .withVelocityX(-velocityX)
                    .withVelocityY(velocityY)
            );

            // Update dashboard values.
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
        System.out.println("rotation: " + rotationalPidController.atSetpoint());
        System.out.println("x: " + xPidController.atSetpoint());
        System.out.println("y: " + yPidController.atSetpoint());

        return rotationalPidController.atSetpoint() 
            && xPidController.atSetpoint() 
            && yPidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> idleRequest);
    }
}
