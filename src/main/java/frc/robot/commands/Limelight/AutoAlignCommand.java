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

    // Increased minimum output thresholds (tunable)
    private static final double MIN_ROTATIONAL_OUTPUT = 0.1; // Minimum power for rotation (radians per second)
    private static final double MIN_VELOCITY_OUTPUT = 0.2;    // Minimum power for translation (meters per second)
    // New threshold for how close the rotation error must be (in degrees) before considering it aligned.
    private static final double ROTATION_ERROR_THRESHOLD_DEGREES = 1.0; 

    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    private static int tagID = -1;
    
    // State machine for sequential control.
    private enum AlignStage {
        ALIGN_Y,
        ALIGN_ROTATION,
        CORRECT_Y,
        DRIVE_X
    }
    private AlignStage currentStage;

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
        // Reset controllers and start with Y alignment.
        rotationalPidController.reset();
        xPidController.reset();
        yPidController.reset();
        currentStage = AlignStage.ALIGN_Y;
    }

    @Override
    public void execute() {
        // Update rotational PID parameters from SmartDashboard.
        rotationalPidController.setP(SmartDashboard.getNumber("Rotate P", VisionConstants.ROTATE_P));
        rotationalPidController.setD(SmartDashboard.getNumber("Rotate D", VisionConstants.ROTATE_D));
        // (Optionally update other PID gains if needed.)

        try {
            RawFiducial fiducial;
            if (tagID == -1) {
                fiducial = m_Limelight.getFiducialWithId(m_Limelight.getClosestFiducial().id);
            } else {
                fiducial = m_Limelight.getFiducialWithId(tagID);
            }
            
            // Initialize control outputs.
            double outputX = 0.0;
            double outputY = 0.0;
            double outputRotation = 0.0;
            
            switch(currentStage) {
                case ALIGN_Y: {
                    // Step 1: Adjust lateral position (Y) only.
                    double yError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
                    double rawYOutput = yPidController.calculate(yError, 0.0);
                    outputY = rawYOutput * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6;
                    if (!yPidController.atSetpoint() && Math.abs(outputY) < MIN_VELOCITY_OUTPUT) {
                        outputY = Math.copySign(MIN_VELOCITY_OUTPUT, outputY);
                    }
                    // Hold X and rotation at zero.
                    outputX = 0.0;
                    outputRotation = 0.0;
                    
                    // Move to next stage when Y is aligned.
                    if (yPidController.atSetpoint()) {
                        currentStage = AlignStage.ALIGN_ROTATION;
                    }
                    break;
                }
                case ALIGN_ROTATION: {
                    // Step 2: Adjust rotation while holding X and Y.
                    double rotationError = fiducial.txnc; // error in degrees
                    if (Math.abs(rotationError) < ROTATION_ERROR_THRESHOLD_DEGREES) {
                        outputRotation = 0.0;
                        currentStage = AlignStage.CORRECT_Y;
                    } else {
                        double rawRotOutput = rotationalPidController.calculate(rotationError, 0.0);
                        outputRotation = rawRotOutput * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * -2;
                        if (!rotationalPidController.atSetpoint() && Math.abs(outputRotation) < MIN_ROTATIONAL_OUTPUT) {
                            outputRotation = Math.copySign(MIN_ROTATIONAL_OUTPUT, outputRotation);
                        }
                    }
                    outputX = 0.0;
                    outputY = 0.0;
                    break;
                }
                case CORRECT_Y: {
                    // Step 3: Re-check and correct Y if needed.
                    double yError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
                    double rawYOutput = yPidController.calculate(yError, 0.0);
                    outputY = rawYOutput * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6;
                    if (!yPidController.atSetpoint() && Math.abs(outputY) < MIN_VELOCITY_OUTPUT) {
                        outputY = Math.copySign(MIN_VELOCITY_OUTPUT, outputY);
                    }
                    outputX = 0.0;
                    outputRotation = 0.0;
                    
                    // When Y is corrected, move to forward (X) drive.
                    if (yPidController.atSetpoint()) {
                        currentStage = AlignStage.DRIVE_X;
                    }
                    break;
                }
                case DRIVE_X: {
                    // Step 4: Drive forward along the X-axis.
                    double rawXOutput = xPidController.calculate(fiducial.distToRobot, 1.0);
                    outputX = rawXOutput * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6;
                    if (!xPidController.atSetpoint() && Math.abs(outputX) < MIN_VELOCITY_OUTPUT) {
                        outputX = Math.copySign(MIN_VELOCITY_OUTPUT, outputX);
                    }
                    // Hold Y and rotation at zero.
                    outputY = 0.0;
                    outputRotation = 0.0;
                    break;
                }
            }
            
            // Send the computed commands to the drivetrain.
            m_drivetrain.setControl(
                alignRequest
                    .withRotationalRate(-outputRotation)  // Adjust sign as needed.
                    .withVelocityX(-outputX)
                    .withVelocityY(outputY)
            );
            
            // Update SmartDashboard with current values.
            SmartDashboard.putString("Align Stage", currentStage.name());
            SmartDashboard.putNumber("txnc", fiducial.txnc);
            SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
            SmartDashboard.putNumber("rotationalRate", outputRotation);
            SmartDashboard.putNumber("velocityX", outputX);
            SmartDashboard.putNumber("velocityY", outputY);
            
        } catch (VisionSubsystem.NoSuchTargetException e) {
            System.out.println("No AprilTag found");
            m_drivetrain.setControl(idleRequest);
        }
    }
    
    @Override
    public boolean isFinished() {
        // Finish when in DRIVE_X stage and the X controller is at setpoint.
        return currentStage == AlignStage.DRIVE_X && xPidController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> idleRequest);
    }
}
