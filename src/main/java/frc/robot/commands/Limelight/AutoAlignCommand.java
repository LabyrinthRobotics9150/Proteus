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
    
    // PID controllers initialized once.
    private static PIDControllerConfigurable rotationalPidController = 
        new PIDControllerConfigurable(VisionConstants.ROTATE_P, VisionConstants.ROTATE_I, VisionConstants.ROTATE_D, VisionConstants.TOLERANCE);
    private static final PIDControllerConfigurable xPidController = 
        new PIDControllerConfigurable(VisionConstants.MOVE_P, VisionConstants.MOVE_I, VisionConstants.MOVE_D, VisionConstants.TOLERANCE);
    private static final PIDControllerConfigurable yPidController = 
        new PIDControllerConfigurable(VisionConstants.STRAFE_P, VisionConstants.STRAFE_I, VisionConstants.STRAFE_D, VisionConstants.TOLERANCE);
    
    // Minimum output thresholds (tunable)
    private static final double MIN_ROTATIONAL_OUTPUT = 0.001; // radians per second
    private static final double MIN_VELOCITY_OUTPUT = 0.2;    // meters per second
    // Rotation error threshold (degrees) for “good enough” rotation.
    private static final double ROTATION_ERROR_THRESHOLD_DEGREES = 1.8;
    
    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    private static int tagID = -1;
    private double yoffset;
    
    // State machine for sequential control.
    private enum AlignStage {
        ALIGN_ROTATION_1,
        ALIGN_Y_1,
        ALIGN_ROTATION_2,
        DRIVE_X,
        FINAL_CORRECT_Y,
        FINAL_CORRECT_ROTATION
    }
    private AlignStage currentStage;
    
    public double rotationalRate = 0;
    public double velocityX = 0;
    public double velocityY = 0;
    // align to closest apriltag centrally
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        yoffset = 0;
        addRequirements(m_Limelight);
    }
    // align to closest apriltag left/right
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, Boolean rightAlign) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        addRequirements(m_Limelight);
        if (rightAlign) {
            yoffset = -.25;
        } else {
            yoffset = .25;
        }
        
    }
    
    @Override
    public void initialize() {
        rotationalPidController.reset();
        xPidController.reset();
        yPidController.reset();
        tagID = m_Limelight.getClosestFiducial().id;
        currentStage = AlignStage.ALIGN_ROTATION_1;
    }
    
    @Override
    public void execute() {
        // Update rotational PID parameters from SmartDashboard.
        rotationalPidController.setP(SmartDashboard.getNumber("Rotate P", VisionConstants.ROTATE_P));
        rotationalPidController.setD(SmartDashboard.getNumber("Rotate D", VisionConstants.ROTATE_D));
        
        try {
            RawFiducial fiducial = m_Limelight.getFiducialWithId(m_Limelight.getClosestFiducial().id);
            if (tagID == -1) {
                fiducial = m_Limelight.getFiducialWithId(m_Limelight.getClosestFiducial().id);
                System.out.println("ID");
            } else {
                fiducial = m_Limelight.getFiducialWithId(tagID);
            }
            
            // Initialize control outputs.
            double outputX = 0.0;
            double outputY = 0.0;
            double outputRotation = 0.0;
            
            switch(currentStage) {
                case ALIGN_ROTATION_1: {
                    // Stage 1: Rotate until within threshold.
                    double rotationError = fiducial.txnc; // error in degrees
                    if (Math.abs(rotationError) < ROTATION_ERROR_THRESHOLD_DEGREES) {
                        outputRotation = 0.0;
                        currentStage = AlignStage.ALIGN_Y_1;
                    } else {
                        double rawRotOutput = rotationalPidController.calculate(rotationError, 0.0);
                        outputRotation = rawRotOutput * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * -2;
                        if (Math.abs(outputRotation) < MIN_ROTATIONAL_OUTPUT) {
                            outputRotation = Math.copySign(MIN_ROTATIONAL_OUTPUT, outputRotation);
                        }
                    }
                    outputX = 0.0;
                    outputY = 0.0;
                    break;
                }
                case ALIGN_Y_1: {
                    // Stage 2: Correct lateral (Y) alignment.
                    double yError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
                    double rawYOutput = yPidController.calculate(yError, yoffset);
                    outputY = rawYOutput * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6;
                    if (Math.abs(outputY) < MIN_VELOCITY_OUTPUT) {
                        outputY = Math.copySign(MIN_VELOCITY_OUTPUT, outputY);
                    }
                    outputX = 0.0;
                    outputRotation = 0.0;
                    
                    if (yPidController.atSetpoint()) {
                        currentStage = AlignStage.DRIVE_X;
                    }
                    break;
                }
                case ALIGN_ROTATION_2: {
                    // Stage 3: Re-correct rotation.
                    double rotationError = fiducial.txnc;
                    if (Math.abs(rotationError) < ROTATION_ERROR_THRESHOLD_DEGREES) {
                        outputRotation = 0.0;
                        currentStage = AlignStage.DRIVE_X;
                    } else {
                        double rawRotOutput = rotationalPidController.calculate(rotationError, 0.0);
                        outputRotation = rawRotOutput * RotationsPerSecond.of(0.75).in(RadiansPerSecond) * -2;
                        if (Math.abs(outputRotation) < MIN_ROTATIONAL_OUTPUT) {
                            outputRotation = Math.copySign(MIN_ROTATIONAL_OUTPUT, outputRotation);
                        }
                    }
                    outputX = 0.0;
                    outputY = 0.0;
                    break;
                }
                case DRIVE_X: {
                    System.out.println("ALIGN FORWARD");
                    // Stage 4: Drive forward (X).
                    double rawXOutput = xPidController.calculate(fiducial.distToRobot, .6);
                    outputX = rawXOutput * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6;
                    if (Math.abs(outputX) < MIN_VELOCITY_OUTPUT) {
                        outputX = Math.copySign(MIN_VELOCITY_OUTPUT, outputX);
                    }
                    outputY = 0.0;
                    outputRotation = 0.0;
                    
                    if (xPidController.atSetpoint()) {
                        currentStage = AlignStage.FINAL_CORRECT_Y;
                    }
                    break;
                }
            }
            
            m_drivetrain.setControl(
                alignRequest
                    .withRotationalRate(-outputRotation)
                    .withVelocityX(-outputX)
                    .withVelocityY(outputY)
            );
            
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
        // In the FINAL_CORRECT_ROTATION stage, finish when rotation error is within threshold,
        // Y is at setpoint, and X is at setpoint.
        if (currentStage == AlignStage.FINAL_CORRECT_ROTATION) {
            try {
                RawFiducial fiducial;
                if (tagID == -1) {
                    fiducial = m_Limelight.getFiducialWithId(m_Limelight.getClosestFiducial().id);
                } else {
                    fiducial = m_Limelight.getFiducialWithId(tagID);
                }
                boolean rotationAligned = Math.abs(fiducial.txnc) < ROTATION_ERROR_THRESHOLD_DEGREES;
                boolean yAligned = yPidController.atSetpoint();
                boolean xAligned = xPidController.atSetpoint();
                return rotationAligned && yAligned && xAligned;
            } catch (VisionSubsystem.NoSuchTargetException e) {
                return false;
            }
        }
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> idleRequest);
    }
}
