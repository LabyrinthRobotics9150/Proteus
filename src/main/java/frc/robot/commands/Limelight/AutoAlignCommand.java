package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * Revised AutoAlignCommand now performs alignment in three stages:
 * 
 * 1. ALIGN_Y: Compute the lateral (Y) correction.
 * 2. ALIGN_ROTATION: Compute the rotation correction (using degrees for error).
 * 3. DRIVE_X: Drive forward/backward until the robot reaches the desired distance.
 * 
 * For each stage, if the PID output is too low, a minimum threshold is applied so that the command can overcome any drivetrain deadband.
 */
public class AutoAlignCommand extends Command {
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final VisionSubsystem m_Limelight;

    private static TrapezoidProfile.Constraints Rotationconstraints = new TrapezoidProfile.Constraints(.05, 10);
    private static TrapezoidProfile.Constraints xyconstraints = new TrapezoidProfile.Constraints(.05, 10);

    
    
    // – For rotation: error is in degrees and tolerance is 1°.
    private static ProfiledPIDController rotationalPidController = 
        new ProfiledPIDController(2.0, 0.0, 0.0, Rotationconstraints); //halfed, test
    // For forward drive (X) and lateral (Y) control (meters)
    private static final ProfiledPIDController xPidController = 
    new ProfiledPIDController(.7, 0.0, 0.0, xyconstraints);
    private static final ProfiledPIDController yPidController = 
    new ProfiledPIDController(.4, 0.0, 0.0, xyconstraints);
    
    // Using a robot-centric control request.
    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    
    // Track the chosen fiducial by ID (if available)
    private static int tagID = -1;
    // Lateral offset (in meters); for left/right alignment, set via the alternate constructor.
    private double yoffset;
    
    // Allowed fiducial IDs for auto-alignment.
    private static final int[] ALLOWED_TAG_IDS = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
    
    // State machine for alignment stages.
    private enum AlignStage {
        ALIGN_Y,
        ALIGN_ROTATION,
        DRIVE_X
    }
    private AlignStage currentStage;

    private static double outputX;
    private static double outputY;
    private static double outputRotation;
    
    // These fields will store the one‑shot PID outputs.
    private Double fixedRotationOutput = null;
    private Double fixedLateralOutput = null;
    
    // Minimum command thresholds:
    private static final double MIN_ROTATION_OUTPUT_DEG = 1; // Minimum rotational output in degrees per second
    private static final double MIN_LATERAL_OUTPUT = 0.05;     // Minimum lateral output in m/s
    private static final double MIN_DRIVE_X_OUTPUT = 0.05;       // Minimum forward/backward output in m/s
    
    // Constructor for central alignment.
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        yoffset = 0;
        addRequirements(m_Limelight);
        addRequirements(m_drivetrain);
    }
    
    // Constructor for left/right alignment. For right alignment, use a small positive lateral offset;
    // for left alignment, a small negative offset.
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, boolean rightAlign) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        yoffset = 0;
        if (rightAlign) {
            yoffset = .6;
        } else {
            yoffset = -.6;
        }

        addRequirements(m_Limelight);
    }
    
    @Override
    public void initialize() {
        rotationalPidController.reset(0);;
        xPidController.reset(0);;
        yPidController.reset(0);
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
        RawFiducial fiducial = new RawFiducial(tagID, outputY, outputX, outputRotation, MIN_ROTATION_OUTPUT_DEG, MIN_LATERAL_OUTPUT, MIN_DRIVE_X_OUTPUT);
        try {
            fiducial = m_Limelight.getFiducialWithId(tagID);
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
            end(true);
    }
        

        outputX = 0.0;
        outputY = 0.0;
        outputRotation = 0.0;

        // ALL AT ONCE

        // y stuff

        double lateralError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
        lateralError += yoffset;
        outputY = yPidController.calculate(lateralError, 0);
        // rotation stuff

        /*
        // Compute rotation error in degrees.
        double rotationErrorDeg = fiducial.txnc;
        if (fixedRotationOutput == null) {
            fixedRotationOutput = rotationalPidController.calculate(rotationErrorDeg, 0.0);
            // Enforce a minimum output if the computed value is too small.
            //if (Math.abs(fixedRotationOutput) < MIN_ROTATION_OUTPUT_DEG &&
            //    Math.abs(rotationErrorDeg) > rotationalPidController.getErrorTolerance()) {
            //}
        }
        // Convert the PID output (in deg/s) to radians per second for the drivetrain.
        outputRotation = Units.degreesToRadians(fixedRotationOutput);
        // When the rotation error is within 1°, lock in the correction and advance.
        if (Math.abs(rotationErrorDeg) < 3.0) {
            fixedRotationOutput = 0.0;
        }
            */


        // Compute rotation error in degrees.
        double rotationErrorRad = Units.degreesToRadians(fiducial.ta);

        // Always compute PID output based on current error.
        outputRotation = rotationalPidController.calculate(rotationErrorRad, 0.0);

        // Stop only when the error is within the PID's acceptable tolerance.

        // Convert to radians per second.
         

        // x stuff

        // Drive forward/backward until the robot is at the desired distance.
        double desiredDistance = 0.1; // meters
        outputX = -xPidController.calculate(fiducial.distToRobot, desiredDistance);
        // Enforce a minimum forward output if needed.
        if (Math.abs(outputX) < MIN_DRIVE_X_OUTPUT &&
            Math.abs(fiducial.distToRobot - desiredDistance) > xPidController.getPositionTolerance()) {
            outputX = Math.copySign(MIN_DRIVE_X_OUTPUT, outputX);
        }


        // Send the calculated control outputs.
        m_drivetrain.setControl(
            alignRequest
                .withRotationalRate(outputRotation)
                .withVelocityX(outputX)
                .withVelocityY(outputY)
        );


        /*
        
        switch (currentStage) {
            case ALIGN_Y: {
                System.out.println("Stage: ALIGN_Y");
                // Compute lateral error (meters) using the target’s horizontal angle.
                double lateralError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
                lateralError += yoffset;
                if (fixedLateralOutput == null) {
                    fixedLateralOutput = yPidController.calculate(lateralError, 0);
                    }
                outputY = fixedLateralOutput;
                // When lateral error is within 20 cm, lock in the correction.
                if (Math.abs(lateralError - yoffset) < 0.2) {
                    fixedLateralOutput = 0.0;
                    currentStage = AlignStage.ALIGN_ROTATION;
                
                break;
            }
        }
            case ALIGN_ROTATION: {
                System.out.println("Stage: ALIGN_ROTATION");
                // Compute rotation error in degrees.
                double rotationErrorDeg = fiducial.txnc;
                if (fixedRotationOutput == null) {
                    fixedRotationOutput = rotationalPidController.calculate(rotationErrorDeg, 0.0);
                    // Enforce a minimum output if the computed value is too small.
                    //if (Math.abs(fixedRotationOutput) < MIN_ROTATION_OUTPUT_DEG &&
                    //    Math.abs(rotationErrorDeg) > rotationalPidController.getErrorTolerance()) {
                    //}
                }
                // Convert the PID output (in deg/s) to radians per second for the drivetrain.
                outputRotation = Units.degreesToRadians(fixedRotationOutput);
                // When the rotation error is within 1°, lock in the correction and advance.
                if (Math.abs(rotationErrorDeg) < 3.0) {
                    fixedRotationOutput = 0.0;
                    currentStage = AlignStage.DRIVE_X;
                    System.out.println("Rotation at setpoint");
                }
                break;
            }
            case DRIVE_X: {
                System.out.println("Stage: DRIVE_X");
                // Drive forward/backward until the robot is at the desired distance.
                double desiredDistance = 0.1; // meters
                outputX = -xPidController.calculate(fiducial.distToRobot, desiredDistance);
                // Enforce a minimum forward output if needed.
                if (Math.abs(outputX) < MIN_DRIVE_X_OUTPUT &&
                    Math.abs(fiducial.distToRobot - desiredDistance) > xPidController.getPositionTolerance()) {
                    outputX = Math.copySign(MIN_DRIVE_X_OUTPUT, outputX);
                }
                break;
            }
        }
        
        // Send the calculated control outputs.
        m_drivetrain.setControl(
            alignRequest
                .withRotationalRate(outputRotation)
                .withVelocityX(outputX)
                .withVelocityY(outputY)
        );


    */
        
        // Publish for debugging.
        SmartDashboard.putString("Align Stage", currentStage.name());
        SmartDashboard.putNumber("txnc (deg)", fiducial.txnc);
        SmartDashboard.putNumber("distToRobot (m)", fiducial.distToRobot);
        SmartDashboard.putNumber("Applied rotationalRate (rad/s)", outputRotation);
        SmartDashboard.putNumber("Applied velocityX (m/s)", outputX);
        SmartDashboard.putNumber("Applied velocityY (m/s)", outputY);
    }
    
    @Override
    public boolean isFinished() {
        // We finish once the drive (X) PID controller reaches its setpoint.
        if (xPidController.atSetpoint()) {
            m_drivetrain.setControl(idleRequest);
            System.out.println("finished");
        }
        return xPidController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(idleRequest);
    }
}
