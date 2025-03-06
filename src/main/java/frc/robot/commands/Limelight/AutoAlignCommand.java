package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class AutoAlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    // Increased proportional gain for sufficient rotational output.
    private final double kP = 5.0; 
    private final double kToleranceDegrees = 0.5; // tolerance in degrees for alignment
    // A constant forward speed (in meters per second) to drive toward the tag.
    private final double forwardSpeed = 0.2; 

    public enum AlignmentDirection {
        CENTER, LEFT, RIGHT
    }
    private final AlignmentDirection direction;

    // Default constructor aligns to center (i.e. target straight ahead)
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this(drivetrain, vision, AlignmentDirection.CENTER);
    }

    // Allows specifying an alignment bias (if needed)
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, AlignmentDirection direction) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.direction = direction;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Optionally reset any state here.
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            System.out.println("target!");
            double tx = vision.getTx();

            // Apply bias for left/right if needed.
            if (direction == AlignmentDirection.LEFT) {
                tx -= 2.0;  
            } else if (direction == AlignmentDirection.RIGHT) {
                tx += 2.0;  
            }

            // Compute rotational correction.
            double rotationalCorrection = kP * Math.toRadians(tx);
            if (Math.abs(tx) < kToleranceDegrees) {
                rotationalCorrection = 0;
            }

            // Create a FieldCentric request that now includes:
            // - A forward velocity command (withVelocityX)
            // - A rotational command computed above.
            // - The drive request type is set to OpenLoopVoltage (matching the teleop setup)
            SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(forwardSpeed) 
                .withVelocityY(0)
                .withRotationalRate(rotationalCorrection);

            drivetrain.setControl(request);
        } else {
            // No valid target: command zero motion.
            drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Command ends when the target is aligned within tolerance.
        return vision.hasTarget() && Math.abs(vision.getTx()) < kToleranceDegrees;
    }
}
