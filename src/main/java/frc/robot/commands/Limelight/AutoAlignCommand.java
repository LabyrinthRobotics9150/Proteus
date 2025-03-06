package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoAlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final double kP = 0.02; // tuning constant (adjust as needed)
    private final double kToleranceDegrees = 0.5; // tolerance in degrees for alignment

    public enum AlignmentDirection {
        CENTER, LEFT, RIGHT
    }
    private final AlignmentDirection direction;

    // Default constructor aligns to center (i.e. makes the target straight ahead)
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
        // Optionally, you can reset any PID state here.
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            // Get horizontal offset in degrees from limelight
            double tx = vision.getTx();
            // Optionally add a bias if aligning left/right (for example purposes, here we simply subtract or add a fixed offset)
            if (direction == AlignmentDirection.LEFT) {
                tx -= 2.0;  // adjust this offset as needed
            } else if (direction == AlignmentDirection.RIGHT) {
                tx += 2.0;  // adjust this offset as needed
            }

            // Calculate the rotational correction (convert tx from degrees to radians for the rotational rate)
            double rotationalCorrection = kP * Math.toRadians(tx);
            // Optionally, if error is very small, command zero rotation
            if (Math.abs(tx) < kToleranceDegrees) {
                rotationalCorrection = 0;
            }

            // Build a FieldCentric request that commands zero translation and the calculated rotational rate
            SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationalCorrection);
            drivetrain.setControl(request);
        } else {
            // No valid target; stop rotation (or you might choose to maintain the last command)
            drivetrain.setControl(new SwerveRequest.FieldCentric()
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop any commanded motion when the command ends
        drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Command ends when the tag is aligned within the tolerance (and a target is present)
        return vision.hasTarget() && Math.abs(vision.getTx()) < kToleranceDegrees;
    }
}
