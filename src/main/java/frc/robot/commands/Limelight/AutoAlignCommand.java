package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class AutoAlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    // Gains and thresholds for the ALIGN phase
    private final double kY_align = 0.05;           // lateral correction (m/s per degree)
    private final double kR_align = 5.0;            // rotational correction (rad/s per radian error)
    private final double toleranceAlign = 1.0;      // error (in degrees) below which we consider the target centered

    // Gains for the APPROACH phase (milder corrections while driving forward)
    private final double kY_approach = 0.02;        // lateral correction gain during approach
    private final double kR_approach = 2.0;         // rotational correction gain during approach
    private final double toleranceApproachExit = 1.5; // if error exceeds this, revert to ALIGN phase

    private final double forwardSpeed = 0.2;        // constant forward speed (m/s) during approach

    private enum State {
        ALIGN,
        APPROACH
    }
    private State currentState = State.ALIGN;

    public enum AlignmentDirection {
        CENTER, LEFT, RIGHT
    }
    private final AlignmentDirection direction;

    // Default constructor aligns to center (target straight ahead)
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this(drivetrain, vision, AlignmentDirection.CENTER);
    }

    // Optionally allow an alignment bias (e.g. LEFT or RIGHT)
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, AlignmentDirection direction) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.direction = direction;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        currentState = State.ALIGN;
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            // If no target is seen, reset state and stop movement.
            currentState = State.ALIGN;
            drivetrain.setControl(new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
            return;
        }

        double tx = vision.getTx();

        // Apply optional bias for left/right alignment.
        if (direction == AlignmentDirection.LEFT) {
            tx -= 2.0;
        } else if (direction == AlignmentDirection.RIGHT) {
            tx += 2.0;
        }

        // State machine: in ALIGN state we correct laterally and rotationally (without moving forward)
        // until the target is centered; then we switch to APPROACH state.
        if (currentState == State.ALIGN) {
            double lateral = kY_align * tx;                // move left/right proportionally to error
            double rotation = kR_align * Math.toRadians(tx); // rotate proportionally (tx converted to radians)
            double forward = 0;                            // no forward motion during alignment

            if (Math.abs(tx) < toleranceAlign) {
                currentState = State.APPROACH;  // target is centered—ready to approach
            }

            SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(forward)
                .withVelocityY(lateral)
                .withRotationalRate(rotation);
            drivetrain.setControl(request);
        }
        // In the APPROACH state we drive forward and apply gentler corrections.
        else if (currentState == State.APPROACH) {
            double lateral = kY_approach * tx;
            double rotation = kR_approach * Math.toRadians(tx);
            double forward = forwardSpeed;

            // If the error increases beyond a threshold, revert to the ALIGN state.
            if (Math.abs(tx) > toleranceApproachExit) {
                currentState = State.ALIGN;
            }

            SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(forward)
                .withVelocityY(lateral)
                .withRotationalRate(rotation);
            drivetrain.setControl(request);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motion when the command ends.
        drivetrain.setControl(new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // This command continues until externally cancelled (or add a condition based on distance
        return false;
    }
}