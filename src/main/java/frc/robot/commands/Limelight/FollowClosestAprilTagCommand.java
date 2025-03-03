package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class FollowClosestAprilTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController xController;
    private final PIDController thetaController;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    private static final double MAX_FORWARD_SPEED = 0.5; // Max forward speed (m/s) for safety

    public FollowClosestAprilTagCommand(LimelightSubsystem limelight, 
                                CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        // PID for forward/backward movement (X-axis)
        xController = new PIDController(0.5, 0, 0.05); 
        xController.setSetpoint(0); // Target X = 0 (directly at the tag)

        // PID for rotation (theta)
        thetaController = new PIDController(3.0, 0, 0.2);
        thetaController.setSetpoint(0); // Target tx = 0 (centered)

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget()) return;

        double[] pose = limelight.getTargetPose();
        if (pose == null || pose.length < 3) return;

        double currentX = pose[0]; // Forward distance from the tag

        // Calculate speeds for forward motion and rotation
        double xSpeed = xController.calculate(currentX);
        double thetaSpeed = thetaController.calculate(Units.degreesToRadians(limelight.getTargetX()));

        // Limit forward speed for safety
        xSpeed = Math.min(xSpeed, MAX_FORWARD_SPEED);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(xSpeed) // Move toward/away from the tag
                .withVelocityY(0) // No lateral movement
                .withRotationalRate(thetaSpeed) // Rotate to face the tag
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}