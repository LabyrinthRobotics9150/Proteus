package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FollowClosestAprilTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;

    private final ProfiledPIDController rotationController;
    private final PIDController distanceController;

    // Constants 
    private static final double TARGET_DISTANCE = 1.0; // meters
    private static final double MAX_SPEED = 1.0; // meters/sec 
    private static final double MAX_ANGULAR_SPEED = Math.PI / 2; // rad/sec 
    private static final double MAX_ANGULAR_ACCELERATION = Math.PI / 4; // rad/sec²
    private static final double LIMELIGHT_HEIGHT = 0.5; // meters
    private static final double APRILTAG_HEIGHT = 0.2; // meters
    private static final double LIMELIGHT_MOUNT_ANGLE = 30.0; // degrees

    public FollowClosestAprilTagCommand(LimelightSubsystem limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;


        rotationController = new ProfiledPIDController(
            0.1, 0, 0.01, // PID gains
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED,
                MAX_ANGULAR_ACCELERATION
            )
        );
        rotationController.enableContinuousInput(-180, 180); 
        
        // Configure distance controller
        distanceController = new PIDController(0.3, 0, 0.05); 
        distanceController.setTolerance(0.1);

        addRequirements(limelight, drivetrain);
    }

    @Override
    public void initialize() {
        limelight.setLedMode(3); // Force LED on
        limelight.setPipeline(0); // AprilTag pipeline
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget()) return;

        // Get target data
        double tx = limelight.getTargetX();
        double ty = limelight.getTargetY();
        double distance = calculateDistance(ty);

        double rotationSpeed = -rotationController.calculate(tx, 0); // Center X
        double forwardSpeed = -distanceController.calculate(distance, TARGET_DISTANCE);

        // Apply speed limits
        rotationSpeed = Math.max(-MAX_ANGULAR_SPEED, Math.min(rotationSpeed, MAX_ANGULAR_SPEED));
        forwardSpeed = Math.max(-MAX_SPEED, Math.min(forwardSpeed, MAX_SPEED));

        final double finalForwardSpeed = forwardSpeed;
        final double finalRotationSpeed = rotationSpeed;

        // apply drive
        drivetrain.applyRequest(() -> 
            new SwerveRequest.FieldCentric()
                .withVelocityX(finalForwardSpeed)
                .withVelocityY(0)
                .withRotationalRate(finalRotationSpeed)
        );
    }

    private double calculateDistance(double ty) {
        double angleToTarget = LIMELIGHT_MOUNT_ANGLE + ty;
        return (APRILTAG_HEIGHT - LIMELIGHT_HEIGHT) / 
               Math.tan(Math.toRadians(angleToTarget));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> 
            new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        
        if (interrupted) {
            System.out.println("Tracking interrupted by driver input!");
        }
        limelight.setLedMode(1);
    }
}