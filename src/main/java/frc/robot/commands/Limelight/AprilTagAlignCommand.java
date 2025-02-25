package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AprilTagAlignCommand extends Command {
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean alignRight; 

    private final ProfiledPIDController rotationController;
    private final PIDController distanceController;

    // Constants
    private static final double TARGET_DISTANCE = 1.0; // meters
    private static final double MAX_SPEED = 1.0; // meters per second (reduced)
    private static final double MAX_ANGULAR_SPEED = Math.PI / 2; // radians per second (reduced)
    private static final double MAX_ANGULAR_ACCELERATION = Math.PI / 4; // radians per second²
    private static final double LIMELIGHT_HEIGHT = 0.5; // meters
    private static final double APRILTAG_HEIGHT = 0.2; // meters
    private static final double LIMELIGHT_MOUNT_ANGLE = 30.0; // degrees
    private static final double HORIZONTAL_OFFSET = 0.5; // meters (distance to the left or right of the AprilTag)

    public AprilTagAlignCommand(LimelightSubsystem limelight, CommandSwerveDrivetrain drivetrain, boolean alignRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.alignRight = alignRight; 

        // Configure rotation controller (ProfiledPIDController for smooth motion)
        rotationController = new ProfiledPIDController(
            0.05, // kP
            0.0, // kI
            0.01, // kD
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED, // Maximum angular velocity (rad/s)
                MAX_ANGULAR_ACCELERATION // Maximum angular acceleration (rad/s²)
            )
        );
        rotationController.setTolerance(1.0); // degrees tolerance
        rotationController.enableContinuousInput(-180, 180);

        // Configure distance controller (regular PIDController)
        distanceController = new PIDController(0.3, 0, 0.1);
        distanceController.setTolerance(0.1); // meters tolerance

        addRequirements(limelight, drivetrain);
    }

    @Override
    public void initialize() {
        limelight.setLedMode(3); // Turn on LEDs
        limelight.setPipeline(0); // AprilTag pipeline
    }

@Override
public void execute() {
    if (!limelight.hasTarget()) return;

    // Get Limelight data
    double tx = limelight.getTargetX();
    double ty = limelight.getTargetY();
    double distance = calculateDistance(ty);
    System.out.println("apriltag! Distance: " + distance);

    // Adjust the tx target based on the alignment direction
    double horizontalOffset = alignRight ? HORIZONTAL_OFFSET : -HORIZONTAL_OFFSET;
    double txTarget = Math.toDegrees(Math.atan(horizontalOffset / distance));

    // Calculate rotation speed using ProfiledPIDController
    double rotationSpeed = -rotationController.calculate(tx, txTarget);

    // Calculate forward speed using regular PIDController
    double forwardSpeed = -distanceController.calculate(distance, TARGET_DISTANCE);

    // Apply speed limits
    rotationSpeed = Math.max(-MAX_ANGULAR_SPEED, Math.min(rotationSpeed, MAX_ANGULAR_SPEED));
    forwardSpeed = Math.max(-MAX_SPEED, Math.min(forwardSpeed, MAX_SPEED));

    // Create final copies of the speeds for use in the lambda
    final double finalForwardSpeed = forwardSpeed;
    final double finalRotationSpeed = rotationSpeed;

    // Drive the robot using a SwerveRequest.FieldCentric request
    drivetrain.applyRequest(() -> 
        new SwerveRequest.FieldCentric()
            .withVelocityX(finalForwardSpeed) // Forward/backward speed
            .withVelocityY(0)                 // No sideways movement
            .withRotationalRate(finalRotationSpeed) // Rotational speed
    );
}

    private double calculateDistance(double ty) {
        double angleToTarget = LIMELIGHT_MOUNT_ANGLE + ty;
        return (APRILTAG_HEIGHT - LIMELIGHT_HEIGHT) / 
               Math.tan(Math.toRadians(angleToTarget));
    }

    @Override
    public boolean isFinished() {
        // Check if both controllers are at their setpoints
        return rotationController.atGoal() && distanceController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.applyRequest(() -> 
            new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        limelight.setLedMode(1); // Turn off LEDs
    }
}