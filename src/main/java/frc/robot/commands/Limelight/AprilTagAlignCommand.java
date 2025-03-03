package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AprilTagAlignCommand extends Command {
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    
    // PID controller to adjust the robot's yaw.
    private final PIDController thetaController;
    
    // Minimum command threshold to overcome static friction.
    private static final double MIN_ANGULAR_COMMAND = 0.05;
    
    // Maximum allowed angular speed.
    private static final double MAX_ANGULAR_SPEED = Math.PI / 6;
    
    // Tolerance for alignment (in radians).
    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(1.0);

    public AprilTagAlignCommand(LimelightSubsystem limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        
        // Initialize the PID controller for rotation.
        thetaController = new PIDController(2.0, 0, 0.15);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(ANGLE_TOLERANCE);
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Do not change LED settings—leave them off.
        thetaController.reset();
        // We want to reduce yaw error to 0.
        thetaController.setSetpoint(0);
    }

    @Override
    public void execute() {
        // If no target is detected, idle.
        if (!limelight.hasTarget()) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }
        
        double[] pose = limelight.getTargetPose();
        if (pose == null) return;
        
        // pose[2] is the yaw error (in degrees). Convert to radians.
        double currentYaw = Math.toRadians(pose[2]);
        
        // Compute the rotational speed from the theta PID controller.
        double rotationSpeed = thetaController.calculate(currentYaw);
        rotationSpeed = applyMinCommand(rotationSpeed, MIN_ANGULAR_COMMAND);
        rotationSpeed = clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
        
        // Command the drivetrain to rotate while keeping translational speeds zero.
        drivetrain.setControl(new SwerveRequest.RobotCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotationSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        // Finish when the yaw error is within the specified tolerance.
        return thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
    
    /**
     * Ensures that a nonzero output meets a minimum magnitude.
     */
    private double applyMinCommand(double output, double minCommand) {
        if (output != 0 && Math.abs(output) < minCommand) {
            return Math.copySign(minCommand, output);
        }
        return output;
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
