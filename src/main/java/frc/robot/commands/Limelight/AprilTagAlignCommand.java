package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AprilTagAlignCommand extends Command {
    private enum State { CENTER, LATERAL }
    
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean alignRight;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private State currentState = State.CENTER;

    // Constants
    private static final double TARGET_DISTANCE = 0.3; // meters in front of tag
    private static final double LATERAL_OFFSET = 0.5; // meters to side
    private static final double POSITION_TOLERANCE = 0.05;
    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(1.5);
    private static final double MAX_LINEAR_SPEED = 0.4;
    private static final double MAX_ANGULAR_SPEED = Math.PI/8;

    public AprilTagAlignCommand(LimelightSubsystem limelight, 
                               CommandSwerveDrivetrain drivetrain,
                               boolean alignRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.alignRight = alignRight;

        // PID Controllers
        xController = new PIDController(2.0, 0, 0.1);
        yController = new PIDController(1.5, 0, 0.1);
        thetaController = new PIDController(3.0, 0, 0.2);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.setLedMode(3);
        currentState = State.CENTER;
        xController.reset();
        yController.reset();
        thetaController.reset();
        
        // Initial setpoints - center first
        xController.setSetpoint(TARGET_DISTANCE);
        yController.setSetpoint(0); // Center laterally
        thetaController.setSetpoint(0); // Face directly
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget()) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        double[] pose = limelight.getTargetPose();
        if (pose == null) return;

        switch (currentState) {
            case CENTER:
                handleCenterPhase(pose);
                break;
            case LATERAL:
                handleLateralPhase(pose);
                break;
        }
    }

    private void handleCenterPhase(double[] pose) {
        // Get robot's position relative to AprilTag
        double currentX = pose[0]; // Forward distance (positive = in front)
        double currentY = pose[1]; // Lateral offset (positive = left)
        double currentYaw = Math.toRadians(pose[2]); // Angle from tag
        
        // Calculate motion components
        double xSpeed = xController.calculate(currentX);
        double ySpeed = yController.calculate(currentY);
        double rotationSpeed = thetaController.calculate(currentYaw);

        // Apply movement with speed limits
        drivetrain.setControl(
            driveRequest
                .withVelocityX(clamp(-xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withVelocityY(clamp(-ySpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withRotationalRate(clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );

        // Transition when centered and aligned
        if (xController.atSetpoint() && 
            yController.atSetpoint() && 
            thetaController.atSetpoint()) {
            currentState = State.LATERAL;
            yController.setSetpoint(alignRight ? -LATERAL_OFFSET : LATERAL_OFFSET);
        }
    }

    private void handleLateralPhase(double[] pose) {
        // Maintain forward distance while moving laterally
        double currentX = pose[0];
        double currentY = pose[1];
        
        // Calculate motion components
        double xSpeed = xController.calculate(currentX);
        double ySpeed = yController.calculate(currentY);
        double rotationSpeed = thetaController.calculate(Math.toRadians(pose[2]));

        drivetrain.setControl(
            driveRequest
                .withVelocityX(clamp(-xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withVelocityY(clamp(-ySpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withRotationalRate(clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );
    }

    @Override
    public boolean isFinished() {
        return currentState == State.LATERAL && yController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
        limelight.setLedMode(1);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}