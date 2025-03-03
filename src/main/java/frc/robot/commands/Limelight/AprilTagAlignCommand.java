package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AprilTagAlignCommand extends Command {
    private enum State { ALIGN, APPROACH, LATERAL }
    
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean alignRight;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private State currentState = State.ALIGN;

    // Constants
    private static final double TARGET_DISTANCE = 0.3; // meters
    private static final double LATERAL_OFFSET = 0.5; // meters
    private static final double POSITION_TOLERANCE = 0.03;
    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(1.0);
    private static final double MAX_LINEAR_SPEED = 0.4;
    private static final double MAX_ANGULAR_SPEED = Math.PI/6;
    
    // Camera constants (update with actual values)
    private static final double CAMERA_HEIGHT = 0.5; // meters
    private static final double TARGET_HEIGHT = 1.0; // meters
    private static final double CAMERA_PITCH = Units.degreesToRadians(25);

    public AprilTagAlignCommand(LimelightSubsystem limelight, 
                               CommandSwerveDrivetrain drivetrain,
                               boolean alignRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.alignRight = alignRight;

        // PID Controllers
        xController = new PIDController(1.5, 0, 0.1);
        yController = new PIDController(1.2, 0, 0.1);
        thetaController = new PIDController(2.5, 0, 0.15);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.setLedMode(3);
        currentState = State.ALIGN;
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget()) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        switch (currentState) {
            case ALIGN:
                handleAlignPhase();
                break;
            case APPROACH:
                handleApproachPhase();
                break;
            case LATERAL:
                handleLateralPhase();
                break;
        }
    }

    private void handleAlignPhase() {
        // Center the AprilTag in the camera view
        double tx = limelight.getTargetX();
        double rotationSpeed = thetaController.calculate(Units.degreesToRadians(tx), 0);
        
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );

        // Transition when aligned
        if (Math.abs(tx) < 1.0) { // Within 1 degree
            currentState = State.APPROACH;
            xController.setSetpoint(TARGET_DISTANCE);
        }
    }

    private void handleApproachPhase() {
        // Calculate distance using camera geometry
        double ty = limelight.getTargetY();
        double verticalAngle = Units.degreesToRadians(ty) + CAMERA_PITCH;
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(verticalAngle);
        
        // Drive straight forward/backward
        double xSpeed = xController.calculate(distance);
        
        drivetrain.setControl(
            driveRequest
                .withVelocityX(clamp(-xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withVelocityY(0)
                .withRotationalRate(0)
        );

        // Transition when at target distance
        if (Math.abs(distance - TARGET_DISTANCE) < POSITION_TOLERANCE) {
            currentState = State.LATERAL;
            yController.setSetpoint(alignRight ? -LATERAL_OFFSET : LATERAL_OFFSET);
        }
    }

    private void handleLateralPhase() {
        double[] pose = limelight.getTargetPose();
        if (pose == null) return;

        // Maintain distance while moving laterally
        double currentY = pose[1];
        double ySpeed = yController.calculate(currentY);
        
        // Maintain facing angle
        double currentYaw = Math.toRadians(pose[2]);
        double rotationSpeed = thetaController.calculate(currentYaw, 0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
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