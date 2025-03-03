package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AprilTagAlignCommand extends Command {
    private enum State { ROTATE, APPROACH, LATERAL }
    
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean alignRight;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private State currentState = State.ROTATE;

    // Constants
    private static final double TARGET_DISTANCE = 0.3; // meters
    private static final double LATERAL_OFFSET = 0.5; // meters
    private static final double POSITION_TOLERANCE = 0.03;
    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(1.0);
    private static final double MAX_LINEAR_SPEED = 0.4;
    private static final double MAX_ANGULAR_SPEED = Math.PI/6;

    public AprilTagAlignCommand(LimelightSubsystem limelight, 
                               CommandSwerveDrivetrain drivetrain,
                               boolean alignRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.alignRight = alignRight;

        // PID Controllers
        xController = new PIDController(1.2, 0, 0.1);
        yController = new PIDController(1.0, 0, 0.1);
        thetaController = new PIDController(2.0, 0, 0.15);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.setLedMode(3);
        currentState = State.ROTATE;
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

        double[] pose = limelight.getTargetPose();
        if (pose == null) return;

        switch (currentState) {
            case ROTATE:
                handleRotatePhase(pose);
                break;
            case APPROACH:
                handleApproachPhase(pose);
                break;
            case LATERAL:
                handleLateralPhase(pose);
                break;
        }
    }

    private void handleRotatePhase(double[] pose) {
        // Get current angle from pose data
        double currentYaw = Math.toRadians(pose[2]);
        double rotationSpeed = thetaController.calculate(currentYaw, 0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );

        // Transition when facing the tag
        if (Math.abs(currentYaw) < ANGLE_TOLERANCE) {
            currentState = State.APPROACH;
            xController.setSetpoint(TARGET_DISTANCE);
        }
    }

    private void handleApproachPhase(double[] pose) {
        // Use pose X for forward distance (positive = in front of tag)
        double currentX = pose[0];
        double xSpeed = xController.calculate(currentX);
        double currentYaw = Math.toRadians(pose[2]);
        double rotationSpeed = thetaController.calculate(currentYaw, 0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(clamp(-xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withVelocityY(0)
                .withRotationalRate(clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );

        // Transition when at target distance and maintaining angle
        if (Math.abs(currentX - TARGET_DISTANCE) < POSITION_TOLERANCE && 
            Math.abs(currentYaw) < ANGLE_TOLERANCE) {
            currentState = State.LATERAL;
            yController.setSetpoint(alignRight ? -LATERAL_OFFSET : LATERAL_OFFSET);
        }
    }

    private void handleLateralPhase(double[] pose) {
        // Maintain forward distance and angle while moving laterally
        double currentX = pose[0];
        double currentY = pose[1];
        double currentYaw = Math.toRadians(pose[2]);

        double xSpeed = xController.calculate(currentX);
        double ySpeed = yController.calculate(currentY);
        double rotationSpeed = thetaController.calculate(currentYaw, 0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(clamp(-xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withVelocityY(clamp(-ySpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withRotationalRate(clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );
    }

    @Override
    public boolean isFinished() {
        return currentState == State.LATERAL && 
               yController.atSetpoint() && 
               xController.atSetpoint() && 
               thetaController.atSetpoint();
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