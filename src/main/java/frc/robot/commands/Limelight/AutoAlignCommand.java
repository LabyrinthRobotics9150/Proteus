package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoAlignCommand extends Command {
    private enum AlignmentState { ROTATE, APPROACH, LATERAL }
    
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean targetRight;
    private AlignmentState currentState = AlignmentState.ROTATE;

    // PID Controllers
    private final PIDController xController = new PIDController(1.2, 0, 0.1);
    private final PIDController yController = new PIDController(1.0, 0, 0.1);
    private final PIDController thetaController = new PIDController(2.5, 0, 0.2);

    // Configuration
    private static final double TARGET_DISTANCE = 0.5; // meters
    private static final double LATERAL_OFFSET = 0.3; // meters
    private static final double MAX_LINEAR_SPEED = 0.75;
    private static final double MAX_ANGULAR_SPEED = Math.PI/4;
    private static final double POSE_TOLERANCE = 0.05;
    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(1.5);

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    public AutoAlignCommand(LimelightSubsystem limelight, 
                           CommandSwerveDrivetrain drivetrain,
                           boolean targetRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.targetRight = targetRight;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(POSE_TOLERANCE);
        yController.setTolerance(POSE_TOLERANCE);
        thetaController.setTolerance(ANGLE_TOLERANCE);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.setLedMode(3);
        currentState = AlignmentState.ROTATE;
        resetControllers();
    }

    private void resetControllers() {
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
            case APPROACH:
                handleApproach(pose);
                break;
            case ROTATE:
                handleRotation(pose);
                break;
            case LATERAL:
                handleLateral(pose);
                break;
        }
    }

    private void handleRotation(double[] pose) {
        double currentYaw = Math.toRadians(pose[2]);
        double rotationSpeed = thetaController.calculate(currentYaw, 0);
        
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );

        if (thetaController.atSetpoint()) {
            currentState = AlignmentState.APPROACH;
            xController.setSetpoint(TARGET_DISTANCE);
        }
    }

    private void handleApproach(double[] pose) {
        double currentX = pose[0];
        double currentYaw = Math.toRadians(pose[2]);
        
        double xSpeed = xController.calculate(currentX);
        double rotationSpeed = thetaController.calculate(currentYaw, 0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(clamp(-xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withVelocityY(0)
                .withRotationalRate(clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );

        if (xController.atSetpoint() && thetaController.atSetpoint()) {
            currentState = AlignmentState.LATERAL;
            yController.setSetpoint(targetRight ? -LATERAL_OFFSET : LATERAL_OFFSET);
        }
    }

    private void handleLateral(double[] pose) {
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
        return currentState == AlignmentState.LATERAL && 
               xController.atSetpoint() && 
               yController.atSetpoint() && 
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