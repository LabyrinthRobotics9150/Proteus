package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AprilTagAlignCommand extends Command {
    private enum State { APPROACH, LATERAL }
    
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean alignRight;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();
    private State currentState = State.APPROACH;

    // Constants
    private static final double TARGET_DISTANCE = 0.3; // meters
    private static final double LATERAL_OFFSET = 0.5; // meters
    private static final double POSITION_TOLERANCE = 0.05;
    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(2);
    private static final double MAX_SPEED = 0.5;
    private static final double CAMERA_HEIGHT = 1.0; // meters
    private static final double TARGET_HEIGHT = 1.5; // meters
    private static final double CAMERA_PITCH = Units.degreesToRadians(25);

    public AprilTagAlignCommand(LimelightSubsystem limelight, 
                               CommandSwerveDrivetrain drivetrain,
                               boolean alignRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.alignRight = alignRight;

        // Controllers for approach phase
        xController = new PIDController(1.2, 0, 0.1);
        thetaController = new PIDController(3.0, 0, 0.2);
        thetaController.setTolerance(ANGLE_TOLERANCE);
        
        // Controller for lateral phase
        yController = new PIDController(1.0, 0, 0.1);
        yController.setTolerance(POSITION_TOLERANCE);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.setLedMode(3);
        currentState = State.APPROACH;
        
        // Reset all controllers
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
                handleApproachPhase(pose);
                break;
            case LATERAL:
                handleLateralPhase(pose);
                break;
        }
    }

    private void handleApproachPhase(double[] pose) {
        // Calculate distance using target height and camera geometry
        double verticalAngle = Units.degreesToRadians(limelight.getTargetY()) + CAMERA_PITCH;
        double distance = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(verticalAngle);
        
        // Calculate forward speed to maintain target distance
        double xSpeed = xController.calculate(distance, TARGET_DISTANCE);
        
        // Calculate rotation to center target (tx = 0)
        double tx = limelight.getTargetX();
        double rotationSpeed = thetaController.calculate(Units.degreesToRadians(tx), 0);

        // Apply movement
        drivetrain.setControl(
            driveRequest
                .withVelocityX(clamp(xSpeed, -MAX_SPEED, MAX_SPEED))
                .withVelocityY(0)
                .withRotationalRate(clamp(rotationSpeed, -MAX_SPEED, MAX_SPEED))
        );

        // Transition to lateral phase when aligned and at distance
        if (Math.abs(distance - TARGET_DISTANCE) < POSITION_TOLERANCE 
            && thetaController.atSetpoint()) {
            currentState = State.LATERAL;
            yController.setSetpoint(alignRight ? -LATERAL_OFFSET : LATERAL_OFFSET);
        }
    }

    private void handleLateralPhase(double[] pose) {
        // Current lateral position from AprilTag (pose[1])
        double currentY = pose[1];
        double ySpeed = yController.calculate(currentY);
        
        // Maintain facing direction toward AprilTag
        double tx = limelight.getTargetX();
        double rotationSpeed = thetaController.calculate(Units.degreesToRadians(tx), 0);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(clamp(ySpeed, -MAX_SPEED, MAX_SPEED))
                .withRotationalRate(clamp(rotationSpeed, -MAX_SPEED, MAX_SPEED))
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