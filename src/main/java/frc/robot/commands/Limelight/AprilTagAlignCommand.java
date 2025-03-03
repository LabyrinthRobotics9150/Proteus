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
    
    // Minimum command thresholds to overcome static friction.
    private static final double MIN_LINEAR_COMMAND = 0.05;
    private static final double MIN_ANGULAR_COMMAND = 0.05;
    
    // Safety speed limits.
    private static final double MAX_LINEAR_SPEED = 0.4;
    private static final double MAX_ANGULAR_SPEED = Math.PI / 6;
    
    // Target and tolerance constants.
    // Note: With your drivetrain’s coordinate system (see CommandSwerveDrivetrain),
    // objects in front of the robot typically yield a negative X value.
    // Therefore, we set the desired X (forward) distance to -0.3 m.
    private static final double TARGET_DISTANCE = -0.3; // meters
    private static final double LATERAL_OFFSET = 0.5;     // meters
    private static final double POSITION_TOLERANCE = 0.03;
    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(1.0);
    
    private State currentState = State.ROTATE;

    public AprilTagAlignCommand(LimelightSubsystem limelight, 
                                CommandSwerveDrivetrain drivetrain,
                                boolean alignRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.alignRight = alignRight;
        
        // Initialize PID controllers with tuned gains.
        // (These gains may require further tuning for your system.)
        xController = new PIDController(1.2, 0, 0.1);
        yController = new PIDController(1.0, 0, 0.1);
        thetaController = new PIDController(2.0, 0, 0.15);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Set tolerances so that atSetpoint() reliably indicates when the error is small.
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        thetaController.setTolerance(ANGLE_TOLERANCE);
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.setLedMode(1);
        currentState = State.ROTATE;
        xController.reset();
        yController.reset();
        thetaController.reset();
        // For rotation, we want to face the target (i.e. 0 yaw error).
        thetaController.setSetpoint(0);
    }

    @Override
    public void execute() {
        if (!limelight.hasTarget()) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        double[] pose = limelight.getTargetPose();
        if (pose == null) return;

        // The LimelightSubsystem returns:
        //   pose[0] = X (meters) — forward distance in robot space,
        //   pose[1] = Y (meters) — lateral offset in robot space,
        //   pose[2] = Yaw (degrees).
        double currentX = pose[0];
        double currentY = pose[1];
        double currentYaw = Math.toRadians(pose[2]);

        switch (currentState) {
            case ROTATE:
                handleRotatePhase(currentYaw);
                break;
            case APPROACH:
                handleApproachPhase(currentX, currentYaw);
                break;
            case LATERAL:
                handleLateralPhase(currentX, currentY, currentYaw);
                break;
        }
    }

    private void handleRotatePhase(double currentYaw) {
        double rotationSpeed = thetaController.calculate(currentYaw);
        rotationSpeed = applyMinCommand(rotationSpeed, MIN_ANGULAR_COMMAND);
        rotationSpeed = clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        drivetrain.setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed)
        );

        if (thetaController.atSetpoint()) {
            currentState = State.APPROACH;
            // In approach, we want to drive until X (forward) equals TARGET_DISTANCE.
            xController.setSetpoint(TARGET_DISTANCE);
        }
    }

    private void handleApproachPhase(double currentX, double currentYaw) {
        double xSpeed = xController.calculate(currentX);
        xSpeed = applyMinCommand(xSpeed, MIN_LINEAR_COMMAND);
        xSpeed = clamp(xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

        double rotationSpeed = thetaController.calculate(currentYaw);
        rotationSpeed = applyMinCommand(rotationSpeed, MIN_ANGULAR_COMMAND);
        rotationSpeed = clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        drivetrain.setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(xSpeed)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed)
        );

        if (Math.abs(currentX - TARGET_DISTANCE) < POSITION_TOLERANCE &&
            Math.abs(currentYaw) < ANGLE_TOLERANCE) {
            currentState = State.LATERAL;
            // Set lateral setpoint; note the sign is chosen based on whether you want to align right or left.
            yController.setSetpoint(alignRight ? -LATERAL_OFFSET : LATERAL_OFFSET);
        }
    }

    private void handleLateralPhase(double currentX, double currentY, double currentYaw) {
        double xSpeed = xController.calculate(currentX);
        xSpeed = applyMinCommand(xSpeed, MIN_LINEAR_COMMAND);
        xSpeed = clamp(xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

        double ySpeed = yController.calculate(currentY);
        ySpeed = applyMinCommand(ySpeed, MIN_LINEAR_COMMAND);
        ySpeed = clamp(ySpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);

        double rotationSpeed = thetaController.calculate(currentYaw);
        rotationSpeed = applyMinCommand(rotationSpeed, MIN_ANGULAR_COMMAND);
        rotationSpeed = clamp(rotationSpeed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        drivetrain.setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        return currentState == State.LATERAL &&
               xController.atSetpoint() &&
               yController.atSetpoint() &&
               thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
        limelight.setLedMode(1);
    }

    /**
     * Applies a minimum command threshold: if the PID output is nonzero but its magnitude is less
     * than the minimum, it is bumped up to that minimum value (preserving its sign).
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
