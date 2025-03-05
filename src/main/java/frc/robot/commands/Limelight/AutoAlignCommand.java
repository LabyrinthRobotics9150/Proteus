package frc.robot.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoAlignCommand extends Command {
    private enum AlignmentState { ROTATE, APPROACH, LATERAL }
    
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final boolean targetRight;
    private AlignmentState currentState = AlignmentState.ROTATE;

    // PID Controllers
    private final PIDController xController = new PIDController(0.4, 0, 0.0006);
    private final PIDController yController = new PIDController(0.3, 0, 0);
    private final PIDController thetaController = new PIDController(0.05, 0, 0.001);

    // Configuration
    private static final double TARGET_DISTANCE = 0.5; // meters
    private static final double LATERAL_OFFSET = 0.3; // meters
    private static final double MAX_LINEAR_SPEED = 4.0; // m/s
    private static final double MAX_ANGULAR_SPEED = Math.PI; // rad/s
    private static final double POSE_TOLERANCE = 0.01;
    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(0.5);

    private final SwerveRequest.RobotCentric driveRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public AutoAlignCommand(LimelightSubsystem limelight, 
                          CommandSwerveDrivetrain drivetrain,
                          boolean targetRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.targetRight = targetRight;

        configureControllers();
        addRequirements(drivetrain);
    }

    private void configureControllers() {
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        xController.setTolerance(POSE_TOLERANCE);
        yController.setTolerance(POSE_TOLERANCE);
        thetaController.setTolerance(ANGLE_TOLERANCE);
    }

    @Override
    public void initialize() {
        limelight.setPipeline(0);
        limelight.setLedMode(0);
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
            drivetrain.setControl(idleRequest);
            return;
        }

        var targetPose = limelight.getTargetPose();
        if (targetPose == null) return;

        double currentX = targetPose[0];
        double currentY = targetPose[1];
        double currentYaw = Math.toRadians(targetPose[2]);

        switch (currentState) {
<<<<<<< HEAD
            case ROTATE -> handleRotation(currentX, currentY, currentYaw);
            case APPROACH -> handleApproach(currentX, currentY, currentYaw);
            case LATERAL -> handleLateral(currentX, currentY, currentYaw);
=======
            case ROTATE:
                handleRotation(pose);
                break;
            //case APPROACH:
            //    handleApproach(pose);
            //    break;
            //case LATERAL:
            //    handleLateral(pose);
            //    break;
>>>>>>> b14315cdf538300819d773ba579fc973a41d8ace
        }

        updateTelemetry();
    }

    private void handleRotation(double x, double y, double yaw) {
        xController.setSetpoint(0);
        yController.setSetpoint(0);
        thetaController.setSetpoint(0);

        double xSpeed = xController.calculate(x);
        double ySpeed = yController.calculate(y);
        double rotation = thetaController.calculate(yaw);

<<<<<<< HEAD
        applyMovement(-xSpeed, -ySpeed, rotation);
        
        if (allControllersAtSetpoint()) {
=======
        drivetrain.setControl(
            driveRequest
                .withVelocityX(clamp(xSpeed, MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withVelocityY(clamp(ySpeed, MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withRotationalRate(clamp(rotationSpeed, MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );

        if (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) {
>>>>>>> b14315cdf538300819d773ba579fc973a41d8ace
            currentState = AlignmentState.APPROACH;
            xController.setSetpoint(TARGET_DISTANCE);
        }
    }

    private void handleApproach(double x, double y, double yaw) {
        yController.setSetpoint(0);
        thetaController.setSetpoint(0);

        double xSpeed = xController.calculate(x);
        double ySpeed = yController.calculate(y);
        double rotation = thetaController.calculate(yaw);

        applyMovement(-xSpeed, -ySpeed, rotation);

        if (allControllersAtSetpoint()) {
            currentState = AlignmentState.LATERAL;
            yController.setSetpoint(targetRight ? -LATERAL_OFFSET : LATERAL_OFFSET);
        }
    }

    private void handleLateral(double x, double y, double yaw) {
        xController.setSetpoint(TARGET_DISTANCE);
        thetaController.setSetpoint(0);

        double xSpeed = xController.calculate(x);
        double ySpeed = yController.calculate(y);
        double rotation = thetaController.calculate(yaw);

        applyMovement(-xSpeed, -ySpeed, rotation);
    }

    private void applyMovement(double xSpeed, double ySpeed, double rotation) {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(clamp(xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withVelocityY(clamp(ySpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED))
                .withRotationalRate(clamp(rotation, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
        );
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Alignment/X Error", xController.getPositionError());
        SmartDashboard.putNumber("Alignment/Y Error", yController.getPositionError());
        SmartDashboard.putNumber("Alignment/Theta Error", thetaController.getPositionError());
        SmartDashboard.putString("Alignment/State", currentState.name());
    }

    private boolean allControllersAtSetpoint() {
        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               thetaController.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        return currentState == AlignmentState.LATERAL && allControllersAtSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(idleRequest);
        limelight.setLedMode(1);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}