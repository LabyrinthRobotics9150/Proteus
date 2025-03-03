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
    private final boolean alignRight;
    private final PIDController yController;
    private final PIDController thetaController;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // Updated constants
    private static final double DESIRED_OFFSET_METERS = 0.3;
    private static final double Y_TOLERANCE_METERS = 0.03;
    private static final double THETA_TOLERANCE_RADIANS = Units.degreesToRadians(1.5);
    private static final double MAX_SPEED = 0.8;  // m/s
    private static final double MAX_ANGULAR_SPEED = Math.PI / 4;  // rad/s

    public AprilTagAlignCommand(LimelightSubsystem limelight, 
                               CommandSwerveDrivetrain drivetrain,
                               boolean alignRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.alignRight = alignRight;

        // Reduced gains for safer movement
        yController = new PIDController(0.8, 0.0, 0.05);
        yController.setTolerance(Y_TOLERANCE_METERS);

        // Now controls heading alignment to AprilTag using pose yaw
        thetaController = new PIDController(1.5, 0.0, 0.1);
        thetaController.setTolerance(THETA_TOLERANCE_RADIANS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() { 
        limelight.setPipeline(0);
        limelight.setLedMode(3);

        yController.reset();
        thetaController.reset();
        double desiredY = alignRight ? -DESIRED_OFFSET_METERS : DESIRED_OFFSET_METERS;
        yController.setSetpoint(desiredY);
        thetaController.setSetpoint(0);  // Align to face the AprilTag directly
    }

    @Override
    public void execute() {
        // Immediately stop if target lost
        if (!limelight.hasTarget()) {
            drivetrain.setControl(new SwerveRequest.Idle());
            return;
        }

        double[] pose = limelight.getTargetPose();
        if (pose == null) return;

        double currentY = pose[1];
        double yawRadians = Math.toRadians(pose[2]);

        // Calculate speeds with clamping
        double ySpeed = clamp(yController.calculate(currentY), -MAX_SPEED, MAX_SPEED);
        
        // Align to face the AprilTag directly using pose yaw
        double thetaSpeed = clamp(thetaController.calculate(yawRadians), 
                               -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        // Convert to robot-centric velocities using current heading
        double vxRobot = -ySpeed * Math.sin(yawRadians);
        double vyRobot = ySpeed * Math.cos(yawRadians);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(vxRobot)
                .withVelocityY(vyRobot)
                .withRotationalRate(thetaSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        // Only finish when both controllers are settled AND still seeing target
        return limelight.hasTarget() && 
               yController.atSetpoint() && 
               thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
        limelight.setLedMode(1);  // Return to pipeline-controlled LED mode
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}