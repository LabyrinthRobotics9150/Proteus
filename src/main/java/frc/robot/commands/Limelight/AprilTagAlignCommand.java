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
    private boolean pipelineValid = false;
    private int pipelineSetAttempts = 0;
    private static final int MAX_PIPELINE_ATTEMPTS = 10;

    private static final double DESIRED_OFFSET_METERS = 0.5;
    private static final double Y_TOLERANCE_METERS = 0.05;
    private static final double THETA_TOLERANCE_RADIANS = Units.degreesToRadians(2);

    public AprilTagAlignCommand(LimelightSubsystem limelight, 
                               CommandSwerveDrivetrain drivetrain,
                               boolean alignRight) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.alignRight = alignRight;


        yController = new PIDController(2.0, 0.0, 0.1);
        yController.setTolerance(Y_TOLERANCE_METERS);

        thetaController = new PIDController(3.0, 0.0, 0.2);
        thetaController.setTolerance(THETA_TOLERANCE_RADIANS);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){ 
        // Force configure Limelight state
        limelight.setPipeline(0);
        limelight.setLedMode(3);  // Force LEDs on
        pipelineValid = false;
        pipelineSetAttempts = 0;

        // Reset controllers
        yController.reset();
        thetaController.reset();
        double desiredY = alignRight ? -DESIRED_OFFSET_METERS : DESIRED_OFFSET_METERS;
        yController.setSetpoint(desiredY);
        thetaController.setSetpoint(0);
    }

    @Override
    public void execute() {
       // Verify pipeline before proceeding
       if (!pipelineValid) {
        pipelineSetAttempts++;
        if (limelight.getCurrentPipeline() == 0) {
            pipelineValid = true;
            System.out.println("Pipeline 0 confirmed after " + pipelineSetAttempts + " attempts");
        } else if (pipelineSetAttempts <= MAX_PIPELINE_ATTEMPTS) {
            limelight.setPipeline(0); // Resend pipeline command
            return; // Skip this cycle
        } else {
            System.out.println("Failed to set pipeline after " + MAX_PIPELINE_ATTEMPTS + " attempts");
            cancel();
            return;
        }
    }
    if (!limelight.hasTarget()) {
        drivetrain.setControl(new SwerveRequest.Idle());
        return;
    }
    // if we are on correct pipeline and it has target, get pose data
    double[] pose = limelight.getTargetPose();
        double currentY = pose[1];
        double yawDegrees = pose[2]; // Get yaw from pose data
    
        // Calculate lateral speed to reach desired offset
        double ySpeed = yController.calculate(currentY);
    
        // Convert field-relative Y movement to robot-centric velocities
        double yawRadians = Math.toRadians(yawDegrees);
        double vxRobot = -ySpeed * Math.sin(yawRadians);
        double vyRobot = ySpeed * Math.cos(yawRadians);
    
        // Calculate rotational speed to center the tag
        double txDegrees = limelight.getTargetX();
        double thetaSpeed = thetaController.calculate(Units.degreesToRadians(txDegrees));
    
        drivetrain.setControl(
            driveRequest
                .withVelocityX(vxRobot) // Adjusted based on yaw
                .withVelocityY(vyRobot) // Adjusted based on yaw
                .withRotationalRate(thetaSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        return yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}