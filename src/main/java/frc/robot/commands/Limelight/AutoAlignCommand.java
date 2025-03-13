package frc.robot.commands.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;


public class AutoAlignCommand extends Command {
    protected final CommandSwerveDrivetrain m_drivetrain;
    protected final VisionSubsystem m_Limelight;

    private static TrapezoidProfile.Constraints Rotationconstraints = new TrapezoidProfile.Constraints(4, 10);
    private static TrapezoidProfile.Constraints xyconstraints = new TrapezoidProfile.Constraints(.05, 10);

    private boolean donewithRotation;
    private boolean initialY;

    
    
    private static ProfiledPIDController rotationalPidController = 
    new ProfiledPIDController(6, 0.0, 0.0, Rotationconstraints);
    private static ProfiledPIDController xPidController = 
    new ProfiledPIDController(.7, 0.0, 0.0, xyconstraints);
    private static ProfiledPIDController yPidController = 
    new ProfiledPIDController(1.3, 0.0, 0.0, xyconstraints);

    
    
    // Using a robot-centric control request.
    private static final SwerveRequest.RobotCentric alignRequest = 
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private static final SwerveRequest.RobotCentric touchRequest = 
    new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    
    // Track the chosen fiducial by ID (if available)
    private static int tagID = -1;
    // Lateral offset (in meters); for left/right alignment, set via the alternate constructor.
    private double yOffset;
    private double xOffset;
    private double rOffset;
    
    // Allowed fiducial IDs for auto-alignment.
    private static final int[] ALLOWED_TAG_IDS = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};

    private static double outputX;
    private static double outputY;
    private static double outputRotation;
    
    
    // Minimum command thresholds:
    private static final double MIN_ROTATION_OUTPUT_DEG = 1; // Minimum rotational output in degrees per second
    private static final double MIN_LATERAL_OUTPUT = 0.05;     // Minimum lateral output in m/s
    private static final double MIN_DRIVE_X_OUTPUT = 0.05;       // Minimum forward/backward output in m/s
    private static RawFiducial fiducial;
    
    // Constructor for central alignment.
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        // set offsets
        yOffset = 0;
        xOffset = 0;
        rOffset = 0;

        donewithRotation = false;
        initialY = false;

        // set requirements
        addRequirements(m_Limelight);
        addRequirements(m_drivetrain);
    }
    
    // Constructor for left/right alignment. For right alignment, use a small positive lateral offset;
    // for left alignment, a small negative offset.
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, boolean rightAlign) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        // set offsets
        yOffset = 0;
        if (rightAlign) {
            yOffset = .4;
        } else {
            yOffset = -.4;
        }
        xOffset = 0;
        rOffset = 0;
        donewithRotation = false;
        initialY = false;

        addRequirements(m_Limelight);
        addRequirements(m_drivetrain);
    }
    
    @Override
    public void initialize() {
        // set goals of pid controllers
        rotationalPidController.setGoal(rOffset);
        xPidController.setGoal(xOffset);
        yPidController.setGoal(0);
        donewithRotation = false;

        try {
            // Choose the closest fiducial as the target.
            tagID = m_Limelight.getClosestFiducial().id;
        } catch (VisionSubsystem.NoSuchTargetException e) {
            tagID = -1;
        }
    }
    
    @Override
    public void execute() {
        fiducial = new RawFiducial(tagID, outputY, outputX, outputRotation, MIN_ROTATION_OUTPUT_DEG, MIN_LATERAL_OUTPUT, MIN_DRIVE_X_OUTPUT);
        // try new method
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);        
        try {
            fiducial = m_Limelight.getFiducialWithId(tagID);
            boolean allowed = false;
            for (int id : ALLOWED_TAG_IDS) {
                if (fiducial.id == id) {
                    allowed = true;
                    break;
                }
            }
            if (!allowed) {
                m_drivetrain.setControl(idleRequest);
                return;
            }
        } catch (VisionSubsystem.NoSuchTargetException e) {
            end(false);
            return;
    }

    // recalculate velocitys each loop
    outputX = 0.0;
    outputY = 0.0;
    outputRotation = 0.0;

        // rotate stuff
        if (rotationalPidController.atGoal()) {}
        outputRotation = rotationalPidController.calculate(Units.degreesToRadians(tx));   


        double lateralError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
        lateralError += yOffset;
        outputY = yPidController.calculate(lateralError);
    
        // x stuff
        outputX = -xPidController.calculate(fiducial.distToRobot);


        // Send the calculated control outputs.
        m_drivetrain.setControl(
            alignRequest
                .withVelocityX(outputX)
                .withVelocityY(outputY)
                .withRotationalRate(outputRotation)
        );
}
    
    @Override
    public boolean isFinished() {
        if (xPidController.atGoal()) {
            m_drivetrain.setControl(idleRequest);
            System.out.println("finished");
        }
        return xPidController.atGoal();
    }
    
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            if (!xPidController.atGoal()) {
                m_drivetrain.setControl(
                    touchRequest
                        .withVelocityX(outputX)
                );
            }
        } else {
            m_drivetrain.setControl(idleRequest);
        }
    }

    }
