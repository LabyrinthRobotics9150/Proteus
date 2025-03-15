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

    private static TrapezoidProfile.Constraints Rotationconstraints = new TrapezoidProfile.Constraints(1, 5);
    private static TrapezoidProfile.Constraints xyconstraints = new TrapezoidProfile.Constraints(.03, 5);

    private boolean doneWithInitial;
    private boolean initialY;
    private boolean initialR;

    
    
    private ProfiledPIDController rotationalPidController = 
    new ProfiledPIDController(7, 0.0, 0.0, Rotationconstraints);
    private ProfiledPIDController xPidController = 
    new ProfiledPIDController(.7, 0.0, 0.0, xyconstraints);
    private ProfiledPIDController yPidController = 
    new ProfiledPIDController(2, 0.0, 0.0, xyconstraints);

    
    
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

        doneWithInitial = false;
        initialY = false;
        initialR = false;

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
            yOffset = .29;
        } else {
            yOffset = -.29;
        }
        xOffset = 0;
        rOffset = 0;
        doneWithInitial = false;
        initialY = false;
        initialR = false;

        addRequirements(m_Limelight);
        addRequirements(m_drivetrain);
    }
    
    @Override
    public void initialize() {

        tagID = -1;
        doneWithInitial = false;
        initialY = false;
        initialR = false;
        // set goals of pid controllers
        rotationalPidController.setGoal(0);
        xPidController.setGoal(xOffset);
        yPidController.setGoal(0);
        rotationalPidController.setTolerance(Math.toRadians(.5));
        try {
            // Choose the closest fiducial as the target.
            tagID = m_Limelight.getClosestFiducial().id;
            System.out.println(tagID);
        } catch (VisionSubsystem.NoSuchTargetException e) {
            tagID = -1;
        }
    }
    
    @Override
    public void execute() {
        SmartDashboard.putBoolean("doneWithInitial", doneWithInitial);
        SmartDashboard.putData(rotationalPidController);
        SmartDashboard.putNumber("rotation error", rotationalPidController.getPositionError());;
        fiducial = new RawFiducial(tagID, outputY, outputX, outputRotation, MIN_ROTATION_OUTPUT_DEG, MIN_LATERAL_OUTPUT, MIN_DRIVE_X_OUTPUT);

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
            if (!doneWithInitial) {
                System.out.println("initialR: " + initialR + " tx: " + fiducial.txnc);
                if (!initialR) {
                    outputRotation = rotationalPidController.calculate(Units.degreesToRadians(fiducial.txnc));
                    if (rotationalPidController.atGoal()) {initialR = true; System.out.println("GOAL VALUE: " + Units.degreesToRadians(fiducial.txnc));}
                }

                if (!initialY && initialR) {
                    // lateral error
                    double lateralError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
                    outputY = yPidController.calculate(lateralError);
                    if (yPidController.atGoal()) {initialY = true;}
                    
                } else {
                    outputRotation = rotationalPidController.calculate(Units.degreesToRadians(fiducial.txnc), 0);
                    if (rotationalPidController.atGoal() && yPidController.atGoal()) {doneWithInitial = true;}
                }
            } else {
            double lateralError = fiducial.distToRobot * Math.sin(Units.degreesToRadians(fiducial.txnc));
            lateralError += yOffset;
            outputY = yPidController.calculate(lateralError);
        
            // x stuff
            outputX = -xPidController.calculate(fiducial.distToRobot);

            }
        
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
        }
        return xPidController.atGoal();
    }
    
    @Override
    public void end(boolean interrupted) {
        tagID = -1;
        doneWithInitial = false;
        initialY = false;
        initialR = false;
        System.out.println("end command");
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
