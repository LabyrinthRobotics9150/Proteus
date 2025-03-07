package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Limelight.AutoAlignCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
//import frc.robot.commands.autos.ShootCommand;

public class AutonomousSequence extends SequentialCommandGroup {

    /*
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem limelight;
    private final ElevatorSubsystem m_elevator;
    private Pose2d initialPose;

    public AutonomousSequence(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, ElevatorSubsystem elevator) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.m_elevator = elevator;

        addCommands(
            // Capture initial pose
            new InstantCommand(() -> initialPose = drivetrain.getState().Pose),
            
            // Alignment phase with proper conditional structure
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new AutoAlignCommand(drivetrain, limelight),
                    new WaitCommand(2)
                ),
                new ConditionalCommand(
                    new SearchPatternCommand(drivetrain)
                        .until(() -> limelight.hasTarget())
                        .andThen(new AutoAlignCommand(drivetrain, limelight)),
                    new InstantCommand(() -> {}),  // Empty command when no search needed
                    () -> !limelight.hasTarget()
                ),
                new WaitCommand(4)
            ),
            
            // Elevator and shooting sequence
            new ElevatorRaise(m_elevator, 3.8)
                .withTimeout(3)
                .andThen(new WaitCommand(0.5)),
            
            new ParallelCommandGroup(
                new ShootCommand().withTimeout(2),
                new ElevatorRaise(m_elevator, 3.8)
            ).withTimeout(2.5),
            
            new ElevatorRaise(m_elevator, 0)
                .withTimeout(2)
                .andThen(new WaitCommand(0.25)),
            
            // Return to initial position
            new FollowPathCommand(drivetrain, initialPose)
                .withTimeout(5)
        );
    }

    // Improved SearchPatternCommand
    private static class SearchPatternCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private Command currentCommand;
        private int step = 0;
        private static final double DRIVE_DISTANCE = 1.5;
        private static final double ROTATION_ANGLE = 60;

        public SearchPatternCommand(CommandSwerveDrivetrain drivetrain) {
            this.drivetrain = drivetrain;
            addRequirements(drivetrain);
        }

        @Override
        public void initialize() {
            step = 0;
            scheduleNextCommand();
        }

        private void scheduleNextCommand() {
            if (currentCommand != null) {
                currentCommand.cancel();
            }

            switch (step % 4) {
                case 0:
                    currentCommand = new DriveForwardCommand(drivetrain, DRIVE_DISTANCE)
                        .withTimeout(2);
                    break;
                case 1:
                    currentCommand = new RotateCommand(drivetrain, ROTATION_ANGLE)
                        .withTimeout(1.5);
                    break;
                case 2:
                    currentCommand = new DriveForwardCommand(drivetrain, DRIVE_DISTANCE)
                        .withTimeout(2);
                    break;
                case 3:
                    currentCommand = new RotateCommand(drivetrain, -ROTATION_ANGLE*1.5)
                        .withTimeout(2);
                    break;
            }
            currentCommand.schedule();
            step++;
        }

        @Override
        public void execute() {
            if (currentCommand.isFinished()) {
                scheduleNextCommand();
            }
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            currentCommand.cancel();
            drivetrain.setControl(new SwerveRequest.Idle());
        }
    }

    // Enhanced DriveForwardCommand with velocity control
    private static class DriveForwardCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final PIDController xController = new PIDController(0.3, 0, 0.02);
        private final PIDController yController = new PIDController(0.3, 0, 0.02);
        private final Pose2d targetPose;

        public DriveForwardCommand(CommandSwerveDrivetrain drivetrain, double meters) {
            this.drivetrain = drivetrain;
            Pose2d current = drivetrain.getState().Pose;
            targetPose = new Pose2d(
                current.getX() + meters * current.getRotation().getCos(),
                current.getY() + meters * current.getRotation().getSin(),
                current.getRotation()
            );
            addRequirements(drivetrain);
            
            xController.setTolerance(0.05);
            yController.setTolerance(0.05);
        }

        @Override
        public void execute() {
            Pose2d current = drivetrain.getState().Pose;
            double xOutput = xController.calculate(current.getX(), targetPose.getX());
            double yOutput = yController.calculate(current.getY(), targetPose.getY());
            
            drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(xOutput)
                .withVelocityY(yOutput)
                .withRotationalRate(0));
        }

        @Override
        public boolean isFinished() {
            return xController.atSetpoint() && yController.atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.setControl(new SwerveRequest.Idle());
        }
    }

    // Enhanced RotateCommand with better PID
    private static class RotateCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final PIDController rotationController = new PIDController(0.05, 0, 0.005);
        private final double targetDegrees;

        public RotateCommand(CommandSwerveDrivetrain drivetrain, double degrees) {
            this.drivetrain = drivetrain;
            targetDegrees = drivetrain.getState().Pose.getRotation().getDegrees() + degrees;
            rotationController.enableContinuousInput(-180, 180);
            rotationController.setTolerance(1.5);
            addRequirements(drivetrain);
        }

        @Override
        public void execute() {
            double currentDegrees = drivetrain.getState().Pose.getRotation().getDegrees();
            double output = rotationController.calculate(currentDegrees, targetDegrees);
            drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(output));
        }

        @Override
        public boolean isFinished() {
            return rotationController.atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.setControl(new SwerveRequest.Idle());
        }
    }

    // Improved FollowPathCommand with trajectory-like control
    private static class FollowPathCommand extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final Pose2d targetPose;
        private final PIDController xController = new PIDController(0.4, 0, 0.03);
        private final PIDController yController = new PIDController(0.4, 0, 0.03);
        private final PIDController rotationController = new PIDController(0.1, 0, 0.01);

        public FollowPathCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
            this.drivetrain = drivetrain;
            this.targetPose = targetPose;
            rotationController.enableContinuousInput(-180, 180);
            addRequirements(drivetrain);
            
            xController.setTolerance(0.1);
            yController.setTolerance(0.1);
            rotationController.setTolerance(2.0);
        }

        @Override
        public void execute() {
            Pose2d current = drivetrain.getState().Pose;
            double xOutput = xController.calculate(current.getX(), targetPose.getX());
            double yOutput = yController.calculate(current.getY(), targetPose.getY());
            double rotationOutput = rotationController.calculate(
                current.getRotation().getDegrees(), 
                targetPose.getRotation().getDegrees()
            );
            
            drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(xOutput)
                .withVelocityY(yOutput)
                .withRotationalRate(rotationOutput));
        }

        @Override
        public boolean isFinished() {
            return xController.atSetpoint() && 
                   yController.atSetpoint() && 
                   rotationController.atSetpoint();
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.setControl(new SwerveRequest.Idle());
        }
    }
        */
}