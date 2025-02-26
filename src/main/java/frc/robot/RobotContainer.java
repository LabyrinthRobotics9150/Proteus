// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Elevator.MoveElevatorManualCommand;
import frc.robot.commands.Intake.BallCommand;
import frc.robot.commands.Intake.IntakeOuttakeCommand;
import frc.robot.commands.Intake.MovePivotManualCommand;
import frc.robot.commands.Intake.WheelMoveCommand;
import frc.robot.commands.Limelight.AprilTagAlignCommand;
import frc.robot.commands.Limelight.FollowClosestAprilTagCommand;
import frc.robot.commands.Swerve.ResetGyroCommand;
import frc.robot.commands.Swerve.SlowCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {

      // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  /* INITIALIZING ALL OBJECTS FOR COMMANDS */

  // primary
  SlowCommand slowCommand = new SlowCommand();
  IntakeOuttakeCommand intakeOuttakeCommand = new IntakeOuttakeCommand(m_intake);
  AprilTagAlignCommand alignRightCommand = new AprilTagAlignCommand( limelight, drivetrain,  true);
  AprilTagAlignCommand alignLeftCommand = new AprilTagAlignCommand(limelight, drivetrain, false);
  ResetGyroCommand resetGyroCommand = new ResetGyroCommand(/* drivetrain? maybe? */);

  // temporary
  FollowClosestAprilTagCommand closestAprilTagCommand = new FollowClosestAprilTagCommand(limelight, drivetrain);

  // secondary
  Command level4Command = m_elevator.goToHeight(176); // tune these
  Command level3Command = m_elevator.goToHeight(104.5);
  Command level2Command = m_elevator.goToHeight(59.29);
  Command level1Command = m_elevator.goToHeight(0.0);
  WheelMoveCommand wheelMoveCommand = new WheelMoveCommand(m_intake, .1);
  WheelMoveCommand wheelMoveReverseCommand = new WheelMoveCommand(m_intake, -.1);
  BallCommand ballCommand = new BallCommand(m_intake);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController m_primaryController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
  private final CommandXboxController m_secondaryController = 
    new CommandXboxController(OperatorConstants.kSecondaryControllerPort);
    // determines which commands are enabled;
    boolean TESTING_MODE = false;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-m_primaryController.getLeftY() / 10) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY((-m_primaryController.getLeftX()/ 10) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate((-m_primaryController.getRightX() / 10) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_primaryController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_primaryController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_primaryController.getLeftY() / 10, -m_primaryController.getLeftX() / 10))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_primaryController.back().and(m_primaryController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_primaryController.back().and(m_primaryController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_primaryController.start().and(m_primaryController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_primaryController.start().and(m_primaryController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_primaryController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


    if(TESTING_MODE) {
    // Left Bumper - manual pivot raise
    m_primaryController.leftBumper()
    .whileTrue(MovePivotManualCommand.movePivot(m_intake, .05));

    // Right Bumper - manual pivot lower
    m_primaryController.rightBumper()
    .whileTrue(MovePivotManualCommand.movePivot(m_intake, -.05));

    // Right Trigger - manual elevator raise
    m_primaryController.rightTrigger()
    .whileTrue(MoveElevatorManualCommand.moveElevator(m_elevator, .1));

    // left Trigger - manual elevator lower
    m_primaryController.leftTrigger()
    .whileTrue(MoveElevatorManualCommand.moveElevator(m_elevator, -.02));

    m_primaryController.a()
    .whileTrue(wheelMoveCommand);

    m_primaryController.x()
    .whileTrue(wheelMoveReverseCommand);

    m_primaryController.y()
    .whileTrue(closestAprilTagCommand);

    m_primaryController.b()
    .whileTrue(alignRightCommand);

    } else {
    /* PRIMARY */

    // Left Trigger - slow mode
    m_primaryController.leftTrigger()
    .whileTrue(slowCommand);

    // Right Trigger - intake / outtake dependant on where the pivot arm is
    m_primaryController.rightTrigger()
    .onTrue(intakeOuttakeCommand);

    // Rb - Auto-align to right coral spoke
    m_primaryController.rightBumper()
    .onTrue(alignRightCommand);

    // Lb - Auto-align to left coral spoke
    m_primaryController.leftBumper()
    .onTrue(alignLeftCommand);

    /*
      new Trigger(limelight::hasTarget)
          .and(m_primaryController.rightBumper())
          .whileTrue(rb_Command);
  }  
     */

    // x - resets gyro
    m_primaryController.x()
    .onTrue(resetGyroCommand);

    // B - Ball command
    m_primaryController.b()
    .whileTrue(ballCommand);
    }


    /* SECONDARY */

    // elevator heights

    // A - Level 4
    m_secondaryController.a()
    .whileTrue(level4Command);

    // B - Level 3
    m_secondaryController.b()
    .whileTrue(level3Command);

    // X - Level 2
    m_secondaryController.x()
    .whileTrue(level2Command);

    // Y - Level 1
    m_secondaryController.y()
    .whileTrue(level1Command);

    // RT - intake in
    m_secondaryController.rightTrigger()
    .whileTrue(wheelMoveCommand);
}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
