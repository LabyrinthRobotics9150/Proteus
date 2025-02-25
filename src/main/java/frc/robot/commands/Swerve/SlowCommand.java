package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;


public class SlowCommand extends Command {

    public SlowCommand(/* Drivetrain drivetrain, */) {
        //this.drivetrain = drivetrain;

        //addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // lower the speed of the robot's turning by dropping swerve coefficients.

    }

    @Override
    public void end(boolean interrupted) {
        // when command ends, return coefficients to normal.
    }

    @Override
    public boolean isFinished() {
        // keep command going until button is released.
        return false; 
    }
}