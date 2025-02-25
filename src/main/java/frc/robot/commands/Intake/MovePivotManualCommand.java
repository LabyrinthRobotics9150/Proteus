package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class MovePivotManualCommand {
    public static Command movePivot(IntakeSubsystem intake, double speed) {
        return new Command() {
            private double previousTarget; 

            @Override
            public void initialize() {
                previousTarget = intake.getTargetPosition();

                intake.setHeight(intake.getHeight());
            }

            @Override
            public void execute() {
                // Set the pivot motor speed directly
                intake.IntakePivotMotor.set(speed);
            }

            @Override
            public void end(boolean interrupted) {
            // Stop & Restore the previous target position to resume the motion profile
                intake.stopPivot();

                intake.setHeight(previousTarget);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }
}