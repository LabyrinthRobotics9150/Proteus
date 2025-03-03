package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorManualCommand {
    public static Command moveElevator(ElevatorSubsystem elevator, double speed) {
        return new Command() {
            @Override
            public void initialize() {
                elevator.stopElevator();
            }

            @Override
            public void execute() {
                // Check limit switches and set the speed
                if ((ElevatorSubsystem.forwardLimitSwitch.isPressed() && speed > 0) || 
                    (ElevatorSubsystem.reverseLimitSwitch.isPressed() && speed < 0)) {
                    elevator.stopElevator(); 
                } else {
                    elevator.setElevatorSpeed(speed); 
                }
            }

            @Override
            public void end(boolean interrupted) {
                // Re-enable the motion profile to hold the current position
                elevator.setHeight(elevator.getHeight());
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }
}