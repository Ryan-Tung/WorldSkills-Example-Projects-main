package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveCommands.DriveWithPID;
import frc.robot.commands.driveCommands.TurnWithPID;


public class DriveSquareWithPID extends AutoCommand {
    
    public DriveSquareWithPID() {
        // 1. Create the main sequential group
        super(new SequentialCommandGroup());
        
        // 2. Loop 4 times to add the 4 sides of the square
        for (int i = 0; i < 4; i++) {
            addCommands(
                // Drive forward
                new DriveWithPID(300, 1, 0, 1).withTimeout(2),
                
                // Turn 90 degrees
                new TurnWithPID(0, 10, 90, 1).withTimeout(2)
            );
        }
    }
}