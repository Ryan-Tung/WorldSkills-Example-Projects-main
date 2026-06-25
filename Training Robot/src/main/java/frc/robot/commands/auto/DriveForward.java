package frc.robot.commands.auto;

import frc.robot.commands.driveCommands.SimpleDrive;
import frc.robot.commands.driveCommands.forward;

public class DriveForward extends AutoCommand
{
    public DriveForward ()
    {
        super(new SimpleDrive(0.0, 0.0 , 0.5).withTimeout(2)); // x , y , z  //y forward //x 
        new forward(0.0, 0.0 , 0.5).withTimeout(2); // x , y , z  //y forward //x 
            
    }
}