package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.SimpleDrive;
import frc.robot.commands.driveCommands.forward;
import frc.robot.commands.driveCommands.heading;
import frc.robot.commands.driveCommands.DriveWithPID;


public class DriveForward extends AutoCommand
{
    public DriveForward ()
    {
        super(new SequentialCommandGroup( 
        // x , y , z  //y forward //x 

        new forward(-500,0),
       
        new WaitCommand(1)));
        
    }
}