package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.driveCommands.DriveForwardPrimitive;
import frc.robot.commands.driveCommands.SimpleDrive;
import frc.robot.commands.driveCommands.TurnToAnglePrimitive;

public class DriveForward extends AutoCommand
{
    public DriveForward ()
    {
        super(new SequentialCommandGroup(
            new DriveForwardPrimitive(1000,0),

            new WaitCommand(1),

            


            new TurnToAnglePrimitive(90)));
        
}}