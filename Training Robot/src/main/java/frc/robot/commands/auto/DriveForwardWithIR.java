package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.driveCommands.DriveForwardPrimitive;
import frc.robot.commands.driveCommands.SimpleDrive;
import frc.robot.commands.driveCommands.TurnToAnglePrimitive;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardWithIR extends SequentialCommandGroup {
    // Add the DriveTrain parameter here
    public DriveForwardWithIR(DriveTrain driveTrain) {
        
        addCommands(
            new DriveForwardPrimitive(500, 0),
            new TurnToAnglePrimitive(90),
            new DriveForwardPrimitive(500, -50),
            new WaitCommand(0.25),
            new TurnToAnglePrimitive(-140)
        );
    
            }
        




    }
