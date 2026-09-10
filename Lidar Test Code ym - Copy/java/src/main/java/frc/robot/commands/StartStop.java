package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import frc.robot.subsystems.LidarTest;
import frc.robot.gamepad.OI;

public class StartStop extends CommandBase
{
    private static final LidarTest lidar = RobotContainer.lidar;
    private static final OI oi = RobotContainer.oi;

    public StartStop()
    {
        addRequirements(lidar);
    }

    @Override
    public void initialize()
    {
        System.out.println("Starting Lidar...");
        lidar.startScan();
    }

    @Override
    public void execute()
    {
        // Press Y to stop the Lidar command
        if (oi.getDriveYButton())
        {
            cancel();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println("Stopping Lidar...");
        lidar.stopScan();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}