package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LidarTest;
import frc.robot.subsystems.CobraTest;

import frc.robot.gamepad.OI;

public class RobotContainer
{
    // =====================================================
    // SUBSYSTEMS
    // =====================================================

    public static final DriveTrain driveTrain =
            new DriveTrain();

    public static final LidarTest lidar =
            new LidarTest();

    public static final CobraTest cobra =
            new CobraTest();

    public static OI oi;


    // =====================================================
    // AUTO / MODE CHOOSER
    // =====================================================

    public static SendableChooser<String> autoChooser =
            new SendableChooser<String>();

    public static HashMap<String, CommandBase> autoMode =
            new HashMap<String, CommandBase>();


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public RobotContainer()
    {
        oi = new OI();

        /*
         * IMPORTANT:
         *
         * Do NOT use:
         *
         * lidar.setDefaultCommand(new StartStop());
         *
         * LidarTest already starts the LiDAR by itself.
         */


        /*
         * CobraTest is also initialized automatically
         * when RobotContainer is created.
         *
         * You do not need to start it manually here.
         */
    }
}