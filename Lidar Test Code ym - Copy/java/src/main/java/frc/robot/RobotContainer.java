package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LidarTest;
import frc.robot.subsystems.CobraTest;
import frc.robot.subsystems.LidarLocalization;

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


    public static final LidarLocalization lidarLocalization =
            new LidarLocalization();


    // =====================================================
    // OPERATOR INTERFACE
    // =====================================================

    public static OI oi;


    // =====================================================
    // MODE CHOOSER
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
        oi =
                new OI();


        /*
         * LidarTest already starts the LiDAR.
         *
         * Do NOT use:
         *
         * lidar.setDefaultCommand(new StartStop());
         */


        /*
         * CobraTest is initialized automatically.
         */


        /*
         * LidarLocalization is also initialized
         * automatically.
         *
         * It reads localization values from
         * NetworkTables that will be sent by Python.
         */
    }
}