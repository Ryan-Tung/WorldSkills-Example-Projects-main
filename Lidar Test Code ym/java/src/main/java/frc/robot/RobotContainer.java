package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LidarTest;
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

    public static OI oi;


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
    }
}