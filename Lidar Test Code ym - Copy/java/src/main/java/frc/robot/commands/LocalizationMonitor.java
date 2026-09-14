package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.LidarLocalization;


public class LocalizationMonitor extends CommandBase
{
    private static final LidarLocalization localization =
            RobotContainer.lidarLocalization;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public LocalizationMonitor()
    {
        /*
         * Do NOT add DriveTrain here.
         *
         * This command only monitors LiDAR localization.
         */
        addRequirements(
                localization
        );
    }


    // =====================================================
    // INITIALIZE
    // =====================================================

    @Override
    public void initialize()
    {
        localization.startMapping();


        SmartDashboard.putString(
                "Localization State",
                "RUNNING"
        );
    }


    // =====================================================
    // EXECUTE
    // =====================================================

    @Override
    public void execute()
    {
        // =================================================
        // ROBOT POSITION
        // =================================================

        double robotX =
                localization.getRobotX();


        double robotY =
                localization.getRobotY();


        double heading =
                localization.getRobotHeading();


        // =================================================
        // NEAREST OBJECT
        // =================================================

        double obstacleX =
                localization.getObstacleX();


        double obstacleY =
                localization.getObstacleY();


        double obstacleDistance =
                localization.getObstacleDistance();


        double obstacleAngle =
                localization.getObstacleAngle();


        // =================================================
        // HUD VALUES
        // =================================================

        SmartDashboard.putNumber(
                "HUD Robot X",
                robotX
        );


        SmartDashboard.putNumber(
                "HUD Robot Y",
                robotY
        );


        SmartDashboard.putNumber(
                "HUD Heading",
                heading
        );


        SmartDashboard.putNumber(
                "HUD Obstacle X",
                obstacleX
        );


        SmartDashboard.putNumber(
                "HUD Obstacle Y",
                obstacleY
        );


        SmartDashboard.putNumber(
                "HUD Obstacle Distance",
                obstacleDistance
        );


        SmartDashboard.putNumber(
                "HUD Obstacle Angle",
                obstacleAngle
        );


        SmartDashboard.putBoolean(
                "HUD Localization Valid",
                localization.isLocalizationValid()
        );
    }


    // =====================================================
    // FINISH?
    // =====================================================

    @Override
    public boolean isFinished()
    {
        return false;
    }


    // =====================================================
    // END
    // =====================================================

    @Override
    public void end(
            boolean interrupted)
    {
        localization.stopMapping();


        SmartDashboard.putString(
                "Localization State",
                "STOPPED"
        );
    }
}