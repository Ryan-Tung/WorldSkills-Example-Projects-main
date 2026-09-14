package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LidarLocalization extends SubsystemBase
{
    // =====================================================
    // NETWORKTABLE
    // =====================================================

    private NetworkTable localizationTable;


    // =====================================================
    // ROBOT POSE ENTRIES
    // =====================================================

    private NetworkTableEntry robotXEntry;
    private NetworkTableEntry robotYEntry;
    private NetworkTableEntry robotHeadingEntry;


    // =====================================================
    // OBSTACLE ENTRIES
    // =====================================================

    private NetworkTableEntry obstacleXEntry;
    private NetworkTableEntry obstacleYEntry;

    private NetworkTableEntry obstacleDistanceEntry;
    private NetworkTableEntry obstacleAngleEntry;


    // =====================================================
    // LOCALIZATION STATUS
    // =====================================================

    private NetworkTableEntry localizationValidEntry;


    // =====================================================
    // MAPPING CONTROLS
    // =====================================================

    private NetworkTableEntry mappingEnabledEntry;
    private NetworkTableEntry saveMapEntry;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public LidarLocalization()
    {
        localizationTable =
                NetworkTableInstance
                        .getDefault()
                        .getTable(
                                "LidarLocalization"
                        );


        // =================================================
        // ROBOT POSE
        // =================================================

        robotXEntry =
                localizationTable
                        .getEntry(
                                "RobotX"
                        );


        robotYEntry =
                localizationTable
                        .getEntry(
                                "RobotY"
                        );


        robotHeadingEntry =
                localizationTable
                        .getEntry(
                                "RobotHeading"
                        );


        // =================================================
        // NEAREST OBSTACLE
        // =================================================

        obstacleXEntry =
                localizationTable
                        .getEntry(
                                "ObstacleX"
                        );


        obstacleYEntry =
                localizationTable
                        .getEntry(
                                "ObstacleY"
                        );


        obstacleDistanceEntry =
                localizationTable
                        .getEntry(
                                "ObstacleDistance"
                        );


        obstacleAngleEntry =
                localizationTable
                        .getEntry(
                                "ObstacleAngle"
                        );


        // =================================================
        // STATUS
        // =================================================

        localizationValidEntry =
                localizationTable
                        .getEntry(
                                "LocalizationValid"
                        );


        // =================================================
        // MAPPING CONTROL
        // =================================================

        mappingEnabledEntry =
                localizationTable
                        .getEntry(
                                "MappingEnabled"
                        );


        saveMapEntry =
                localizationTable
                        .getEntry(
                                "SaveMap"
                        );


        // Default values
        mappingEnabledEntry.setBoolean(
                false
        );


        saveMapEntry.setBoolean(
                false
        );
    }


    // =====================================================
    // ROBOT POSITION
    // =====================================================

    public double getRobotX()
    {
        return robotXEntry.getDouble(
                0.0
        );
    }


    public double getRobotY()
    {
        return robotYEntry.getDouble(
                0.0
        );
    }


    public double getRobotHeading()
    {
        return robotHeadingEntry.getDouble(
                0.0
        );
    }


    // =====================================================
    // OBSTACLE POSITION
    // =====================================================

    public double getObstacleX()
    {
        return obstacleXEntry.getDouble(
                0.0
        );
    }


    public double getObstacleY()
    {
        return obstacleYEntry.getDouble(
                0.0
        );
    }


    public double getObstacleDistance()
    {
        return obstacleDistanceEntry.getDouble(
                9999.0
        );
    }


    public double getObstacleAngle()
    {
        return obstacleAngleEntry.getDouble(
                0.0
        );
    }


    // =====================================================
    // LOCALIZATION STATUS
    // =====================================================

    public boolean isLocalizationValid()
    {
        return localizationValidEntry
                .getBoolean(
                        false
                );
    }


    // =====================================================
    // MAPPING CONTROL
    // =====================================================

    public void startMapping()
    {
        mappingEnabledEntry
                .setBoolean(
                        true
                );
    }


    public void stopMapping()
    {
        mappingEnabledEntry
                .setBoolean(
                        false
                );
    }


    public boolean isMappingEnabled()
    {
        return mappingEnabledEntry
                .getBoolean(
                        false
                );
    }


    // =====================================================
    // SAVE MAP REQUEST
    // =====================================================

    public void requestSaveMap()
    {
        saveMapEntry
                .setBoolean(
                        true
                );
    }


    public void clearSaveRequest()
    {
        saveMapEntry
                .setBoolean(
                        false
                );
    }


    // =====================================================
    // DASHBOARD
    // =====================================================

    @Override
    public void periodic()
    {
        // Robot pose
        SmartDashboard.putNumber(
                "Lidar Robot X",
                getRobotX()
        );


        SmartDashboard.putNumber(
                "Lidar Robot Y",
                getRobotY()
        );


        SmartDashboard.putNumber(
                "Lidar Heading",
                getRobotHeading()
        );


        // Nearest obstacle
        SmartDashboard.putNumber(
                "Obstacle X",
                getObstacleX()
        );


        SmartDashboard.putNumber(
                "Obstacle Y",
                getObstacleY()
        );


        SmartDashboard.putNumber(
                "Obstacle Distance",
                getObstacleDistance()
        );


        SmartDashboard.putNumber(
                "Obstacle Angle",
                getObstacleAngle()
        );


        // Status
        SmartDashboard.putBoolean(
                "Lidar Localization Valid",
                isLocalizationValid()
        );


        SmartDashboard.putBoolean(
                "Mapping Enabled",
                isMappingEnabled()
        );
    }
}