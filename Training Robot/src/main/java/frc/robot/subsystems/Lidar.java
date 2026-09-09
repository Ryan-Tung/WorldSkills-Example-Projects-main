package frc.robot.subsystems;

import frc.robot.NetPrinter_v2;

public class Lidar {
    // Notice the fully qualified name "com.studica.frc.Lidar" is used here
    // to separate it from your subsystem class also named Lidar.
    private com.studica.frc.Lidar lidar;
    private com.studica.frc.Lidar.ScanData scanData;
    private boolean isInitialized = false;

    public Lidar() {
        try {
            lidar = new com.studica.frc.Lidar(com.studica.frc.Lidar.Port.kUSB1);
            lidar.start();
            isInitialized = true;
        } catch (Exception e) {
            isInitialized = false;
            NetPrinter_v2.printf("LidarLog", "LIDAR INIT ERROR: " + e.getMessage());
        }
    }

    public double getDistanceAtAngle(int angle) {
        if (!isInitialized || lidar == null) return 9999.0;
        
        scanData = lidar.getData();
        int normalizedAngle = (angle % 360 + 360) % 360;

        if (scanData != null && scanData.distance != null && normalizedAngle < scanData.distance.length) {
            return scanData.distance[normalizedAngle];
        }
        return 9999.0;
    }
}