package frc.robot.subsystems;

import com.studica.frc.Lidar;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LidarTest extends SubsystemBase {

    private Lidar lidar;
    private Lidar.ScanData scanData;
    public boolean scanning = true;

    public LidarTest() {
        lidar = new Lidar(Lidar.Port.kUSB2);

        lidar.clusterConfig(50.0f, 5);
        lidar.enableFilter(Lidar.Filter.kCLUSTER, false);
    }

    public void startScan() {
        lidar.start();
        scanning = true;
    }

    public void stopScan() {
        lidar.stop();
        scanning = false;
    }

    @Override
    public void periodic() {
        if (!scanning) {
            return;
        }

        scanData = lidar.getData();

        if (scanData != null && scanData.distance != null && scanData.angle != null) {
            int length = Math.min(scanData.distance.length, scanData.angle.length);

            if (length > 0) {
                int mid = length / 2;
                int len2 = length - mid;

                // Part 1: Convert float values to double array
                double[] angles1 = new double[mid];
                double[] distances1 = new double[mid];
                for (int i = 0; i < mid; i++) {
                    angles1[i] = scanData.angle[i];
                    distances1[i] = scanData.distance[i];
                }

                // Part 2: Convert float values to double array
                double[] angles2 = new double[len2];
                double[] distances2 = new double[len2];
                for (int i = 0; i < len2; i++) {
                    angles2[i] = scanData.angle[mid + i];
                    distances2[i] = scanData.distance[mid + i];
                }

                NetworkTable table = NetworkTableInstance.getDefault().getTable("Lidar");

                table.getEntry("ScanAngles_Part1").setDoubleArray(angles1);
                table.getEntry("ScanDistances_Part1").setDoubleArray(distances1);
                table.getEntry("ScanAngles_Part2").setDoubleArray(angles2);
                table.getEntry("ScanDistances_Part2").setDoubleArray(distances2);

                if (length > 60) {
                    SmartDashboard.putNumber("Angle", scanData.angle[60]);
                    SmartDashboard.putNumber("Distance", scanData.distance[60]);
                }
            }
        }
    }
}