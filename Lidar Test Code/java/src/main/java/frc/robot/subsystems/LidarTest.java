package frc.robot.subsystems;

import com.studica.frc.Lidar;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarTest extends SubsystemBase
{
    private Lidar lidar;
    private Lidar.ScanData scanData;

    public boolean scanning = true;

    private NetworkTable lidarTable;

    public LidarTest()
    {
        lidar = new Lidar(Lidar.Port.kUSB1);

        lidarTable = NetworkTableInstance
                .getDefault()
                .getTable("Lidar");

        System.out.println("Lidar initialized");
    }

    public void startScan()
    {
        lidar.start();
        scanning = true;

        System.out.println("Lidar started");
    }

    public void stopScan()
    {
        lidar.stop();
        scanning = false;

        System.out.println("Lidar stopped");
    }

    @Override
    public void periodic()
    {
        if (!scanning)
        {
            return;
        }

        scanData = lidar.getData();

        if (scanData == null ||
            scanData.angle == null ||
            scanData.distance == null)
        {
            return;
        }

        int count = Math.min(
                scanData.angle.length,
                scanData.distance.length
        );

        if (count <= 0)
        {
            return;
        }

        double[] tempA = new double[count * 2];
        double[] tempB = new double[count * 2];

        int countA = 0;
        int countB = 0;

        int q1 = 0;
        int q2 = 0;
        int q3 = 0;
        int q4 = 0;

        double minAngle = 999.0;
        double maxAngle = -999.0;

        for (int i = 0; i < count; i++)
        {
            double angle = scanData.angle[i];
            double distance = scanData.distance[i];

            angle = angle % 360.0;

            if (angle < 0)
            {
                angle += 360.0;
            }

            if (distance >= 120.0 &&
                distance <= 5000.0)
            {
                if (angle < minAngle)
                {
                    minAngle = angle;
                }

                if (angle > maxAngle)
                {
                    maxAngle = angle;
                }

                if (angle < 180.0)
                {
                    tempA[countA * 2] = angle;
                    tempA[countA * 2 + 1] = distance;
                    countA++;

                    if (angle < 90.0)
                    {
                        q1++;
                    }
                    else
                    {
                        q2++;
                    }
                }
                else
                {
                    tempB[countB * 2] = angle;
                    tempB[countB * 2 + 1] = distance;
                    countB++;

                    if (angle < 270.0)
                    {
                        q3++;
                    }
                    else
                    {
                        q4++;
                    }
                }
            }
        }

        double[] outputA = new double[countA * 2];
        double[] outputB = new double[countB * 2];

        for (int i = 0; i < countA * 2; i++)
        {
            outputA[i] = tempA[i];
        }

        for (int i = 0; i < countB * 2; i++)
        {
            outputB[i] = tempB[i];
        }

        lidarTable
                .getEntry("ScanA")
                .setDoubleArray(outputA);

        lidarTable
                .getEntry("ScanB")
                .setDoubleArray(outputB);

        SmartDashboard.putNumber("Raw Count", count);
        SmartDashboard.putNumber("ScanA Points", countA);
        SmartDashboard.putNumber("ScanB Points", countB);

        SmartDashboard.putNumber("Min Raw Angle", minAngle);
        SmartDashboard.putNumber("Max Raw Angle", maxAngle);

        SmartDashboard.putNumber("0-90 Points", q1);
        SmartDashboard.putNumber("90-180 Points", q2);
        SmartDashboard.putNumber("180-270 Points", q3);
        SmartDashboard.putNumber("270-360 Points", q4);
    }
}