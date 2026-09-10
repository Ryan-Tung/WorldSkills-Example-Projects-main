package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import com.studica.frc.Lidar;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class DriveTrain extends SubsystemBase
{

    // ============================================================
    // LIDAR
    // ============================================================

    private Lidar lidar;
    private Lidar.ScanData scanData;

    public boolean scanning = true;


    // ============================================================
    // MOTORS
    // ============================================================

    private TitanQuad leftMotor;
    private TitanQuad rightMotor;
    private TitanQuad backMotor;


    // ============================================================
    // ENCODERS
    // ============================================================

    private TitanQuadEncoder leftEncoder;
    private TitanQuadEncoder rightEncoder;
    private TitanQuadEncoder backEncoder;


    // ============================================================
    // NAVX
    // ============================================================

    private AHRS navx;


    // ============================================================
    // NETWORKTABLES
    // ============================================================

    private final NetworkTable driveTable =
        NetworkTableInstance.getDefault().getTable("Drive");

    private final NetworkTable lidarTable =
        NetworkTableInstance.getDefault().getTable("Lidar");


    // ============================================================
    // SHUFFLEBOARD
    // ============================================================

    private ShuffleboardTab tab =
        Shuffleboard.getTab("Training Robot");

    private NetworkTableEntry leftEncoderValue =
        tab.add("Left Encoder", 0)
           .getEntry();

    private NetworkTableEntry rightEncoderValue =
        tab.add("Right Encoder", 0)
           .getEntry();

    private NetworkTableEntry backEncoderValue =
        tab.add("Back Encoder", 0)
           .getEntry();

    private NetworkTableEntry gyroValue =
        tab.add("NavX Heading", 0)
           .getEntry();

    private NetworkTableEntry poseXValue =
        tab.add("Pose X", 0)
           .getEntry();

    private NetworkTableEntry poseYValue =
        tab.add("Pose Y", 0)
           .getEntry();

    private NetworkTableEntry poseHeadingValue =
        tab.add("Pose Heading", 0)
           .getEntry();


    // ============================================================
    // ROBOT POSE
    // ============================================================

    private double poseX = 0.0;
    private double poseY = 0.0;


    // Previous encoder positions
    private double previousLeftDistance = 0.0;
    private double previousRightDistance = 0.0;
    private double previousBackDistance = 0.0;

    // Previous heading
    private double previousHeading = 0.0;


    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    public DriveTrain()
    {

        // --------------------------------------------------------
        // LIDAR
        // --------------------------------------------------------

        lidar = new Lidar(Lidar.Port.kUSB2);

        lidar.clusterConfig(50.0f, 5);
        lidar.enableFilter(
            Lidar.Filter.kCLUSTER,
            false
        );


        // --------------------------------------------------------
        // MOTORS
        // --------------------------------------------------------

        leftMotor =
            new TitanQuad(
                Constants.TITAN_ID,
                Constants.M3
            );

        rightMotor =
            new TitanQuad(
                Constants.TITAN_ID,
                Constants.M0
            );

        backMotor =
            new TitanQuad(
                Constants.TITAN_ID,
                Constants.M1
            );


        // --------------------------------------------------------
        // ENCODERS
        // --------------------------------------------------------

        leftEncoder =
            new TitanQuadEncoder(
                leftMotor,
                Constants.M3,
                Constants.WHEEL_DIST_PER_TICK
            );

        rightEncoder =
            new TitanQuadEncoder(
                rightMotor,
                Constants.M0,
                Constants.WHEEL_DIST_PER_TICK
            );

        backEncoder =
            new TitanQuadEncoder(
                backMotor,
                Constants.M1,
                Constants.WHEEL_DIST_PER_TICK
            );


        // --------------------------------------------------------
        // NAVX
        // --------------------------------------------------------

        navx =
            new AHRS(
                SPI.Port.kMXP
            );


        // --------------------------------------------------------
        // Initialise odometry
        // --------------------------------------------------------

        resetOdometry();
    }


    // ============================================================
    // LIDAR CONTROL & READINGS
    // ============================================================

    public void startScan()
    {
        lidar.start();
        scanning = true;
    }


    public void stopScan()
    {
        lidar.stop();
        scanning = false;
    }

    /**
     * Gets the LiDAR distance measurement at approximately 0 degrees (forward).
     * @return distance in centimeters (CM), or 999.0 if unavailable.
     */
    public double getLidarAtZeroDegrees()
    {
        if (scanData == null || scanData.distance == null || scanData.angle == null)
        {
            return 999.0;
        }

        int length = Math.min(scanData.distance.length, scanData.angle.length);
        if (length == 0)
        {
            return 999.0;
        }

        double minDiff = Double.MAX_VALUE;
        double distanceAt0 = 999.0;

        for (int i = 0; i < length; i++)
        {
            double angle = scanData.angle[i];
            // Handles angle boundary near 0° / 360°
            double diff = Math.min(Math.abs(angle - 0.0), Math.abs(angle - 360.0));

            if (diff < minDiff)
            {
                minDiff = diff;
                // Studica Lidar provides mm; convert to cm to match existing threshold logic
                distanceAt0 = scanData.distance[i] / 10.0;
            }
        }

        return distanceAt0;
    }


    // ============================================================
    // MOTOR CONTROL
    // ============================================================

    public void setLeftMotorSpeed(double speed)
    {
        leftMotor.set(speed);
    }


    public void setRightMotorSpeed(double speed)
    {
        rightMotor.set(speed);
    }


    public void setBackMotorSpeed(double speed)
    {
        backMotor.set(speed);
    }


    public void setDriveMotorSpeeds(
        double leftSpeed,
        double rightSpeed,
        double backSpeed
    )
    {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
        backMotor.set(backSpeed);
    }


    // ============================================================
    // HOLONOMIC DRIVE
    // ============================================================

    public void holonomicDrive(
        double x,
        double y,
        double z
    )
    {

        double rightSpeed =
            ((x / 3)
            - (y / Math.sqrt(3))
            + z)
            * Math.sqrt(3);

        double leftSpeed =
            ((x / 3)
            + (y / Math.sqrt(3))
            + z)
            * Math.sqrt(3);

        double backSpeed =
            (-2 * x / 3)
            + z;


        double max =
            Math.abs(rightSpeed);

        if (Math.abs(leftSpeed) > max)
            max = Math.abs(leftSpeed);

        if (Math.abs(backSpeed) > max)
            max = Math.abs(backSpeed);


        if (max > 1)
        {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }


        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
        backMotor.set(backSpeed);
    }


    // ============================================================
    // ENCODER DISTANCES
    // ============================================================

    public double getLeftEncoderDistance()
    {
        return leftEncoder.getEncoderDistance() * -1;
    }


    public double getRightEncoderDistance()
    {
        return rightEncoder.getEncoderDistance() * -1;
    }


    public double getBackEncoderDistance()
    {
        return backEncoder.getEncoderDistance();
    }


    public double getAverageForwardEncoderDistance()
    {
        return (
            getLeftEncoderDistance()
            + getRightEncoderDistance()
        ) / 2.0;
    }


    // ============================================================
    // NAVX
    // ============================================================

    public double getYaw()
    {
        return navx.getYaw();
    }


    private double normalizeAngle(
        double angle
    )
    {

        while (angle > 180.0)
            angle -= 360.0;

        while (angle < -180.0)
            angle += 360.0;

        return angle;
    }


    // ============================================================
    // UPDATE ODOMETRY
    // ============================================================

    private void updateOdometry()
    {

        double currentLeft =
            getLeftEncoderDistance();

        double currentRight =
            getRightEncoderDistance();

        double currentBack =
            getBackEncoderDistance();


        double deltaLeft =
            currentLeft
            - previousLeftDistance;

        double deltaRight =
            currentRight
            - previousRightDistance;

        double deltaBack =
            currentBack
            - previousBackDistance;


        double currentHeading =
            getYaw();


        double deltaHeading =
            normalizeAngle(
                currentHeading
                - previousHeading
            );


        double robotDeltaX =
            -(
                (deltaLeft + deltaRight)
                / (2.0 * Math.sqrt(3.0))
            )
            - deltaBack;


        double robotDeltaY =
            (
                deltaRight
                - deltaLeft
            ) / 2.0;


        double averageHeading =
            previousHeading
            + (deltaHeading / 2.0);


        double headingRad =
            Math.toRadians(
                averageHeading
            );


        double fieldDeltaX =
            robotDeltaX
            * Math.cos(headingRad)
            +
            robotDeltaY
            * Math.sin(headingRad);


        double fieldDeltaY =
            -robotDeltaX
            * Math.sin(headingRad)
            +
            robotDeltaY
            * Math.cos(headingRad);


        poseX += fieldDeltaX / 1000.0;
        poseY += fieldDeltaY / 1000.0;


        previousLeftDistance =
            currentLeft;

        previousRightDistance =
            currentRight;

        previousBackDistance =
            currentBack;

        previousHeading =
            currentHeading;
    }


    // ============================================================
    // GET POSE
    // ============================================================

    public double getPoseX()
    {
        return poseX;
    }


    public double getPoseY()
    {
        return poseY;
    }


    public double getPoseHeading()
    {
        return getYaw();
    }


    // ============================================================
    // RESET ENCODERS & YAW
    // ============================================================

    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
        backEncoder.reset();
    }


    public void resetYaw()
    {
        navx.zeroYaw();
    }


    public void resetOdometry()
    {

        resetEncoders();

        resetYaw();

        poseX = 0.0;
        poseY = 0.0;

        previousLeftDistance =
            getLeftEncoderDistance();

        previousRightDistance =
            getRightEncoderDistance();

        previousBackDistance =
            getBackEncoderDistance();

        previousHeading =
            getYaw();
    }


    // ============================================================
    // PERIODIC
    // ============================================================

    @Override
    public void periodic()
    {

        updateOdometry();


        // SHUFFLEBOARD
        leftEncoderValue.setDouble(getLeftEncoderDistance());
        rightEncoderValue.setDouble(getRightEncoderDistance());
        backEncoderValue.setDouble(getBackEncoderDistance());
        gyroValue.setDouble(getYaw());
        poseXValue.setDouble(poseX);
        poseYValue.setDouble(poseY);
        poseHeadingValue.setDouble(getYaw());


        // SMARTDASHBOARD
        SmartDashboard.putNumber("Pose X", poseX);
        SmartDashboard.putNumber("Pose Y", poseY);
        SmartDashboard.putNumber("Pose Heading", getYaw());


        // PUBLISH POSE TO NETWORKTABLES
        driveTable.getEntry("PoseX").setDouble(poseX);
        driveTable.getEntry("PoseY").setDouble(poseY);
        driveTable.getEntry("PoseHeading").setDouble(getYaw());


        // LIDAR
        if (!scanning)
        {
            return;
        }

        scanData = lidar.getData();

        if (
            scanData != null
            && scanData.distance != null
            && scanData.angle != null
        )
        {
            int length = Math.min(
                scanData.distance.length,
                scanData.angle.length
            );

            if (length > 0)
            {
                int mid = length / 2;
                int len2 = length - mid;

                double[] angles1 = new double[mid];
                double[] distances1 = new double[mid];

                for (int i = 0; i < mid; i++)
                {
                    angles1[i] = scanData.angle[i];
                    distances1[i] = scanData.distance[i];
                }

                double[] angles2 = new double[len2];
                double[] distances2 = new double[len2];

                for (int i = 0; i < len2; i++)
                {
                    angles2[i] = scanData.angle[mid + i];
                    distances2[i] = scanData.distance[mid + i];
                }

                lidarTable.getEntry("ScanAngles_Part1").setDoubleArray(angles1);
                lidarTable.getEntry("ScanDistances_Part1").setDoubleArray(distances1);
                lidarTable.getEntry("ScanAngles_Part2").setDoubleArray(angles2);
                lidarTable.getEntry("ScanDistances_Part2").setDoubleArray(distances2);

                if (length > 60)
                {
                    SmartDashboard.putNumber("Angle", scanData.angle[60]);
                    SmartDashboard.putNumber("Distance", scanData.distance[60]);
                }
            }
        }
    }
}