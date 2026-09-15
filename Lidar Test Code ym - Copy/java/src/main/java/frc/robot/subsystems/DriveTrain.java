package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase
{
    // =====================================================
    // IR SENSOR
    // =====================================================

    private final AnalogInput sharp = new AnalogInput(0);


    // =====================================================
    // MOTORS
    // =====================================================

    private TitanQuad leftMotor;
    private TitanQuad rightMotor;
    private TitanQuad backMotor;


    // =====================================================
    // ENCODERS
    // =====================================================

    private TitanQuadEncoder leftEncoder;
    private TitanQuadEncoder rightEncoder;
    private TitanQuadEncoder backEncoder;


    // =====================================================
    // NAVX
    // =====================================================

    private AHRS navx;

    private double yawOffset = 0.0;
    private boolean yawReferenceSet = false;


    // =====================================================
    // ROBOT POSITION / LOCALISATION
    // =====================================================

    /*
     * Robot-local coordinate:
     *
     *                +Y
     *               FRONT
     *                 ^
     *                 |
     *       -X <---- ROBOT ----> +X
     *
     * X = robot sideways
     * Y = robot forward/backward
     *
     * robotX / robotY below are then converted
     * into world/map coordinates using yaw.
     */

    private double robotX = 0.0;
    private double robotY = 0.0;


    private double previousLeftDistance = 0.0;
    private double previousRightDistance = 0.0;
    private double previousBackDistance = 0.0;


    // =====================================================
    // NETWORKTABLES FOR PYTHON
    // =====================================================

    private final NetworkTable poseTable =
            NetworkTableInstance
                    .getDefault()
                    .getTable("RobotPose");


    // =====================================================
    // KEYBOARD COMMAND STATE FOR ODOMETRY FILTERING
    //
    // Used only to remove known encoder cross-coupling:
    //
    // - pure A/D crab: ignore tiny false forward/back drift
    // - pure Q/E spin: keep X/Y fixed while robot rotates
    //
    // Motor control itself is still handled by KeyboardDrive.
    // =====================================================

    private final NetworkTable keyboardTable =
            NetworkTableInstance
                    .getDefault()
                    .getTable("KeyboardDrive");


    private static final double KEYBOARD_TRANSLATION_THRESHOLD =
            0.05;


    private static final double KEYBOARD_ROTATION_THRESHOLD =
            0.05;


    // =====================================================
    // SHUFFLEBOARD
    // =====================================================

    private ShuffleboardTab tab =
            Shuffleboard.getTab("Training Robot");


    private NetworkTableEntry leftEncoderValue =
            tab.add(
                    "Left Encoder",
                    0
            ).getEntry();


    private NetworkTableEntry rightEncoderValue =
            tab.add(
                    "Right Encoder",
                    0
            ).getEntry();


    private NetworkTableEntry backEncoderValue =
            tab.add(
                    "Back Encoder",
                    0
            ).getEntry();


    private NetworkTableEntry average =
            tab.add(
                    "Average Forward Encoder",
                    0
            ).getEntry();


    private NetworkTableEntry strafeAverage =
            tab.add(
                    "Average Strafe Encoder",
                    0
            ).getEntry();


    private NetworkTableEntry IRsensor =
            tab.add(
                    "IRsensor",
                    0
            ).getEntry();


    private NetworkTableEntry gyroValue =
            tab.add(
                    "NavX Yaw",
                    0
            ).getEntry();


    private NetworkTableEntry rawYawValue =
            tab.add(
                    "Raw NavX Yaw",
                    0
            ).getEntry();


    private NetworkTableEntry correctedYawValue =
            tab.add(
                    "Corrected Yaw",
                    0
            ).getEntry();


    private NetworkTableEntry yawOffsetValue =
            tab.add(
                    "Yaw Offset",
                    0
            ).getEntry();


    private NetworkTableEntry yawReferenceValue =
            tab.add(
                    "Yaw Reference Set",
                    false
            ).getEntry();


    private NetworkTableEntry robotXValue =
            tab.add(
                    "Robot X (m)",
                    0
            ).getEntry();


    private NetworkTableEntry robotYValue =
            tab.add(
                    "Robot Y (m)",
                    0
            ).getEntry();


    private NetworkTableEntry pureCrabTrackingValue =
            tab.add(
                    "Pure Crab Tracking Filter",
                    false
            ).getEntry();


    private NetworkTableEntry pureSpinTrackingValue =
            tab.add(
                    "Pure Spin XY Hold",
                    false
            ).getEntry();


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public DriveTrain()
    {
        // Motors
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


        // Encoders
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


        // NavX
        navx =
                new AHRS(
                        SPI.Port.kMXP
                );


        previousLeftDistance =
                getLeftEncoderDistance();

        previousRightDistance =
                getRightEncoderDistance();

        previousBackDistance =
                getBackEncoderDistance();
    }


    // =====================================================
    // IR SENSOR
    // =====================================================

    public double getDistance()
    {
        return (
                Math.pow(
                        sharp.getAverageVoltage(),
                        -1.2045
                )
        )
        *
        27.726;
    }


    // =====================================================
    // MOTOR CONTROL
    // =====================================================

    public void setLeftMotorSpeed(
            double speed)
    {
        leftMotor.set(
                speed
        );
    }


    public void setRightMotorSpeed(
            double speed)
    {
        rightMotor.set(
                speed
        );
    }


    public void setBackMotorSpeed(
            double speed)
    {
        backMotor.set(
                speed
        );
    }


    public void setDriveMotorSpeeds(
            double leftSpeed,
            double rightSpeed,
            double backSpeed)
    {
        leftMotor.set(
                leftSpeed
        );

        rightMotor.set(
                rightSpeed
        );

        backMotor.set(
                backSpeed
        );
    }


    // =====================================================
    // HOLONOMIC DRIVE
    // =====================================================

    public void holonomicDrive(
            double x,
            double y,
            double z)
    {
        double rightSpeed =
                (
                    (x / 3.0)
                    -
                    (y / Math.sqrt(3.0))
                    +
                    z
                )
                *
                Math.sqrt(3.0);


        double leftSpeed =
                (
                    (x / 3.0)
                    +
                    (y / Math.sqrt(3.0))
                    +
                    z
                )
                *
                Math.sqrt(3.0);


        double backSpeed =
                (-2.0 * x / 3.0)
                +
                z;


        // Normalize motor speeds
        double max =
                Math.abs(
                        rightSpeed
                );


        if (
            Math.abs(leftSpeed)
            >
            max
        )
        {
            max =
                    Math.abs(
                            leftSpeed
                    );
        }


        if (
            Math.abs(backSpeed)
            >
            max
        )
        {
            max =
                    Math.abs(
                            backSpeed
                    );
        }


        if (max > 1.0)
        {
            rightSpeed /=
                    max;

            leftSpeed /=
                    max;

            backSpeed /=
                    max;
        }


        leftMotor.set(
                leftSpeed
        );

        rightMotor.set(
                rightSpeed
        );

        backMotor.set(
                backSpeed
        );
    }


    // =====================================================
    // ENCODERS
    // =====================================================

    public double getLeftEncoderDistance()
    {
        return leftEncoder
                .getEncoderDistance()
                *
                -1.0;
    }


    public double getRightEncoderDistance()
    {
        return rightEncoder
                .getEncoderDistance()
                *
                -1.0;
    }


    public double getBackEncoderDistance()
    {
        return backEncoder
                .getEncoderDistance();
    }


    // =====================================================
    // ROBOT-LOCAL Y DISTANCE
    // FORWARD / BACKWARD
    // =====================================================

    public double getAverageForwardEncoderDistance()
    {
        return (
                getLeftEncoderDistance()
                -
                getRightEncoderDistance()
        )
        /
        2.0;
    }


    // =====================================================
    // ROBOT-LOCAL X DISTANCE
    // STRAFE / CRAB
    // =====================================================

    public double getAverageStrafeEncoderDistance()
    {
        double left =
                getLeftEncoderDistance();


        double right =
                getRightEncoderDistance();


        double back =
                getBackEncoderDistance();


        /*
         * Same robot-local X relationship
         * already used in updateRobotPose().
         */

        return (
                (
                    left
                    +
                    right
                )
                /
                (
                    2.0
                    *
                    Math.sqrt(3.0)
                )
        )
        -
        back;
    }


    // =====================================================
    // RAW NAVX YAW
    // =====================================================

    public double getRawYaw()
    {
        return navx
                .getYaw();
    }


    // =====================================================
    // CORRECTED YAW
    // =====================================================

    public double getYaw()
    {
        double heading =
                navx.getYaw()
                -
                yawOffset;


        while (heading > 180.0)
        {
            heading -=
                    360.0;
        }


        while (heading < -180.0)
        {
            heading +=
                    360.0;
        }


        return heading;
    }


    // =====================================================
    // RESET YAW
    //
    // Makes the robot's CURRENT physical direction become
    // corrected heading 0 degrees.
    //
    // This is important for LIDAR_MOVING because
    // NavigateOneMeter holds heading 0 while driving
    // straight. Without refreshing this reference when the
    // mode starts, the robot can curve while trying to turn
    // back toward an old heading reference.
    // =====================================================

    public void resetYaw()
    {
        yawOffset =
                navx.getYaw();


        yawReferenceSet =
                true;
    }


    // =====================================================
    // ROBOT POSE
    // =====================================================

    public double getRobotX()
    {
        return robotX
                /
                1000.0;
    }


    public double getRobotY()
    {
        return robotY
                /
                1000.0;
    }


    public double getRobotHeading()
    {
        return getYaw();
    }


    // =====================================================
    // RESET ENCODERS
    // =====================================================

    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
        backEncoder.reset();


        previousLeftDistance =
                0.0;

        previousRightDistance =
                0.0;

        previousBackDistance =
                0.0;
    }


    // =====================================================
    // RESET FULL POSE
    // =====================================================

    public void resetPose()
    {
        robotX =
                0.0;

        robotY =
                0.0;


        resetEncoders();

        // Current physical direction becomes heading 0°.
        resetYaw();
    }


    // =====================================================
    // UPDATE ROBOT POSITION
    // =====================================================

    private void updateRobotPose()
    {
        double currentLeft =
                getLeftEncoderDistance();


        double currentRight =
                getRightEncoderDistance();


        double currentBack =
                getBackEncoderDistance();


        // Movement since previous loop
        double deltaLeft =
                currentLeft
                -
                previousLeftDistance;


        double deltaRight =
                currentRight
                -
                previousRightDistance;


        double deltaBack =
                currentBack
                -
                previousBackDistance;


        previousLeftDistance =
                currentLeft;


        previousRightDistance =
                currentRight;


        previousBackDistance =
                currentBack;


        // =================================================
        // ROBOT-LOCAL Y
        // =================================================

        double localY =
                (
                    deltaLeft
                    -
                    deltaRight
                )
                /
                2.0;


        // =================================================
        // ROBOT-LOCAL X
        // =================================================

        double localX =
                (
                    (
                        deltaLeft
                        +
                        deltaRight
                    )
                    /
                    (
                        2.0
                        *
                        Math.sqrt(3.0)
                    )
                )
                -
                deltaBack;


        // =================================================
        // KEYBOARD MOTION TYPE
        //
        // NetworkTables contains the USER command before
        // KeyboardDrive adds its small navX crab correction.
        // That means pure A/D still appears here as Z = 0,
        // which is exactly what we want for this filter.
        // =================================================

        boolean keyboardEnabled =
                keyboardTable
                        .getEntry("Enabled")
                        .getBoolean(false);


        double keyboardX =
                keyboardTable
                        .getEntry("X")
                        .getDouble(0.0);


        double keyboardY =
                keyboardTable
                        .getEntry("Y")
                        .getDouble(0.0);


        double keyboardZ =
                keyboardTable
                        .getEntry("Z")
                        .getDouble(0.0);


        boolean pureKeyboardCrab =
                keyboardEnabled
                &&
                Math.abs(keyboardX)
                >=
                KEYBOARD_TRANSLATION_THRESHOLD
                &&
                Math.abs(keyboardY)
                <
                KEYBOARD_TRANSLATION_THRESHOLD
                &&
                Math.abs(keyboardZ)
                <
                KEYBOARD_ROTATION_THRESHOLD;


        boolean pureKeyboardSpin =
                keyboardEnabled
                &&
                Math.abs(keyboardZ)
                >=
                KEYBOARD_ROTATION_THRESHOLD
                &&
                Math.abs(keyboardX)
                <
                KEYBOARD_TRANSLATION_THRESHOLD
                &&
                Math.abs(keyboardY)
                <
                KEYBOARD_TRANSLATION_THRESHOLD;


        // =================================================
        // PURE A / D CRAB TRACKING FILTER
        //
        // Ideal pure crab has localY = 0.
        //
        // Small left/right wheel differences can otherwise
        // create a false localY term and bend the plotted
        // crab route even when the robot is moving sideways.
        // =================================================

        if (pureKeyboardCrab)
        {
            localY =
                    0.0;
        }


        // =================================================
        // PURE Q / E SPIN X/Y HOLD
        //
        // During an in-place spin the robot centre should
        // not translate. Encoder mismatch used to create a
        // false circular route. Consume the encoder deltas
        // normally, but do not add them to X/Y.
        // =================================================

        if (pureKeyboardSpin)
        {
            localX =
                    0.0;


            localY =
                    0.0;
        }


        pureCrabTrackingValue.setBoolean(
                pureKeyboardCrab
        );


        pureSpinTrackingValue.setBoolean(
                pureKeyboardSpin
        );


        // =================================================
        // CONVERT LOCAL MOVEMENT TO MAP/WORLD MOVEMENT
        // =================================================

        double headingDeg =
                getYaw();


        double headingRad =
                Math.toRadians(
                        headingDeg
                );


        double worldDeltaX =
                localX
                *
                Math.cos(
                        headingRad
                )
                +
                localY
                *
                Math.sin(
                        headingRad
                );


        double worldDeltaY =
                -localX
                *
                Math.sin(
                        headingRad
                )
                +
                localY
                *
                Math.cos(
                        headingRad
                );


        robotX +=
                worldDeltaX;


        robotY +=
                worldDeltaY;
    }


    // =====================================================
    // PERIODIC
    // =====================================================

    @Override
    public void periodic()
    {
        // Set starting direction to corrected 0 degrees
        if (
            !yawReferenceSet
            &&
            !navx.isCalibrating()
        )
        {
            yawOffset =
                    navx.getYaw();


            yawReferenceSet =
                    true;
        }


        updateRobotPose();


        double rawYaw =
                getRawYaw();


        double correctedYaw =
                getYaw();


        // =================================================
        // SHUFFLEBOARD
        // =================================================

        leftEncoderValue.setDouble(
                getLeftEncoderDistance()
        );


        rightEncoderValue.setDouble(
                getRightEncoderDistance()
        );


        backEncoderValue.setDouble(
                getBackEncoderDistance()
        );


        average.setDouble(
                getAverageForwardEncoderDistance()
        );


        strafeAverage.setDouble(
                getAverageStrafeEncoderDistance()
        );


        IRsensor.setDouble(
                getDistance()
        );


        gyroValue.setDouble(
                correctedYaw
        );


        rawYawValue.setDouble(
                rawYaw
        );


        correctedYawValue.setDouble(
                correctedYaw
        );


        yawOffsetValue.setDouble(
                yawOffset
        );


        yawReferenceValue.setBoolean(
                yawReferenceSet
        );


        robotXValue.setDouble(
                getRobotX()
        );


        robotYValue.setDouble(
                getRobotY()
        );


        // =================================================
        // SEND WORLD/MAP POSE TO PYTHON
        // =================================================

        poseTable
                .getEntry("X")
                .setDouble(
                        getRobotX()
                );


        poseTable
                .getEntry("Y")
                .setDouble(
                        getRobotY()
                );


        poseTable
                .getEntry("Heading")
                .setDouble(
                        correctedYaw
                );
    }
}