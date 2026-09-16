package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class FusionLocalization extends SubsystemBase
{
    // =====================================================
    // EXISTING DRIVETRAIN
    //
    // Encoder odometry:
    //     X / Y
    //
    // navX:
    //     Heading
    // =====================================================

    private final DriveTrain driveTrain;


    // =====================================================
    // NETWORKTABLES
    //
    // Python LiDAR localization already publishes:
    //
    // LidarLocalization/RobotX
    // LidarLocalization/RobotY
    // LidarLocalization/RobotHeading
    // LidarLocalization/LocalizationValid
    //
    // This subsystem publishes the fused result to:
    //
    // FusionLocalization/X
    // FusionLocalization/Y
    // FusionLocalization/Heading
    // =====================================================

    private final NetworkTable lidarTable;
    private final NetworkTable fusionTable;
    private final NetworkTable keyboardTable;


    // =====================================================
    // FUSION SETTINGS
    //
    // SENSOR-DRIVEN FUSION TEST
    //
    // IMPORTANT:
    //
    // The estimator does NOT decide movement from keyboard keys.
    //
    // Encoder odometry:
    //     always predicts X/Y from actual wheel movement.
    //
    // navX:
    //     always provides heading.
    //
    // LiDAR ICP:
    //     corrects encoder X/Y whenever a fresh valid LiDAR
    //     observation is available.
    //
    // Therefore:
    //
    // - Q/E is NOT hard locked to zero X/Y.
    // - idle is NOT hard locked to zero X/Y.
    // - collision/wheel slip is allowed to appear in encoder
    //   prediction and then be corrected by LiDAR.
    //
    // Keyboard values are retained ONLY for debugging.
    // They are not used to decide whether fusion may move.
    // =====================================================

    private static final double COMMAND_THRESHOLD = 0.05;

    // Raw encoder distances in this project are millimetres.
    private static final double ENCODER_TO_METERS = 1.0 / 1000.0;

    // Ignore extremely tiny encoder quantisation/noise only.
    private static final double ENCODER_DELTA_DEADBAND_M = 0.0005;

    // Reject only a physically impossible one-cycle encoder jump.
    private static final double MAX_ENCODER_STEP_M = 0.15;


    // -----------------------------------------------------
    // LIDAR COMPLEMENTARY CORRECTION
    //
    // These are estimator tuning parameters, NOT keyboard
    // rules.
    //
    // LiDAR is allowed to correct meaningful disagreement,
    // including wheel slip / collision drift.
    //
    // A very large innovation is still rejected as a likely
    // bad ICP association / frame error.
    // -----------------------------------------------------

    private static final double LIDAR_CORRECTION_GAIN = 0.15;

    private static final double LIDAR_CORRECTION_DEADBAND_M = 0.01;

    private static final double MAX_LIDAR_INNOVATION_M = 0.75;

    private static final double MAX_LIDAR_CORRECTION_STEP_M = 0.05;


    // -----------------------------------------------------
    // LIDAR QUALITY GATE
    //
    // Python already publishes ICPError and InlierRatio.
    // Use actual localization quality rather than keyboard
    // state to decide whether a LiDAR correction is trusted.
    // -----------------------------------------------------

    private static final double MAX_LIDAR_ICP_ERROR_M = 0.16;

    private static final double MIN_LIDAR_INLIER_RATIO = 0.20;


    // =====================================================
    // FUSION STATE
    // =====================================================

    private boolean fusionActive = false;

    private double fusedX = 0.0;
    private double fusedY = 0.0;
    private double fusedHeading = 0.0;


    // =====================================================
    // ENCODER / HEADING REFERENCE
    //
    // IMPORTANT:
    //
    // Fusion now reads the THREE wheel encoders directly.
    //
    // It does NOT use DriveTrain.getRobotX()/getRobotY() as
    // the movement source.
    //
    // This avoids:
    // - old RobotPose filters affecting fusion
    // - world-frame re-zero problems
    // - accumulated RobotPose state being reinterpreted after
    //   a fusion reset
    // =====================================================

    private double encoderStartHeading = 0.0;

    private double previousLeftDistance = 0.0;
    private double previousRightDistance = 0.0;
    private double previousBackDistance = 0.0;


    // =====================================================
    // LIDAR REFERENCE
    //
    // LiDAR and encoder localization may have different:
    //
    // - X/Y origins
    // - heading zero references
    //
    // Therefore the first VALID LiDAR pose becomes a reference
    // and is aligned with the fused frame automatically.
    // =====================================================

    private boolean lidarReferenceReady = false;

    private double lidarReferenceX = 0.0;
    private double lidarReferenceY = 0.0;
    private double lidarReferenceHeading = 0.0;

    // Fused pose at the moment the LiDAR reference is captured.
    private double fusedXAtLidarReference = 0.0;
    private double fusedYAtLidarReference = 0.0;
    private double fusedHeadingAtLidarReference = 0.0;

    // Rotation needed to convert LiDAR world coordinates into
    // the fusion coordinate frame.
    private double lidarToFusionRotationDeg = 0.0;

    // Python increments SampleId for EVERY localization update,
    // even when LiDAR X/Y stays unchanged. This is essential
    // after a collision: encoders may slip while LiDAR correctly
    // says the robot stayed in the same place.
    private long lastProcessedLidarSampleId = -1;

    // Python increments ResetId whenever LiDAR ICP is manually
    // reset. A changed ResetId forces us to capture a fresh
    // LiDAR-to-fusion frame alignment.
    private long lastLidarResetId = -1;


    // =====================================================
    // EXTERNAL FULL-RESET HANDSHAKE
    //
    // Python increments FusionLocalization/ResetRequestId
    // when C is pressed.
    //
    // This makes the reset affect the REAL fusion coordinate
    // frame instead of only changing the Python drawing frame.
    // =====================================================

    private long lastFusionResetRequestId = -1;


    // =====================================================
    // DEBUG / STATUS
    // =====================================================

    private boolean lidarValid = false;

    private double lastLidarAlignedX = 0.0;
    private double lastLidarAlignedY = 0.0;

    private double lastCorrectionErrorM = 0.0;
    private double lastCorrectionAppliedM = 0.0;

    private long acceptedLidarCorrections = 0;
    private long rejectedLidarCorrections = 0;
    private long rejectedEncoderSteps = 0;

    private long lastSeenLidarSampleId = -1;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public FusionLocalization(
            DriveTrain driveTrain)
    {
        this.driveTrain = driveTrain;

        lidarTable =
                NetworkTableInstance
                        .getDefault()
                        .getTable(
                                "LidarLocalization"
                        );

        fusionTable =
                NetworkTableInstance
                        .getDefault()
                        .getTable(
                                "FusionLocalization"
                        );

        keyboardTable =
                NetworkTableInstance
                        .getDefault()
                        .getTable(
                                "KeyboardDrive"
                        );
    }


    // =====================================================
    // START / RESET FUSION
    //
    // IMPORTANT:
    //
    // This does NOT reset:
    // - DriveTrain encoders
    // - DriveTrain RobotPose
    // - navX
    // - LiDAR localization
    //
    // It only creates a NEW fusion origin at the robot's
    // current position.
    // =====================================================

    public void startFusion()
    {
        fusedX = 0.0;
        fusedY = 0.0;
        fusedHeading = 0.0;

        previousLeftDistance =
                driveTrain.getLeftEncoderDistance();

        previousRightDistance =
                driveTrain.getRightEncoderDistance();

        previousBackDistance =
                driveTrain.getBackEncoderDistance();

        encoderStartHeading =
                driveTrain.getRobotHeading();

        lidarReferenceReady = false;

        lastLidarAlignedX = 0.0;
        lastLidarAlignedY = 0.0;

        lastCorrectionErrorM = 0.0;
        lastCorrectionAppliedM = 0.0;

        acceptedLidarCorrections = 0;
        rejectedLidarCorrections = 0;
        rejectedEncoderSteps = 0;

        lastProcessedLidarSampleId = -1;
        lastLidarResetId = -1;
        lastSeenLidarSampleId = -1;

        /*
         * Do NOT consume a pending external reset request here.
         *
         * startFusion() is called when FUSION_LOCALIZATION mode
         * begins. A Python reset request may arrive immediately
         * afterwards, so the request must be handled explicitly
         * by checkExternalResetRequest().
         */
        lastFusionResetRequestId =
                (long) fusionTable
                        .getEntry("ResetAckId")
                        .getDouble(-1.0);

        fusionActive = true;

        SmartDashboard.putString(
                "FusionLocalization/Status",
                "RUNNING"
        );
    }


    public void resetFusionPose()
    {
        startFusion();
    }


    public void stopFusion()
    {
        fusionActive = false;

        fusionTable
                .getEntry("ResetRequested")
                .setBoolean(false);

        SmartDashboard.putString(
                "FusionLocalization/Status",
                "STOPPED"
        );
    }


    // =====================================================
    // NORMALIZE HEADING
    // =====================================================

    private double normalizeHeading(
            double angleDeg)
    {
        while (angleDeg > 180.0)
        {
            angleDeg -= 360.0;
        }

        while (angleDeg < -180.0)
        {
            angleDeg += 360.0;
        }

        return angleDeg;
    }


    // =====================================================
    // ROTATE A WORLD VECTOR INTO ANOTHER FRAME
    //
    // Same clockwise-positive coordinate convention already
    // used by your current robot odometry.
    //
    // Input:
    //     worldX / worldY
    //
    // Output:
    //     [0] = rotated X
    //     [1] = rotated Y
    // =====================================================

    private double[] rotateIntoFrame(
            double worldX,
            double worldY,
            double frameRotationDeg)
    {
        double angleRad =
                Math.toRadians(
                        frameRotationDeg
                );

        double cos =
                Math.cos(
                        angleRad
                );

        double sin =
                Math.sin(
                        angleRad
                );

        double rotatedX =
                worldX
                *
                cos
                -
                worldY
                *
                sin;

        double rotatedY =
                worldX
                *
                sin
                +
                worldY
                *
                cos;

        return new double[]
        {
            rotatedX,
            rotatedY
        };
    }


    // =====================================================
    // KEYBOARD COMMAND STATE
    // =====================================================

    private double getCommandX()
    {
        return keyboardTable
                .getEntry("X")
                .getDouble(0.0);
    }


    private double getCommandY()
    {
        return keyboardTable
                .getEntry("Y")
                .getDouble(0.0);
    }


    private double getCommandZ()
    {
        return keyboardTable
                .getEntry("Z")
                .getDouble(0.0);
    }


    private boolean isTranslationCommanded()
    {
        return (
                Math.abs(getCommandX())
                >=
                COMMAND_THRESHOLD
                ||
                Math.abs(getCommandY())
                >=
                COMMAND_THRESHOLD
        );
    }


    private boolean isRotationCommanded()
    {
        return (
                Math.abs(getCommandZ())
                >=
                COMMAND_THRESHOLD
        );
    }


    private boolean isPureRotationCommanded()
    {
        return (
                isRotationCommanded()
                &&
                !isTranslationCommanded()
        );
    }


    private boolean isIdleCommand()
    {
        return (
                !isTranslationCommanded()
                &&
                !isRotationCommanded()
        );
    }


    // =====================================================
    // ENCODER PREDICTION
    //
    // Encoder odometry provides the fast continuous movement.
    //
    // IMPORTANT:
    // There is NO keyboard-based X/Y freeze in this version.
    //
    // Actual encoder motion is always processed.
    // LiDAR then corrects accumulated odometry error.
    // =====================================================

    private void updateEncoderPrediction()
    {
        // -------------------------------------------------
        // READ RAW THREE-WHEEL ENCODER DISTANCES
        // -------------------------------------------------

        double currentLeft =
                driveTrain.getLeftEncoderDistance();

        double currentRight =
                driveTrain.getRightEncoderDistance();

        double currentBack =
                driveTrain.getBackEncoderDistance();


        // -------------------------------------------------
        // WHEEL MOVEMENT SINCE PREVIOUS ROBOT LOOP
        // -------------------------------------------------

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


        /*
         * ALWAYS consume the latest encoder values.
         *
         * This is critical for Q/E:
         *
         * Q/E may physically make the wheels move, but because
         * pure rotation must NOT move the X/Y route, those
         * encoder changes are deliberately ignored.
         *
         * Updating the baseline here prevents that ignored
         * rotation movement from becoming a position jump when
         * W/A/S/D is pressed afterwards.
         */

        previousLeftDistance =
                currentLeft;

        previousRightDistance =
                currentRight;

        previousBackDistance =
                currentBack;


        // -------------------------------------------------
        // HEADING IS ALWAYS navX
        // -------------------------------------------------

        double previousFusionHeading =
                fusedHeading;

        double currentFusionHeading =
                normalizeHeading(
                        driveTrain.getRobotHeading()
                        -
                        encoderStartHeading
                );


        fusedHeading =
                currentFusionHeading;


        // -------------------------------------------------
        // NO COMMAND-BASED X/Y LOCK
        //
        // Fusion is being tested as a real sensor estimator.
        //
        // Even during:
        // - pure Q/E rotation
        // - idle
        // - wheel slip
        // - a collision
        //
        // we still evaluate the actual wheel encoder deltas.
        //
        // In an ideal in-place rotation the 3-wheel geometry
        // should naturally produce approximately zero X/Y.
        //
        // If wheel imperfections produce false translation,
        // LiDAR is allowed to correct it later.
        // -------------------------------------------------


        // -------------------------------------------------
        // THREE-WHEEL HOLONOMIC ODOMETRY
        //
        // Robot-local:
        //
        // +X = crab right
        // +Y = forward
        // -------------------------------------------------

        double localYmm =
                (
                    deltaLeft
                    -
                    deltaRight
                )
                /
                2.0;


        double localXmm =
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


        double localX =
                localXmm
                *
                ENCODER_TO_METERS;

        double localY =
                localYmm
                *
                ENCODER_TO_METERS;


        double encoderStepM =
                Math.hypot(
                        localX,
                        localY
                );


        // -------------------------------------------------
        // TINY ENCODER NOISE
        // -------------------------------------------------

        if (encoderStepM < ENCODER_DELTA_DEADBAND_M)
        {
            return;
        }


        // -------------------------------------------------
        // IMPOSSIBLE ONE-CYCLE JUMP
        // -------------------------------------------------

        if (encoderStepM > MAX_ENCODER_STEP_M)
        {
            rejectedEncoderSteps++;

            return;
        }


        // -------------------------------------------------
        // USE MIDPOINT HEADING FOR MOVE + TURN
        //
        // This improves W+Q / W+E / A+Q etc.
        // -------------------------------------------------

        double headingDelta =
                normalizeHeading(
                        currentFusionHeading
                        -
                        previousFusionHeading
                );


        double midpointHeading =
                normalizeHeading(
                        previousFusionHeading
                        +
                        (
                            headingDelta
                            /
                            2.0
                        )
                );


        double headingRad =
                Math.toRadians(
                        midpointHeading
                );


        // -------------------------------------------------
        // ROBOT-LOCAL MOVEMENT -> FUSION WORLD FRAME
        //
        // Same convention as the existing drivetrain:
        //
        // worldX = localX*cos + localY*sin
        // worldY = -localX*sin + localY*cos
        // -------------------------------------------------

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


        fusedX +=
                worldDeltaX;

        fusedY +=
                worldDeltaY;
    }


    // =====================================================
    // READ LIDAR
    // =====================================================

    private double getLidarX()
    {
        return lidarTable
                .getEntry("RobotX")
                .getDouble(9999.0);
    }


    private double getLidarY()
    {
        return lidarTable
                .getEntry("RobotY")
                .getDouble(9999.0);
    }


    private double getLidarHeading()
    {
        return lidarTable
                .getEntry("RobotHeading")
                .getDouble(9999.0);
    }


    private boolean getLidarValid()
    {
        return lidarTable
                .getEntry("LocalizationValid")
                .getBoolean(false);
    }


    private long getLidarSampleId()
    {
        return (long) lidarTable
                .getEntry("SampleId")
                .getDouble(-1.0);
    }


    private long getLidarResetId()
    {
        return (long) lidarTable
                .getEntry("ResetId")
                .getDouble(-1.0);
    }


    private double getLidarIcpError()
    {
        return lidarTable
                .getEntry("ICPError")
                .getDouble(9999.0);
    }


    private double getLidarInlierRatio()
    {
        return lidarTable
                .getEntry("InlierRatio")
                .getDouble(0.0);
    }


    // =====================================================
    // CREATE LIDAR REFERENCE
    //
    // This may happen immediately when fusion starts, or later
    // if the LiDAR localization is not valid yet.
    // =====================================================

    private void createLidarReference(
            double lidarX,
            double lidarY,
            double lidarHeading)
    {
        lidarReferenceX =
                lidarX;

        lidarReferenceY =
                lidarY;

        lidarReferenceHeading =
                lidarHeading;


        fusedXAtLidarReference =
                fusedX;

        fusedYAtLidarReference =
                fusedY;

        fusedHeadingAtLidarReference =
                fusedHeading;


        /*
         * At the same physical robot orientation:
         *
         * LiDAR says heading = lidarReferenceHeading
         * Fusion says heading = fusedHeadingAtLidarReference
         *
         * Their difference tells us how the two coordinate
         * frames are rotated relative to each other.
         */

        lidarToFusionRotationDeg =
                normalizeHeading(
                        lidarReferenceHeading
                        -
                        fusedHeadingAtLidarReference
                );


        lastLidarAlignedX =
                fusedX;

        lastLidarAlignedY =
                fusedY;


        lidarReferenceReady =
                true;
    }


    // =====================================================
    // LIDAR CORRECTION
    //
    // Encoder odometry predicts continuously.
    // Every NEW valid LiDAR sample can correct that prediction.
    //
    // CRITICAL COLLISION FIX:
    // We detect a new LiDAR observation using SampleId, NOT by
    // checking whether LiDAR X/Y changed. If the robot hits a
    // box, the wheels may spin while LiDAR correctly reports
    // the SAME X/Y. That unchanged LiDAR pose is exactly the
    // evidence needed to pull encoder drift back.
    // =====================================================

    private void updateLidarCorrection()
    {
        lidarValid =
                getLidarValid();


        double lidarX =
                getLidarX();

        double lidarY =
                getLidarY();

        double lidarHeading =
                getLidarHeading();

        long lidarSampleId =
                getLidarSampleId();

        long lidarResetId =
                getLidarResetId();


        double lidarIcpError =
                getLidarIcpError();

        double lidarInlierRatio =
                getLidarInlierRatio();


        lastSeenLidarSampleId =
                lidarSampleId;


        boolean lidarValuesAvailable =
                lidarX != 9999.0
                &&
                lidarY != 9999.0
                &&
                lidarHeading != 9999.0
                &&
                lidarSampleId >= 0
                &&
                lidarResetId >= 0;


        if (!lidarValuesAvailable)
        {
            lastCorrectionAppliedM =
                    0.0;

            return;
        }


        // -------------------------------------------------
        // LiDAR ICP was reset in Python.
        // Drop the old alignment and wait for a fresh valid
        // LiDAR pose to define the new frame reference.
        // -------------------------------------------------

        if (lidarResetId != lastLidarResetId)
        {
            lastLidarResetId =
                    lidarResetId;

            lidarReferenceReady =
                    false;

            lastProcessedLidarSampleId =
                    -1;
        }


        if (!lidarValid)
        {
            lastCorrectionAppliedM =
                    0.0;

            return;
        }


        // -------------------------------------------------
        // Process each Python LiDAR localization update once.
        // -------------------------------------------------

        if (lidarSampleId == lastProcessedLidarSampleId)
        {
            lastCorrectionAppliedM =
                    0.0;

            return;
        }


        lastProcessedLidarSampleId =
                lidarSampleId;


        // -------------------------------------------------
        // FIRST VALID LIDAR POSE / NEW RESET REFERENCE
        // -------------------------------------------------

        if (!lidarReferenceReady)
        {
            createLidarReference(
                    lidarX,
                    lidarY,
                    lidarHeading
            );

            lastCorrectionErrorM =
                    0.0;

            lastCorrectionAppliedM =
                    0.0;

            return;
        }


        // -------------------------------------------------
        // LIDAR QUALITY GATE
        //
        // IMPORTANT:
        //
        // Correction is based on SENSOR QUALITY, not keyboard
        // commands.
        //
        // A fresh LiDAR observation is trusted when:
        //
        // - Python says localization is valid
        // - ICP residual is reasonable
        // - enough matched points agree
        //
        // This allows fusion to correct:
        //
        // - rotation-induced encoder translation
        // - idle encoder drift
        // - wheel slip
        // - collision drift
        //
        // without hard-coding what X/Y "should" do based on
        // W/A/S/D/Q/E.
        // -------------------------------------------------

        boolean lidarQualityGood =
                lidarIcpError
                <=
                MAX_LIDAR_ICP_ERROR_M
                &&
                lidarInlierRatio
                >=
                MIN_LIDAR_INLIER_RATIO;


        if (!lidarQualityGood)
        {
            rejectedLidarCorrections++;

            lastCorrectionAppliedM =
                    0.0;

            return;
        }


        // -------------------------------------------------
        // LIDAR DISPLACEMENT FROM ITS REFERENCE
        // -------------------------------------------------

        double lidarDeltaWorldX =
                lidarX
                -
                lidarReferenceX;

        double lidarDeltaWorldY =
                lidarY
                -
                lidarReferenceY;


        // -------------------------------------------------
        // ALIGN LIDAR COORDINATES WITH FUSION COORDINATES
        // -------------------------------------------------

        double[] lidarDeltaFusion =
                rotateIntoFrame(
                        lidarDeltaWorldX,
                        lidarDeltaWorldY,
                        lidarToFusionRotationDeg
                );


        double lidarAlignedX =
                fusedXAtLidarReference
                +
                lidarDeltaFusion[0];

        double lidarAlignedY =
                fusedYAtLidarReference
                +
                lidarDeltaFusion[1];


        lastLidarAlignedX =
                lidarAlignedX;

        lastLidarAlignedY =
                lidarAlignedY;


        // -------------------------------------------------
        // DIFFERENCE BETWEEN ENCODER PREDICTION AND LIDAR
        // -------------------------------------------------

        double errorX =
                lidarAlignedX
                -
                fusedX;

        double errorY =
                lidarAlignedY
                -
                fusedY;


        double errorDistance =
                Math.hypot(
                        errorX,
                        errorY
                );


        lastCorrectionErrorM =
                errorDistance;


        // -------------------------------------------------
        // SMALL DIFFERENCE -> ignore LiDAR jitter
        // -------------------------------------------------

        if (errorDistance < LIDAR_CORRECTION_DEADBAND_M)
        {
            lastCorrectionAppliedM =
                    0.0;

            return;
        }


        // -------------------------------------------------
        // EXTREME INNOVATION -> likely wrong association/frame
        //
        // Normal wheel-slip / collision disagreement is allowed
        // through and corrected gradually.
        //
        // Only a very large mismatch is rejected.
        // -------------------------------------------------

        if (errorDistance > MAX_LIDAR_INNOVATION_M)
        {
            rejectedLidarCorrections++;

            lastCorrectionAppliedM =
                    0.0;

            return;
        }


        // -------------------------------------------------
        // GRADUAL COMPLEMENTARY CORRECTION
        // -------------------------------------------------

        double correctionX =
                errorX
                *
                LIDAR_CORRECTION_GAIN;

        double correctionY =
                errorY
                *
                LIDAR_CORRECTION_GAIN;


        double correctionMagnitude =
                Math.hypot(
                        correctionX,
                        correctionY
                );


        if (correctionMagnitude > MAX_LIDAR_CORRECTION_STEP_M)
        {
            double scale =
                    MAX_LIDAR_CORRECTION_STEP_M
                    /
                    correctionMagnitude;

            correctionX *=
                    scale;

            correctionY *=
                    scale;

            correctionMagnitude =
                    MAX_LIDAR_CORRECTION_STEP_M;
        }


        fusedX +=
                correctionX;

        fusedY +=
                correctionY;


        lastCorrectionAppliedM =
                correctionMagnitude;

        acceptedLidarCorrections++;
    }


    // =====================================================
    // GETTERS
    // =====================================================

    public double getFusedX()
    {
        return fusedX;
    }


    public double getFusedY()
    {
        return fusedY;
    }


    public double getFusedHeading()
    {
        return fusedHeading;
    }


    public boolean isFusionActive()
    {
        return fusionActive;
    }


    public boolean isLidarValid()
    {
        return lidarValid;
    }


    public boolean isLidarReferenceReady()
    {
        return lidarReferenceReady;
    }


    public double getLastCorrectionErrorM()
    {
        return lastCorrectionErrorM;
    }


    // =====================================================
    // CHECK EXTERNAL FULL-RESET REQUEST
    //
    // Returns true when a reset was performed this loop.
    // =====================================================

    private void performExternalReset(
            long requestId)
    {
        // -------------------------------------------------
        // Capture CURRENT encoder odometry as the new origin.
        // -------------------------------------------------

        previousLeftDistance =
                driveTrain.getLeftEncoderDistance();

        previousRightDistance =
                driveTrain.getRightEncoderDistance();

        previousBackDistance =
                driveTrain.getBackEncoderDistance();


        // -------------------------------------------------
        // Capture CURRENT navX direction as heading 0°.
        // -------------------------------------------------

        encoderStartHeading =
                driveTrain.getRobotHeading();


        // -------------------------------------------------
        // Reset actual fused pose.
        // -------------------------------------------------

        fusedX = 0.0;
        fusedY = 0.0;
        fusedHeading = 0.0;


        // -------------------------------------------------
        // Force LiDAR to establish a fresh alignment after
        // Python's LiDAR ICP reset.
        // -------------------------------------------------

        lidarReferenceReady = false;

        lastLidarAlignedX = 0.0;
        lastLidarAlignedY = 0.0;

        lastCorrectionErrorM = 0.0;
        lastCorrectionAppliedM = 0.0;

        acceptedLidarCorrections = 0;
        rejectedLidarCorrections = 0;
        rejectedEncoderSteps = 0;

        lastProcessedLidarSampleId = -1;
        lastSeenLidarSampleId = -1;

        /*
         * Do NOT force lastLidarResetId to -1 here.
         *
         * Read the CURRENT Python ResetId so the next normal
         * LiDAR sample is treated as belonging to this freshly
         * reset frame rather than repeatedly resetting again.
         */
        lastLidarResetId =
                getLidarResetId();


        // -------------------------------------------------
        // Mark request handled and acknowledge it.
        // -------------------------------------------------

        lastFusionResetRequestId =
                requestId;


        fusionTable
                .getEntry("ResetAckId")
                .setDouble(
                        requestId
                );


        fusionTable
                .getEntry("ResetRequested")
                .setBoolean(false);


        SmartDashboard.putNumber(
                "FusionLocalization/ResetAckId",
                requestId
        );


        SmartDashboard.putString(
                "FusionLocalization/Status",
                "RESET_COMPLETE"
        );
    }


    private boolean checkExternalResetRequest()
    {
        boolean resetRequested =
                fusionTable
                        .getEntry("ResetRequested")
                        .getBoolean(false);


        long requestId =
                (long) fusionTable
                        .getEntry("ResetRequestId")
                        .getDouble(-1.0);


        if (!resetRequested)
        {
            return false;
        }


        if (requestId < 0)
        {
            return false;
        }


        /*
         * If Python repeats the same request because of a
         * NetworkTables retry, acknowledge it again but do not
         * reset the pose twice.
         */
        if (requestId == lastFusionResetRequestId)
        {
            fusionTable
                    .getEntry("ResetAckId")
                    .setDouble(
                            requestId
                    );

            fusionTable
                    .getEntry("ResetRequested")
                    .setBoolean(false);

            return false;
        }


        performExternalReset(
                requestId
        );


        return true;
    }


    // =====================================================
    // PUBLISH FUSED POSE
    // =====================================================

    private void publishFusionPose()
    {
        fusionTable
                .getEntry("X")
                .setDouble(
                        fusedX
                );

        fusionTable
                .getEntry("Y")
                .setDouble(
                        fusedY
                );

        fusionTable
                .getEntry("Heading")
                .setDouble(
                        fusedHeading
                );

        fusionTable
                .getEntry("Active")
                .setBoolean(
                        fusionActive
                );

        fusionTable
                .getEntry("LidarValid")
                .setBoolean(
                        lidarValid
                );

        fusionTable
                .getEntry("LidarReferenceReady")
                .setBoolean(
                        lidarReferenceReady
                );

        fusionTable
                .getEntry("LidarAlignedX")
                .setDouble(
                        lastLidarAlignedX
                );

        fusionTable
                .getEntry("LidarAlignedY")
                .setDouble(
                        lastLidarAlignedY
                );

        fusionTable
                .getEntry("CorrectionErrorM")
                .setDouble(
                        lastCorrectionErrorM
                );

        fusionTable
                .getEntry("CorrectionAppliedM")
                .setDouble(
                        lastCorrectionAppliedM
                );

        fusionTable
                .getEntry("AcceptedLidarCorrections")
                .setDouble(
                        acceptedLidarCorrections
                );

        fusionTable
                .getEntry("RejectedLidarCorrections")
                .setDouble(
                        rejectedLidarCorrections
                );

        fusionTable
                .getEntry("RejectedEncoderSteps")
                .setDouble(
                        rejectedEncoderSteps
                );

        fusionTable
                .getEntry("LastLidarSampleId")
                .setDouble(
                        lastSeenLidarSampleId
                );

        fusionTable
                .getEntry("LastLidarResetId")
                .setDouble(
                        lastLidarResetId
                );

        fusionTable
                .getEntry("LastFusionResetRequestId")
                .setDouble(
                        lastFusionResetRequestId
                );

        fusionTable
                .getEntry("ResetPending")
                .setBoolean(
                        fusionTable
                                .getEntry("ResetRequested")
                                .getBoolean(false)
                );


        /*
         * Kept for compatibility/debugging only.
         *
         * XYLocked is always false in this sensor-driven fusion
         * version because keyboard state no longer freezes the
         * estimator.
         */
        fusionTable
                .getEntry("XYLocked")
                .setBoolean(false);

        fusionTable
                .getEntry("PureRotation")
                .setBoolean(
                        isPureRotationCommanded()
                );

        fusionTable
                .getEntry("EstimatorMode")
                .setString(
                        "SENSOR_DRIVEN_NO_COMMAND_GATING"
                );

        fusionTable
                .getEntry("LidarICPError")
                .setDouble(
                        getLidarIcpError()
                );

        fusionTable
                .getEntry("LidarInlierRatio")
                .setDouble(
                        getLidarInlierRatio()
                );

        fusionTable
                .getEntry("CommandX")
                .setDouble(
                        getCommandX()
                );

        fusionTable
                .getEntry("CommandY")
                .setDouble(
                        getCommandY()
                );

        fusionTable
                .getEntry("CommandZ")
                .setDouble(
                        getCommandZ()
                );


        // -------------------------------------------------
        // SMARTDASHBOARD
        // -------------------------------------------------

        SmartDashboard.putNumber(
                "FusionLocalization/X",
                fusedX
        );

        SmartDashboard.putNumber(
                "FusionLocalization/Y",
                fusedY
        );

        SmartDashboard.putNumber(
                "FusionLocalization/Heading",
                fusedHeading
        );

        SmartDashboard.putBoolean(
                "FusionLocalization/LidarValid",
                lidarValid
        );

        SmartDashboard.putBoolean(
                "FusionLocalization/LidarReferenceReady",
                lidarReferenceReady
        );

        SmartDashboard.putNumber(
                "FusionLocalization/CorrectionErrorM",
                lastCorrectionErrorM
        );

        SmartDashboard.putNumber(
                "FusionLocalization/CorrectionAppliedM",
                lastCorrectionAppliedM
        );

        SmartDashboard.putNumber(
                "FusionLocalization/RejectedEncoderSteps",
                rejectedEncoderSteps
        );

        SmartDashboard.putNumber(
                "FusionLocalization/LastLidarSampleId",
                lastSeenLidarSampleId
        );


        SmartDashboard.putString(
                "FusionLocalization/EstimatorMode",
                "SENSOR_DRIVEN_NO_COMMAND_GATING"
        );


        SmartDashboard.putNumber(
                "FusionLocalization/LidarICPError",
                getLidarIcpError()
        );


        SmartDashboard.putNumber(
                "FusionLocalization/LidarInlierRatio",
                getLidarInlierRatio()
        );


        SmartDashboard.putNumber(
                "FusionLocalization/LidarAlignedX",
                lastLidarAlignedX
        );


        SmartDashboard.putNumber(
                "FusionLocalization/LidarAlignedY",
                lastLidarAlignedY
        );
    }


    // =====================================================
    // PERIODIC
    // =====================================================

    @Override
    public void periodic()
    {
        if (!fusionActive)
        {
            fusionTable
                    .getEntry("Active")
                    .setBoolean(false);

            return;
        }


        // -------------------------------------------------
        // Synchronized full reset requested by Python.
        //
        // Process it BEFORE reading encoder deltas so the new
        // origin starts cleanly at the current physical pose.
        // -------------------------------------------------

        if (checkExternalResetRequest())
        {
            publishFusionPose();

            return;
        }


        // 1. Fast continuous prediction from encoders + navX
        updateEncoderPrediction();


        // 2. LiDAR ICP correction when a valid new LiDAR
        //    localization estimate is available
        updateLidarCorrection();


        // 3. Publish final fused X/Y/Heading
        publishFusionPose();
    }
}
