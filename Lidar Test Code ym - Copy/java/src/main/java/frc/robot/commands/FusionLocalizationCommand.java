package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.FusionLocalization;


public class FusionLocalizationCommand extends CommandBase
{
    // =====================================================
    // FUSION SUBSYSTEM
    // =====================================================

    private final FusionLocalization fusionLocalization;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public FusionLocalizationCommand(
            FusionLocalization fusionLocalization)
    {
        this.fusionLocalization =
                fusionLocalization;


        /*
         * This command owns ONLY FusionLocalization.
         *
         * It does NOT require DriveTrain.
         *
         * Therefore:
         *
         * KeyboardDrive
         *       +
         * FusionLocalizationCommand
         *
         * can run at the same time.
         */
        addRequirements(
                fusionLocalization
        );
    }


    // =====================================================
    // INITIALIZE
    // =====================================================

    @Override
    public void initialize()
    {
        // Start a fresh fusion coordinate frame.
        //
        // Fusion pose becomes:
        //
        // X = 0
        // Y = 0
        // Heading = 0

        fusionLocalization
                .startFusion();


        SmartDashboard.putString(
                "FusionLocalizationCommand/Status",
                "RUNNING"
        );


        SmartDashboard.putBoolean(
                "FusionLocalizationCommand/Active",
                fusionLocalization
                        .isFusionActive()
        );
    }


    // =====================================================
    // EXECUTE
    //
    // Actual fusion calculation is performed automatically
    // inside FusionLocalization.periodic().
    // =====================================================

    @Override
    public void execute()
    {
        SmartDashboard.putNumber(
                "FusionLocalizationCommand/X",
                fusionLocalization
                        .getFusedX()
        );


        SmartDashboard.putNumber(
                "FusionLocalizationCommand/Y",
                fusionLocalization
                        .getFusedY()
        );


        SmartDashboard.putNumber(
                "FusionLocalizationCommand/Heading",
                fusionLocalization
                        .getFusedHeading()
        );


        SmartDashboard.putBoolean(
                "FusionLocalizationCommand/LidarValid",
                fusionLocalization
                        .isLidarValid()
        );


        SmartDashboard.putBoolean(
                "FusionLocalizationCommand/LidarReferenceReady",
                fusionLocalization
                        .isLidarReferenceReady()
        );


        SmartDashboard.putNumber(
                "FusionLocalizationCommand/CorrectionErrorM",
                fusionLocalization
                        .getLastCorrectionErrorM()
        );


        SmartDashboard.putBoolean(
                "FusionLocalizationCommand/Active",
                fusionLocalization
                        .isFusionActive()
        );
    }


    // =====================================================
    // END
    // =====================================================

    @Override
    public void end(
            boolean interrupted)
    {
        fusionLocalization
                .stopFusion();


        SmartDashboard.putString(
                "FusionLocalizationCommand/Status",
                interrupted
                        ? "INTERRUPTED"
                        : "FINISHED"
        );


        SmartDashboard.putBoolean(
                "FusionLocalizationCommand/Active",
                false
        );
    }


    // =====================================================
    // KEEP RUNNING
    // =====================================================

    @Override
    public boolean isFinished()
    {
        return false;
    }
}