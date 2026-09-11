package frc.robot.subsystems;

import com.studica.frc.Cobra;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CobraTest extends SubsystemBase
{
    private Cobra cobra;


    // =====================================================
    // BLACK TAPE THRESHOLD
    // =====================================================

    /*
     * TEMPORARY VALUE.
     *
     * First look at the readings on Shuffleboard
     * for:
     *
     * 1. normal floor
     * 2. black tape
     *
     * Then we will set this properly.
     */
    private static final double BLACK_THRESHOLD =
            1000.0;


    /*
     * Depending on your floor/tape,
     * black may give HIGHER or LOWER values.
     *
     * Start with true.
     *
     * After testing, change to false if black
     * gives a LOWER number than the normal floor.
     */
    private static final boolean BLACK_IS_HIGH =
            true;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public CobraTest()
    {
        cobra =
                new Cobra();
    }


    // =====================================================
    // RAW VALUES
    // =====================================================

    public double getSensor0()
    {
        return cobra.getRawValue(0);
    }


    public double getSensor1()
    {
        return cobra.getRawValue(1);
    }


    public double getSensor2()
    {
        return cobra.getRawValue(2);
    }


    public double getSensor3()
    {
        return cobra.getRawValue(3);
    }


    // =====================================================
    // CHECK ONE SENSOR FOR BLACK
    // =====================================================

    private boolean isBlack(
            double value)
    {
        if (BLACK_IS_HIGH)
        {
            return value
                    >=
                    BLACK_THRESHOLD;
        }
        else
        {
            return value
                    <=
                    BLACK_THRESHOLD;
        }
    }


    // =====================================================
    // BLACK TAPE DETECTED?
    // =====================================================

    public boolean blackTapeDetected()
    {
        return
                isBlack(getSensor0())
                ||
                isBlack(getSensor1())
                ||
                isBlack(getSensor2())
                ||
                isBlack(getSensor3());
    }


    // =====================================================
    // DASHBOARD
    // =====================================================

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber(
                "Cobra 0",
                getSensor0()
        );


        SmartDashboard.putNumber(
                "Cobra 1",
                getSensor1()
        );


        SmartDashboard.putNumber(
                "Cobra 2",
                getSensor2()
        );


        SmartDashboard.putNumber(
                "Cobra 3",
                getSensor3()
        );


        SmartDashboard.putBoolean(
                "Black Tape",
                blackTapeDetected()
        );
    }
}