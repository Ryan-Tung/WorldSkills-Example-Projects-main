package frc.robot.subsystems;

import com.studica.frc.Cobra;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CobraTest extends SubsystemBase
{
    private Cobra cobra;


    // =====================================================
    // BLACK TAPE VOLTAGE THRESHOLD
    // =====================================================

    private static final double BLACK_VOLTAGE_THRESHOLD =
            2.0;


    // =====================================================
    // CONSTRUCTOR
    // =====================================================

    public CobraTest()
    {
        cobra =
                new Cobra();
    }


    // =====================================================
    // RAW SENSOR VALUES
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
    // VOLTAGE VALUES
    // =====================================================

    public double getVoltage0()
    {
        return cobra.getVoltage(0);
    }


    public double getVoltage1()
    {
        return cobra.getVoltage(1);
    }


    public double getVoltage2()
    {
        return cobra.getVoltage(2);
    }


    public double getVoltage3()
    {
        return cobra.getVoltage(3);
    }


    // =====================================================
    // CHECK ONE SENSOR
    // =====================================================

    private boolean isBlackVoltage(
            double voltage)
    {
        return voltage
                <
                BLACK_VOLTAGE_THRESHOLD;
    }


    // =====================================================
    // BLACK TAPE DETECTED?
    // =====================================================

    public boolean blackTapeDetected()
    {
        double voltage0 =
                getVoltage0();

        double voltage1 =
                getVoltage1();

        double voltage2 =
                getVoltage2();

        double voltage3 =
                getVoltage3();


        return
                isBlackVoltage(voltage0)
                ||
                isBlackVoltage(voltage1)
                ||
                isBlackVoltage(voltage2)
                ||
                isBlackVoltage(voltage3);
    }


    // =====================================================
    // DASHBOARD
    // =====================================================

    @Override
    public void periodic()
    {
        // =================================================
        // READ RAW VALUES
        // =================================================

        double sensor0 =
                getSensor0();

        double sensor1 =
                getSensor1();

        double sensor2 =
                getSensor2();

        double sensor3 =
                getSensor3();


        // =================================================
        // READ VOLTAGES
        // =================================================

        double voltage0 =
                getVoltage0();

        double voltage1 =
                getVoltage1();

        double voltage2 =
                getVoltage2();

        double voltage3 =
                getVoltage3();


        // =================================================
        // CHECK BLACK TAPE
        // =================================================

        boolean blackDetected =
                isBlackVoltage(voltage0)
                ||
                isBlackVoltage(voltage1)
                ||
                isBlackVoltage(voltage2)
                ||
                isBlackVoltage(voltage3);


        // =================================================
        // RAW VALUES
        // =================================================

        SmartDashboard.putNumber(
                "Cobra 0",
                sensor0
        );


        SmartDashboard.putNumber(
                "Cobra 1",
                sensor1
        );


        SmartDashboard.putNumber(
                "Cobra 2",
                sensor2
        );


        SmartDashboard.putNumber(
                "Cobra 3",
                sensor3
        );


        // =================================================
        // VOLTAGE VALUES
        // =================================================

        SmartDashboard.putNumber(
                "Cobra V0",
                voltage0
        );


        SmartDashboard.putNumber(
                "Cobra V1",
                voltage1
        );


        SmartDashboard.putNumber(
                "Cobra V2",
                voltage2
        );


        SmartDashboard.putNumber(
                "Cobra V3",
                voltage3
        );


        // =================================================
        // BLACK TAPE STATUS
        // =================================================

        SmartDashboard.putBoolean(
                "Black Tape",
                blackDetected
        );
    }
}