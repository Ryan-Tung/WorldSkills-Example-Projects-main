package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class MoveYAxis extends CommandBase
{
    private static final DriveTrain drive =
            RobotContainer.driveTrain;


    private final double targetDistance;
    private final double targetYaw;


    private final PIDController pidY;
    private final PIDController pidYaw;


    public MoveYAxis(
            double distance_mm,
            double heading_deg)
    {
        targetDistance =
                distance_mm;


        targetYaw =
                heading_deg;


        addRequirements(
                drive
        );


        pidY =
                new PIDController(
                        0.01,
                        0.0,
                        0.0
                );


        pidY.setTolerance(
                10.0
        );


        pidYaw =
                new PIDController(
                        0.01,
                        0.0,
                        0.0
                );


        pidYaw.setTolerance(
                1.0
        );
    }


    @Override
    public void initialize()
    {
        drive.resetEncoders();

        drive.resetYaw();


        pidY.reset();

        pidYaw.reset();
    }


    @Override
    public void execute()
    {
        double currentY =
                drive
                        .getAverageForwardEncoderDistance();


        double speedY =
                MathUtil.clamp(
                        pidY.calculate(
                                currentY,
                                targetDistance
                        ),
                        -0.5,
                        0.5
                );


        double speedZ =
                MathUtil.clamp(
                        pidYaw.calculate(
                                drive.getYaw(),
                                targetYaw
                        ),
                        -1.0,
                        1.0
                );


        drive.holonomicDrive(
                0.0,
                speedY,
                speedZ
        );


        SmartDashboard.putNumber(
                "Move Y Target",
                targetDistance
        );


        SmartDashboard.putNumber(
                "Move Y Distance",
                currentY
        );


        SmartDashboard.putNumber(
                "Move Y Yaw",
                drive.getYaw()
        );
    }


    @Override
    public boolean isFinished()
    {
        return pidY
                .atSetpoint();
    }


    @Override
    public void end(
            boolean interrupted)
    {
        drive.holonomicDrive(
                0.0,
                0.0,
                0.0
        );
    }
}