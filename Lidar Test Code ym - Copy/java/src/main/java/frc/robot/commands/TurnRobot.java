package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class TurnRobot extends CommandBase
{
    private static final DriveTrain drive =
            RobotContainer.driveTrain;


    private final double targetYaw;


    private final PIDController pidYaw;


    public TurnRobot(
            double angle_deg)
    {
        targetYaw =
                angle_deg;


        addRequirements(
                drive
        );


        // Same P gain as your working TurnWithPID
        pidYaw =
                new PIDController(
                        0.03,
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


        pidYaw.reset();


        System.out.println(
                "TurnRobot START"
        );
    }


    @Override
    public void execute()
    {
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
                0.0,
                speedZ
        );


        SmartDashboard.putNumber(
                "Turn Yaw",
                drive.getYaw()
        );
    }


    @Override
    public boolean isFinished()
    {
        return pidYaw
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


        System.out.println(
                "TurnRobot END"
        );
    }
}