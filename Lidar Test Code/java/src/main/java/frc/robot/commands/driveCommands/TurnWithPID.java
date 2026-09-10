package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class TurnWithPID extends CommandBase
{
    //Bring in the Drive Train subsystem
    private static final DriveTrain drive = RobotContainer.driveTrain;

    private double setpointDistance;
    private double setpointYaw; 

    //Create two PID Controllers
    PIDController pidYAxis;
    PIDController pidZAxis;

    public TurnWithPID(double setpointDistance, double epsilonDistance, double setpointYaw, double epsilonYaw)
    {
        this.setpointDistance = setpointDistance;
        this.setpointYaw = setpointYaw;
        addRequirements(drive);

        pidYAxis = new PIDController(0, 0, 0);
        pidYAxis.setTolerance(epsilonDistance);

        // pidZAxis = new PIDController(20, 0, 0);
        pidZAxis = new PIDController(0.03, 0.0, 0);

        pidZAxis.setTolerance(epsilonYaw);
    }

    @Override
    public void initialize()
    {
        drive.resetEncoders();
        drive.resetYaw();
        pidYAxis.reset();
        pidZAxis.reset();
    }

    @Override
    public void execute()
    {
        drive.holonomicDrive(0.0,
         0.0,
         MathUtil.clamp(pidZAxis.calculate(drive.getYaw(), setpointYaw), -1, 1));
         SmartDashboard.putNumber("Yaw", drive.getYaw());
    }

    @Override
    public void end (boolean interrupted)
    {
        drive.setDriveMotorSpeeds(0.0, 0.0, 0.0);;
    }

    @Override
    public boolean isFinished()
    {
        // return pidYAxis.atSetpoint();
        return pidZAxis.atSetpoint();
    }
}