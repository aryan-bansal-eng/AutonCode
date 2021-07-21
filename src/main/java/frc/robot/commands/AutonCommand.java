// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonCommand extends SequentialCommandGroup 
{
  private Drivetrain drivetrain;
  
  public AutonCommand(Drivetrain drivetrain) 
  {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
    addCommands(getTrajectory());
  }

  public Command getTrajectory()
  {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(15.047871380355277,-5.950502781787172, new Rotation2d(-1.0351107626622706,-0.9518259886549671)),
        new Pose2d(13.049036804179842,-7.55670913764243, new Rotation2d(-1.8203672033026255,-0.08328477400730971)),
        new Pose2d(10.205456663073125,-7.521015663067868, new Rotation2d(-1.1421911863859613,0.02379564971637471)),
        new Pose2d(8.14713296260676,-7.616198261933365, new Rotation2d(-0.2974456214546777,0.17846737287280678)),
        new Pose2d(7.742606917428398,-6.771452697002081, new Rotation2d(0.3807303954619874,0.45211734461111064)),
        new Pose2d(8.539761182926933,-5.795831058630739, new Rotation2d(0.9518259886549689,0.4759129943274836)),
        new Pose2d(10.728960956833358,-4.962983318557642, new Rotation2d(1.0945998869532154,-0.30934344631286503)),
        new Pose2d(13.453562849358203,-5.819626708347113, new Rotation2d(0.8804390395058483,0.0))
      ), drivetrain.getTrajectoryConfig()
    );

    RamseteCommand command = new RamseteCommand(
      trajectory, 
      drivetrain::getCurrentPos,
      new RamseteController(2, 0.7),
      drivetrain.getFeedforward(), 
      drivetrain.getKinematics(), 
      drivetrain::getSpeeds, 
      drivetrain.getLeftController(), 
      drivetrain.getRightController(),
      drivetrain::setOutputVoltage
    );
    return command;
  }
}
