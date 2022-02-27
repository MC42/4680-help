// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveCommand extends CommandBase {
  private double forward;
  private double strafe;
  private double rot;
  private DrivetrainSubsystem drive;

  //hacky deadband with a cap to replace their old library deadband, I hope.
  public double deadband(double deadband, double tolerance) {
    if (deadband > 1) {
      return 1;
    }
    if (deadband < -1) {
      return -1;
    }

    if (deadband > tolerance || deadband < -tolerance) {
      return deadband;
    }

    return 0;
  }

  /** Creates a new DriveCommand. */
  // this command is intended to be run continuously.
  public DriveCommand(DrivetrainSubsystem drive, double forward, double strafe, double rot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
    this.forward = forward;
    this.strafe = strafe;
    this.rot = rot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

        double forward = this.forward;
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = this.strafe;
        strafe = deadband(strafe, .05);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = this.rot;
        rotation = deadband(rotation, .05);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);
  
        //imperfect, but we'd need to command individual modules, which are within the passed subsystem for this command.
        //this.drive.drive(new Translation2d(forward, strafe), rotation, true);
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
