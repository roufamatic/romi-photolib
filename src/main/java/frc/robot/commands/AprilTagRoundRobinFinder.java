// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class AprilTagRoundRobinFinder extends SequentialCommandGroup {
  private Drivetrain m_drivetrain;
  private PhotonCamera m_camera;
  private NetworkTableInstance m_ntInstance;
  private int m_currentTag = 1;

  // I bought four stands, so that's how many AprilTags we can find. :-)
  private static final int kMaxTag = 4;

  /**
   * Creates a new Autonomous Drive based on four april tags, numbered 1-4.
   * The robot spins in place looking for the next AprilTag. It pauses for one
   * second when it finds it, then looks for the next one in order, going back
   * to #1 after it finds #4. This repeats indefinitely.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AprilTagRoundRobinFinder(Drivetrain drivetrain) {
    // Can't go anywhere without a drivetrain!
    m_drivetrain = drivetrain;
    // PhotonVision depends on NetworkTables. The robot code hosts the server,
    // while the coprocessor has the client.
    m_ntInstance = NetworkTableInstance.getDefault();
    m_ntInstance.startServer();
    // "teenycam1" is a name I gave the camera using the PhotonVision web UI.
    m_camera = new PhotonCamera(m_ntInstance, "teenycam1");
    // This is the tag number we are currently looking for.
    m_currentTag = 1;

    // This command spins around until it finds the current tag.
    Command hunt = (new TurnDegrees(0.4, 360, m_drivetrain))
        .until(this::foundTag);
    
    // This command waits one second, then changes the current tag to the next
    // highest number, or 1 if we're already at the maximum number.
    Command attack = (new WaitCommand(1))
        .andThen(Commands.runOnce(() -> {
            m_currentTag = m_currentTag == kMaxTag ? 1 : (m_currentTag + 1);
            System.out.printf("Staring at tag #%d", m_currentTag);
        }));
    
    // Finally we make one composite command that performs hunt / attack
    // indefinitely.
    Command start = Commands.sequence(hunt, attack).repeatedly();

    // AprilTagRoundRobinFinder is itself a SequentialCommand, so we can add
    // even more commands here. I don't think they'd actually ever run because
    // of the "repeatedly()" in the start command though!
    addCommands(start);

    // Each command specifies what subsystems it requires.
    addRequirements(drivetrain);
  }

  // This function checks every 20ms (the robot period) to see if the latest
  // camera image contains the current AprilTag.
  private boolean foundTag() {
      PhotonPipelineResult cameraResult = m_camera.getLatestResult();
      if (!cameraResult.hasTargets()) {
        return false;
      }
      for (var t : cameraResult.targets) {
        if (t.getFiducialId() == m_currentTag) {
          return true;
        }
      }
      return false;
  }    
}
