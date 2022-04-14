/*-------------------------------------------------------------------------------*/
/* Copyright (c) 2021-2022 BHS Devilbotz. All Rights Reserved.                   */
/* Open Source Software - may be modified, commercialized, distributed,          */
/* sub-licensed and used for private use under the terms of the License.md       */
/* file in the root of the source code tree.                                     */
/*                                                                               */
/* You MUST include the original copyright and license files in any and all      */
/* revised/modified code. You may NOT remove this header under any circumstance  */
/* unless explicitly noted                                                       */
/*-------------------------------------------------------------------------------*/

package bhs.devilbotz;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/**
 * This is the main robot class.
 * It is automatically ran when the robot is started, and the correct methods are called.
 *
 * @author Devilbotz
 * @version 1.0.0
 * @since 1.0.0
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private double time;

    private RobotContainer robotContainer;

    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(5);

    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(6);

    // Angle between horizontal and the camera.

    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);


    // How far from the target we want to be

    final double GOAL_RANGE_METERS = Units.feetToMeters(3);


    // Change this to match the name of your camera

    PhotonCamera camera = new PhotonCamera("Logitech_Webcam_C930e");


    // PID constants should be tuned per robot

    final double LINEAR_P = 0.7;

    final double LINEAR_D = 0.0;

    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);


    final double ANGULAR_P = 0.7;

    final double ANGULAR_D = 0.0;

    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    /**
     * This method is run when the robot is first started up and is used for initialization
     *
     * @since 1.0.0
     */
    @Override
    public void robotInit() {
        // Instantiate the RobotContainer.
        robotContainer = new RobotContainer();
        Logger.configureLoggingAndConfig(this, false);
    }


    /**
     * This method is called every robot packet, no matter the mode.
     * This is used for diagnostic purposes.
     * <p>
     * This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     *
     * @since 1.0.0
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.
        CommandScheduler.getInstance().run();
        Logger.updateEntries();
    }

    /**
     * This method is called once when the robot is disabled.
     *
     * @since 1.0.0
     */
    @Override
    public void disabledInit() {
    }

    /**
     * This method is called periodically when the robot is disabled.
     *
     * @since 1.0.0
     */
    @Override
    public void disabledPeriodic() {
    }


    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */

    @Override
    public void autonomousInit() {
    }


    /**
     * This method is called periodically during autonomous.
     *
     * @since 1.0.0
     */
    @Override
    public void autonomousPeriodic() {
    }
    /**
     * This method is called once when the robot is in teleoperated mode.
     *
     * @since 1.0.0
     */
    @Override
    public void teleopInit() {

        camera.setPipelineIndex(3);
    }

    double forwardSpeed;

    double rotationSpeed;
    /**
     * This method is called periodically while the robot is in teleoperated mode.
     *
     * @since 1.0.0
     */
    @Override
    public void teleopPeriodic() {
        if (true) {

            // Vision-alignment mode

            // Query the latest result from PhotonVision

            var result = camera.getLatestResult();


            if (result.hasTargets()) {

                // First calculate range

                double range =

                        PhotonUtils.calculateDistanceToTargetMeters(

                                CAMERA_HEIGHT_METERS,

                                TARGET_HEIGHT_METERS,

                                CAMERA_PITCH_RADIANS,

                                Units.degreesToRadians(result.getBestTarget().getPitch()));



                // Use this range as the measurement we give to the PID controller.

                // -1.0 required to ensure positive PID controller effort _increases_ range

                forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

                System.out.println(forwardSpeed);

                // Also calculate angular power

                // -1.0 required to ensure positive PID controller effort _increases_ yaw

                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

            } else {

                // If we have no targets, stay still.

                forwardSpeed = 0;

                rotationSpeed = 0;

            }


        } else {
            forwardSpeed = 0;
            rotationSpeed = 0;
        }
        robotContainer.getDriveTrain().arcadeDrive(forwardSpeed, rotationSpeed);

    }


    /**
     * This robot is called once when the robot is in test mode.
     *
     * @since 1.0.0
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }


    /**
     * This method is called periodically during test mode.
     *
     * @since 1.0.0
     */
    @Override
    public void testPeriodic() {
    }
}
