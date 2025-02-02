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

package bhs.devilbotz.commands;

import bhs.devilbotz.subsystems.DriveTrain;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final DriveTrain drive;
    private final DoubleSupplier left;
    private final DoubleSupplier right;
    private NetworkTableEntry maxSpeed = Shuffleboard.getTab("Drive").add("Speedlimit", 0.95).withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

    public DriveCommand(DriveTrain drive, DoubleSupplier left, DoubleSupplier right) {
        this.drive = drive;
        this.left = left;
        this.right = right;
        addRequirements(this.drive);

    }

    @Override
    public void execute() {
        double r = right.getAsDouble();
        double l = left.getAsDouble();
        double speedlimit = maxSpeed.getDouble(0.95);

        r = r * speedlimit;
        l = l * speedlimit;

        // Forward Snapping
        if ((Math.abs((l - r) / r) < 0.15 || Math.abs((r - l) / l) < 0.15) && ((l > 0.05 || l < -0.05) || (r > 0.05 || r < -0.05))) {
            double oldLeft = l;
            double oldRight = l;

            l = (oldLeft + oldRight) / 2;

            r = (oldLeft + oldRight) / 2;
        }

        // (a*(x^3)+(b-a)*x)*1.1
        double a = 0.2;
        double b = 0.9;

        // Modified curve
        r = (a * (r * r * r) + (b - a) * r) * 1.1;
        l = (a * (l * l * l) + (b - a) * l) * 1.1;

        drive.tankDrive(r, l);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
