package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utilities.Util.*;
import frc.robot.Robot;
//import frc.robot.subsystems.NeoPixelControl.LedAssignment;
//import frc.robot.subsystems.NeoPixelControl.LedType;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

// See http://docs.limelightvision.io/en/latest/networktables_api.html for the api definition

public class LimeLightVisionComplexWithPose extends SubsystemBase {
    DrivetrainSubsystem drive;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv"); // Is Data valid 0 or 1
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");

    // tshort Side length of shortest side of the fitted bounding box (pixels)
    // tlong Side length of longest side of the fitted bounding box (pixels)
    // thor Horizontal side length of the rough bounding box (0 - 320 pixels)
    // tvert Vertical side length of the rough bounding box (0 - 320 pixels)
    NetworkTableEntry tShort = table.getEntry("tshort");
    NetworkTableEntry tLong = table.getEntry("tlong");
    NetworkTableEntry tHor = table.getEntry("thor");
    NetworkTableEntry tVert = table.getEntry("tvert");

    // tx0 Raw Screen space X
    // ty0 Raw Screen space Y
    // ta0 Area (0% of image to 100% image)
    // ts0 Skew or rotation (-90 degrees to 0 degrees)
    NetworkTableEntry tx0 = table.getEntry("tx0");
    NetworkTableEntry ty0 = table.getEntry("ty0");
    NetworkTableEntry ta0 = table.getEntry("ta0");
    NetworkTableEntry ts0 = table.getEntry("ts0");

    NetworkTableEntry tx1 = table.getEntry("tx1");
    NetworkTableEntry ty1 = table.getEntry("ty1");
    NetworkTableEntry ta1 = table.getEntry("ta1");
    NetworkTableEntry ts1 = table.getEntry("ts1");

    NetworkTableEntry tx2 = table.getEntry("tx2");
    NetworkTableEntry ty2 = table.getEntry("ty2");
    NetworkTableEntry ta2 = table.getEntry("ta2");
    NetworkTableEntry ts2 = table.getEntry("ts2");
    NetworkTableEntry ledMode = table.getEntry("ledMode");
    NetworkTableEntry cornx = table.getEntry("tCornx");
    boolean ledState = true;

    public double x = 0;
    public double y = 0;
    public double area = 0;
    public double skew = 0;
    public double correctedSkew = 0;
    public boolean valid = false;

    public double vShort;
    public double vLong;
    public double vHor;
    public double vVert;

    public double[] rawX = new double[3];
    public double[] rawY = new double[3];
    public double[] rawArea = new double[3];
    public double[] rawSkew = new double[3];

    public LimeLightVisionComplexWithPose(DrivetrainSubsystem drive ) {
        // Get the various keys from the Lime Light

            Set<String> h1 = table.getKeys(0);
            logf("Lime Light started number of keys size: %d \n", h1.size());
            System.out.println(h1);
            cornx.setNumber(0);
        
    }

    @Override
    public void periodic() {
        setValues();
        // Set the led to indicate that we have a valid vision target
        //Robot.neoPixelControl.setLed(LedAssignment.LimeLight, valid ? LedType.GREEN : LedType.RED);
        long period = Robot.count % 25;
        if (period == 6)
            postBase();
        if (period == 14)
            postSides();
        if (period == 20)
            postRaw();
        tasks();

    }



    public void postBase() {
        SmartDashboard.putNumber("LimeL Valid", valid ? 1 : 0);
        SmartDashboard.putNumber("LimeL X", round3(x));
        SmartDashboard.putNumber("LimeL Y", round3(y));
        SmartDashboard.putNumber("LimeL Area", round3(area));
        SmartDashboard.putNumber("LimeL Skew", round3(skew));
    }

    public void postSides() {
        SmartDashboard.putNumber("LimeL Short", round3(vShort));
        SmartDashboard.putNumber("LimeL Long", round3(vLong));
        SmartDashboard.putNumber("LimeL Hor", round3(vHor));
        SmartDashboard.putNumber("LimeL Vert", round3(vVert));
    }

    public void postRaw() {
        SmartDashboard.putNumberArray("Raw X", rawX);
        SmartDashboard.putNumberArray("Raw Y", rawY);
        SmartDashboard.putNumberArray("Raw Area", rawArea);
        SmartDashboard.putNumberArray("Raw Skew", rawSkew);
    }

    private void setValues() {
        valid = (tv.getDouble(0.0) > 0.0);
        if (!valid) // If the data is not valid no need to read
            return;

        // read values periodically
        x = round3(tx.getDouble(0.0));
        y = round3(ty.getDouble(0.0));
        area = round3(ta.getDouble(0.0));
        skew = round3(ts.getDouble(0.0));

        // Correct skew for driving logic
        correctedSkew = skew;
        if (skew < -45)
            correctedSkew = (skew + 90);

        vShort = round3(tShort.getDouble(0.0));
        vLong = round3(tLong.getDouble(0.0));
        vHor = round3(tHor.getDouble(0.0));
        vVert = round3(tVert.getDouble(0.0));

        rawX[0] = round3(tx0.getDouble(0.0));
        rawY[0] = round3(ty0.getDouble(0.0));
        rawArea[0] = round3(ta0.getDouble(0.0));
        rawSkew[0] = round3(ts0.getDouble(0.0));

        rawX[1] = round3(tx1.getDouble(0.0));
        rawY[1] = round3(ty1.getDouble(0.0));
        rawArea[1] = round3(ta1.getDouble(0.0));
        rawSkew[1] = round3(ts1.getDouble(0.0));

        rawX[2] = round3(tx2.getDouble(0.0));
        rawY[2] = round3(ty2.getDouble(0.0));
        rawArea[2] = round3(ta2.getDouble(0.0));
        rawSkew[2] = round3(ts2.getDouble(0.0));
    }

    public boolean tasks() {

        // Left Joy Buttons
        // 5 -- Seek towards target
        // 3 -- Derive towards target
        // 4 -- Move back with a turn
        // 6 -- Make Parallel to target
        // if (Robot.leftJoy.getRawButtonPressed(2)) {
        // ledToggle();
        // }
        // if (Robot.oi.getLimelightSeekLeft()) { // Seeks counterclockwise
        //     limeLightSeek(-1);
        // } else if (Robot.oi.getLimelightSeekRight()) { // Seeks clockwise
        //     limeLightSeek(1);
        // } else if (Robot.oi.getLimelightTracking()) {
        //     limeLightTracking();
            // } else if (Robot.leftJoy.getRawButton(4)) {
            // limeLightTurnTo();
            // } else if (Robot.leftJoy.getRawButton(6)) {
            // limeLightLineUp();
       // } else {
            return false;
      //  }
        //return true;
    }

    public void ledOn() {
        ledMode.setNumber(3);
        ledState = true;
    }

    public void ledOff() {
        ledMode.setNumber(1);
        ledState = false;
    }

    public void ledBlink() {
        ledMode.setNumber(2);
    }

    public void ledToggle() {
        if (ledState)
            ledOff();
        else
            ledOn();
    }

    /**
     * This function implements a simple method of generating driving and steering
     * commands based on the tracking data from a limelight camera.
     */
    double driveCommand = 0.0;
    double steerCommand = 0.0;

    public void limeLightTracking() {
        // These numbers must be tuned for your Robot! Be careful!
        double STEER_K = 0.03; // how hard to turn toward the target //0.03
        final double DRIVE_K = 0.26; // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 15.0; // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.8; // Simple speed limit so we don't drive too fast

        if (!valid) {
            // If invalid data from LimeLight stop the motors
            driveCommand = 0.0;
            steerCommand = 0.0;
            drive.drive(new ChassisSpeeds(0, 0, Math.toRadians(0)));
            return;
        }
        STEER_K *= 2;

        // offset is a how far we should adjust base upon how far away the target is
        double offset = correctedSkew * 2.0; // 2.0
        if (area > 11.0) // 6
            offset /= 2;
        if (area > 13.0) // 7
            offset = 0;

        // x is the number of degrees that you are off the center from the target
        steerCommand = (x + offset) * STEER_K;

        // try to drive forward or backward until the target area reaches our desired
        // area
        driveCommand = (DESIRED_TARGET_AREA - area) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (driveCommand > MAX_DRIVE) {
            driveCommand = MAX_DRIVE;
        }

        // Start with proportional steering
        drive.drive(new ChassisSpeeds(driveCommand, 0, Math.toRadians(steerCommand)));
        // drive.setSpeed(-driveCommand + steerCommand, driveCommand + steerCommand);
        // Robot.drive.setIgnoreNext();
        logf("Valid:%b Drive:%.1f Steer:%.1f X:%.2f Area:%.2f Skew:%.2f correctSkew:%.2f\n", valid, driveCommand,
                steerCommand, x, area, skew, correctedSkew);
    }

    public void limeLightLineUp() {
        if (!valid) {
            // If no valid data from LimeLight stop the motors
            logf("Lineup -- vision invalid\n");
               drive.drive(new ChassisSpeeds(0, 0, Math.toRadians(0)));
            return;
        }
        double delta = vShort - vVert;
        delta = rawY[0] - rawY[1];
        // delta = Math.abs(delta);
        delta *= -3.5;
        if (delta < .2)
            delta *= 2.5;
        // delta *= .075; was it .025??
        logf("Lineup x:%.1f delta:%.1f y0:%.3f y1%.3f short:%.1f vert:%.1f\n", x, delta, rawY[0], rawY[1], vShort,
                vVert);
// TODO fix
        if (x < 0) {
           //drive.setSpeed(delta, delta);
        } else {
           //drive.setSpeed(-delta, -delta);
        }
        // Robot.drive.setIgnoreNext();
    }

    public void limeLightSeek(int direction) { // 1 = right, -1 =left
        double steeringAdjust;
        if (valid) {
            double steerP = 0.03; // how hard to turn toward the target
            if (Math.abs(x) < 2) {
                steerP *= 3;
            }
            if (Math.abs(x) < 0.5) {
                steerP *= 3;
            }
            steeringAdjust = x * steerP;
            logf("Target Found align x:%.1f speed:%.1f\n", x, steeringAdjust);
          //drive.setSpeed(steeringAdjust, steeringAdjust);
           drive.drive(new ChassisSpeeds(0, 0, Math.toRadians( steeringAdjust)));
        } else {
            // We don't see the target, seek for the target by spinning in place
            steeringAdjust = 0.5 * direction;
            logf("Target not found \n", steeringAdjust);
           
           drive.drive(new ChassisSpeeds(0, 0, Math.toRadians(steeringAdjust)));

        }
        // Robot.drive.setIgnoreNext();
    }

    public void limeLightTurnTo() {
        // Turn back back angle
        double angle;
        if (vShort < vVert)
            angle = Math.toDegrees(Math.acos(vShort / vVert));
        else
            angle = Math.toDegrees(Math.acos(vVert / vShort));
        logf("Limelight TurnTo yaw:%.1f angle:%.1f ratio:%.3f\n", drive.m_navx.getYaw(), angle, vShort / vVert);
    }

    FileWriter fileWriter;
    PrintWriter printWriter;

    public void createLimeLightFile() {
        String header = "Time,Elapsed,Yaw,leftPos,rightPos,FrontUltra,BackUltra,Lidar,";
        header += "tv,tx,ty,ta,ts,cors,tShort,tLong,thor,Vert,";
        header += "tx0,ty0,ta0,ts0,tx1,ty1,ta1,ts1,tx2,ty2,ta2,ts2,count";
        DateFormat dateFormat = new SimpleDateFormat("dd-MM-yyyy");
        dateFormat.setTimeZone(TimeZone.getTimeZone("America/New_York"));
        String file = "/home/lvuser/raw-" + dateFormat.format(new Date()) + ".csv";
        try {
            fileWriter = new FileWriter(file, true);
            printWriter = new PrintWriter(fileWriter);
            printWriter.println(header);
        } catch (Exception e) {
            logf("Unable to open file:%s\n", file);
        }

    }

    public void closeLimeLightFile() {
        try {
            fileWriter.close();
        } catch (Exception e) {
            logf("Unable to close file\n");
        }
    }

    public void saveLimeLightData(double elapsed) {
        if (fileWriter == null)
            return;
        StringBuilder str = new StringBuilder(200);
        // "Date,Time,Elapsed,Yaw,leftPos,rightPos,FrontUltra,BackUltra,Lidar,"
        DateFormat dateFormat = new SimpleDateFormat("HH:mm:ss-SSS,");
        dateFormat.setTimeZone(TimeZone.getTimeZone("America/New_York"));
        str.append(dateFormat.format(new Date()));
        // this.timeSinceInitialized()

        str.append(round3(elapsed));
        str.append(',');
        str.append(round2(drive.m_navx.getYaw()));
        str.append(',');
        str.append(drive.getLeft());
        str.append(',');
        str.append(drive.getRight());
        str.append(',');
        //str.append(round3(Robot.frontDist));
        //str.append(',');
        //str.append(round3(Robot.rearDist));
        //str.append(',');
        // if (Robot.lidar != null)
        // str.append(Util.round3(Robot.lidar.getDistanceInches()));
        str.append(',');
        // "tv,tx,ty,ta,ts,tShort,tLong,thor,tVert,
        str.append(valid);
        str.append(',');
        str.append(x);
        str.append(',');
        str.append(y);
        str.append(',');
        str.append(area);
        str.append(',');
        str.append(skew);
        str.append(',');
        str.append(correctedSkew);
        str.append(',');
        str.append(vShort);
        str.append(',');
        str.append(vLong);
        str.append(',');
        str.append(vHor);
        str.append(',');
        str.append(vVert);
        str.append(',');
        // "tx0,ty0,ta0,ts0,tx1,ty1,ta1,ts1,tx2,ty2,ta2,ts2,count"
        for (int i = 0; i <= 2; i++) {
            str.append(rawX[i]);
            str.append(',');
            str.append(rawY[i]);
            str.append(',');
            str.append(rawArea[i]);
            str.append(',');
            str.append(rawSkew[i]);
            str.append(',');
        }
        str.append(Robot.count);
        printWriter.println(str.toString());

    }

}