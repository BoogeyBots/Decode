package org.firstinspires.ftc.teamcode.Meeturi;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp
public class TeleOP_pedro extends OpMode {
    private Follower follower;
    public static Pose startingPose;
    double targetHeading = Math.toRadians(-32); // Radians
    PIDFController controller = new PIDFController(follower.constants.coefficientsHeadingPIDF);
    boolean headingLock = true;

    public double getHeadingError() {
        if (follower.getCurrentPath() == null) {
            return 0;
        }

        return (MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading));
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        if(gamepad1.triangle) {
            headingLock = true;
        }

        follower.update();
        if (headingLock)
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run());
        else
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        controller.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        controller.updateError(getHeadingError());
    }
}
