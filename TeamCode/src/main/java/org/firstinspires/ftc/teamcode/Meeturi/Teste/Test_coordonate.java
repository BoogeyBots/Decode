package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Meeturi.Module.Constants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
@TeleOp
public class Test_coordonate extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;
    SampleMecanumDrive drive;
    double currentX, currentY, currentHeading;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        pinpoint.setOffsets(-130.0, -11.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setHeading(-90, AngleUnit.DEGREES);

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            currentX = pinpoint.getPosX(DistanceUnit.INCH);
            currentY = pinpoint.getPosY(DistanceUnit.INCH);
            currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

            if(currentHeading < 0) {
                currentHeading = 360 - Math.abs(currentHeading);
            }

            double relative_angle = Math.toDegrees(Math.atan2(Math.sqrt(2) * Math.abs(1.4197 * currentX + currentY - 137.3728), 1.73 * Math.abs(currentX - currentY)));
            double error = currentHeading - Constants.turret.targetHeading - relative_angle;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * 0.7
                    )
            );

            drive.update();

            if(gamepad1.a) {
                pinpoint.resetPosAndIMU();
            }

            telemetry.addData("X", currentX);
            telemetry.addData("Y", currentY);
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Corectie fata de y = x", relative_angle);
            telemetry.addData("Error", error);

            telemetry.update();


        }

    }
}
