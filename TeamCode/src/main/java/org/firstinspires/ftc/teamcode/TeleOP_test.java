package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.currentX;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.currentY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Meeturi.Module.PinpointModule;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import java.util.List;
@TeleOp
public class TeleOP_test extends LinearOpMode {
    SampleMecanumDrive d;
    PinpointModule p;
    @Override
    public void runOpMode() throws InterruptedException {
        d = new SampleMecanumDrive(hardwareMap);
        p = new PinpointModule(hardwareMap);

        d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        p.init();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        while (opModeIsActive()) {
            p.update_blue();
            d.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            d.update();

            if(gamepad1.a) {
                p.reset_blue();
            }

            telemetry.addData("x", currentX);
            telemetry.addData("y", currentY);
            telemetry.update();
        }

    }
}
