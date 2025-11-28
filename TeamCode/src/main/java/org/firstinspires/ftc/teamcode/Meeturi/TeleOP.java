package org.firstinspires.ftc.teamcode.Meeturi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Meeturi.Module.Constants;
import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.PinpointModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import java.util.List;
@TeleOp
public class TeleOP extends LinearOpMode {
    SampleMecanumDrive drive = null;
    IntakeModule intake = null;
    OuttakeModule outtake = null;
    TurretModule turret = null;
    PinpointModule pinpoint;
    ElapsedTime timer;

    enum STATE {
        trage,
        numaitrage
    }
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        turret = new TurretModule(hardwareMap);
        pinpoint = new PinpointModule(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.init_teleOP();
        outtake.init_teleOP();
        turret.init_teleOP();
        pinpoint.init();

        STATE mode = STATE.trage;

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        boolean switchingState = false;

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            outtake.update();
            pinpoint.update();
            turret.update();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * 0.7
                    )
            );

            drive.update();

            if (gamepad1.right_trigger > 0.01) {
                intake.trage(1);
            }

            else if (gamepad1.left_trigger > 0.01) {
               intake.scuipa(1);
            }

            else {
                intake.stop();
            }

            if(gamepad1.dpad_up) {
                intake.sus();
            }

            if(gamepad1.dpad_down) {
                intake.jos();
            }

            if(gamepad1.a && mode == STATE.trage) {
                Constants.outtake.activated = true;
                switchingState = true;
                timer.reset();
            }

            if(gamepad1.a && mode == STATE.numaitrage) {
                Constants.outtake.activated = false;
                switchingState = true;
                Constants.outtake.target_velocity = 0;
                outtake.blocat();
                timer.reset();
            }


            if (switchingState && timer.seconds() > 0.15) {
                mode = (mode == STATE.trage) ? STATE.numaitrage : STATE.trage;
                switchingState = false;
            }

            if(Constants.outtake.target_velocity <= Constants.outtake.velocity - 20 && Constants.turret.error <= 3) {
                outtake.deblocat();
            }

            if(gamepad1.triangle) {
                pinpoint.recalibration();
            }


//            if(gamepad1.b) {
//                outtake.aproape();
//                timer.reset();
//                shooter = true;
//                intake.sus();
//            }
//
//            if(timer.seconds() > 1.7 && shooter) {
//                outtake.deblocat();
//                intake.trage(1);
//                shooter = false;
//                timer.reset();
//            }


//            if(gamepad1.y) {
//                outtake.stop();
//                outtake.blocat();
//                intake.stop();
//            }

            //tureta manuala
//            if(gamepad1.right_bumper) {
//                turret.manual(1);
//            }
//
//            else if(gamepad1.left_bumper) {
//                turret.manual(-1);
//            }
//
//            else {
//                turret.manual(0);
//            }

            telemetry.addData("Power", turret.getPower());
            telemetry.addData("Error", turret.getErrore());
            telemetry.addData("Grade:", turret.gra());
            telemetry.addData("STATE", mode);
            telemetry.update();
        }
    }
}
