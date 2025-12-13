package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.activated;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.auto;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.ramp;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.decalation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Meeturi.Module.Constants;
import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.PinpointModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import java.util.List;
@TeleOp
public class TeleOP_red extends LinearOpMode {

    DistanceSensor sensor;
    SampleMecanumDrive drive = null;
    IntakeModule intake = null;
    OuttakeModule outtake = null;
    TurretModule turret = null;
    PinpointModule pinpoint;
    ElapsedTime timer, timer_delta_velocity;
    double distanta_sensor;
    double delta_velocity;
    boolean deschis = false;

    enum STATE {
        trage,
        numaitrage
    }
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(DistanceSensor.class, "senzor_distanta");

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
        //timer_delta_velocity = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        boolean switchingState = false;

        activated = false;
        auto = false;

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
            pinpoint.update_red();
            turret.update_red();

            delta_velocity = velocity - target_velocity - 1;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x * 0.7
                    )
            );

            drive.update();


            if (gamepad1.right_trigger > 0.01 && mode == STATE.trage) {
                intake.trage_intake(1);
                intake.trage_transfer(0.47);
            }

            else if(gamepad1.right_trigger > 0.01 && mode == STATE.numaitrage) {
                intake.trage_intake(1);
                if(distanta < 110) {
                    intake.trage_transfer(1);
                }
                else intake.trage_transfer(1);
            }

            else if (gamepad1.left_trigger > 0.01) {
               intake.scuipa_intake(1);
               intake.scuipa_transfer(1);
            }

            else {
                intake.stop_intake();
                intake.stop_transfer();
            }

            if(gamepad1.dpad_up) {
                intake.sus();
            }

            if(gamepad1.dpad_down) {
                intake.jos();
            }

            if(gamepad1.a && mode == STATE.trage) {
                ramp = false;
                Constants.outtake.activated = true;
                switchingState = true;
                timer.reset();
            }

            if(gamepad1.a && mode == STATE.numaitrage) {
                Constants.outtake.target_velocity = 0;
                deschis = false;
                Constants.outtake.activated = false;
                switchingState = true;
                outtake.blocat();
                timer.reset();
            }


            if (switchingState && timer.seconds() > 0.15) {
                mode = (mode == STATE.trage) ? STATE.numaitrage : STATE.trage;
                switchingState = false;
            }

            if(delta_velocity > 0 && Constants.turret.error <= 3 && Constants.outtake.activated && timer.seconds() > 1) {
                outtake.deblocat();
                deschis = true;
            }

            if(gamepad1.triangle) {
                pinpoint.recalibration();
            }

            if(target_velocity != 0 && activated && delta_velocity < -150 && deschis) {
                outtake.reglare();
            }

            if(gamepad1.right_bumper) {
                decalation += 0.3;
            }

            else if(gamepad1.left_bumper) {
                decalation -= 0.3;
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

            telemetry.addData("V", velocity);
            telemetry.addData("T", target_velocity);
            telemetry.addData("Delta velocity", delta_velocity);
            telemetry.addData("Decalare", decalation);
            telemetry.addData("STATE", mode);
            telemetry.addData("Distance", distanta);
            telemetry.update();
        }
    }
}
