package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.act_outtake;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.auto;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.nominalvoltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.ramp;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.voltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.velocityX;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.velocityY;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.act_turret;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.decalation;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.error;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.gr;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.trage_gresit;

import static java.lang.Math.abs;

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
@TeleOp (name = "Tiliopa red")
public class TeleOP_red extends LinearOpMode {
    SampleMecanumDrive drive = null;
    IntakeModule intake = null;
    OuttakeModule outtake = null;
    TurretModule turret = null;
    PinpointModule pinpoint;
    ElapsedTime timer, timer_intake, loop;
    double delta_velocity;
    boolean deschis = false;
    boolean target_atins = false;
    boolean intaking = true, intaking_timer = false;

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

        intake.init();
        outtake.init_teleOP();
        turret.init_teleOP();
        pinpoint.init();

        STATE mode = STATE.trage;

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        timer_intake = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        loop = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        boolean switchingState = false;
        boolean bila_in_intake = false;

        act_outtake = false;
        act_turret = false;
        auto = false;
        trage_gresit = false;
        target_atins = false;

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive()) {
            loop.reset();
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            outtake.update_kinematics();
            pinpoint.update_red();
            turret.update_kinematics_red();

            delta_velocity = velocity - target_velocity - 1;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();


            if (gamepad1.right_trigger > 0.01 && mode == STATE.trage && intaking) {
                intake.trage_intake(1);
//                if(intake.a_transfer() < 2.5)
                intake.trage_transfer(0.47);
//                else intake.trage_transfer(0.2);
            }

            else if (gamepad1.left_trigger > 0.01) {
                intake.scuipa_intake(1);
                intake.scuipa_transfer(1);
                intaking = true;
            }

            else {
                intake.stop_intake();
                intake.stop_transfer();
            }


            if(gamepad1.a && mode == STATE.trage) {
                intaking = true;
                intaking_timer = false;
                act_turret = true;
                Constants.outtake.act_outtake = true;
                switchingState = true;
                target_atins = false;
                ramp = false;
                timer.reset();
            }


            if(gamepad1.a && mode == STATE.numaitrage) {
                if(distanta > 120) {
                    Constants.outtake.target_velocity = 1340;
                }
                else {
                    Constants.outtake.target_velocity = 1030;
                }
                deschis = false;
                Constants.outtake.act_outtake = false;
//                act_turret = false;
                switchingState = true;
                outtake.blocat();
                timer.reset();
            }


            if (switchingState && timer.seconds() > 0.15) {
                mode = (mode == STATE.trage) ? STATE.numaitrage : STATE.trage;
                switchingState = false;
            }

            boolean tragemsinoi = (delta_velocity < 40 && delta_velocity > 0 && act_outtake && timer.seconds() > 0.17 && gr > 239 && gr < 544);

            if(tragemsinoi) {
                if(distanta >= 120 && abs(error) <= 3 && turret.getPower() < 0.07) {
                    outtake.deblocat();
                    deschis = true;
                    target_atins = true;
                }

                else if(distanta < 120 && abs(error) <= 5.4) {
                    outtake.deblocat();
                    deschis = true;
                    target_atins = true;
                }
            }

            if(target_atins && delta_velocity < -50 && distanta > 125) {
                outtake.reglare();
            }

            if(deschis && timer.seconds() > 0.25 && distanta < 120) {
                intake.trage_intake(1);
                if(velocityX > 0.7 || velocityY > 0.7) {
                    intake.trage_transfer(0.77);
                }
                else intake.trage_transfer(1);
            }

            else if(deschis && distanta >= 120 && timer.seconds() > 0.25) {
                intake.trage_transfer(nominalvoltage / voltage);
                intake.trage_intake(1);
            }

            if(gamepad1.triangle) {
                pinpoint.recalibration();
            }

            if(gamepad1.right_bumper) {
                decalation += 0.3;
            }

            else if(gamepad1.left_bumper) {
                decalation -= 0.3;
            }

            if(intake.a_intake() > 1 && intake.a_transfer() > 1.3 && !intake.sensor()) {
                if(!intaking_timer) {
                    timer_intake.reset();
                    intaking_timer = true;
                }

                else if(timer_intake.seconds() > 0.25) {
                    intake.stop_intake();
                    intake.stop_transfer();
                    intaking = false;
                    intaking_timer = false;
                }
            }
            else intaking_timer = false;




//            telemetry.addData("V", velocity);
//            telemetry.addData("T", target_velocity);
//            telemetry.addData("TT", final_target);
//            telemetry.addData("Delta velocity", delta_velocity);
//            telemetry.addData("Decalare", decalation);
//            telemetry.addData("STATE", mode);
//            telemetry.addData("Gra", turret.gra());
//            telemetry.addData("Eroare", turret.getError());
//            telemetry.addData("Loop time", loop.milliseconds());
//            telemetry.addData("Gra", turret.gra());
//            telemetry.addData("CurrentHeading", currentHeading);
//            telemetry.addData("Turret current pose", turretCurrentPos);
            telemetry.addData("Transfer", intake.a_transfer());
            telemetry.addData("Intake", intake.a_intake());
////            telemetry.addData("Sensor intake", intake.s_intake());
////            telemetry.addData("Sensor mij", intake.s_mij());
//            telemetry.addData("Intaking_timer", intaking_timer);
//            telemetry.addData("Timer", timer_intake.seconds());
//            if(gamepad1.dpad)
//            telemetry.addData("Distance", distanta);
//            telemetry.addData("VelocityX", velocityX);
//            telemetry.addData("VelocityY", velocityY);
//            telemetry.addData("Heading", currentHeading);
//            telemetry.addData("Tureta", turretCurrentPos);
//            telemetry.addData("Timp", timp_aer);

            telemetry.update();
        }
    }
}
