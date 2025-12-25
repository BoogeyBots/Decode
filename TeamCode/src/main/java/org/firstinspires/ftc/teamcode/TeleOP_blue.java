package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.act_outtake;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.auto;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.final_target;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.ramp;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.voltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.velocityX;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.velocityY;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.act_turret;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.decalation;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.timp_aer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Meeturi.Module.Constants;
import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.PinpointModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import java.util.List;
@TeleOp (name = "Tiliopa blue")
public class TeleOP_blue extends LinearOpMode {

    DistanceSensor sensor;
    SampleMecanumDrive drive = null;
    IntakeModule intake = null;
    OuttakeModule outtake = null;
    TurretModule turret = null;
    PinpointModule pinpoint;
    ElapsedTime timer;
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

        act_outtake = false;
        act_turret = false;
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

            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

            outtake.update_kinematics();
            pinpoint.update_blue();
            turret.update_kinematics_blue();

            delta_velocity = velocity - target_velocity - 1;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
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
                if(velocityX > 0.7 || velocityY > 0.7) {
                    intake.trage_transfer(0.77);
                }
                else if(distanta > 120) {
                    intake.trage_transfer(0.87);
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
                act_turret = true;
                Constants.outtake.act_outtake = true;
                switchingState = true;
                timer.reset();
            }

            if(gamepad1.a && mode == STATE.numaitrage) {
                Constants.outtake.target_velocity = 990;
                deschis = false;
                Constants.outtake.act_outtake = false;
                act_turret = false;
                switchingState = true;
                outtake.blocat();
                timer.reset();
            }


            if (switchingState && timer.seconds() > 0.15) {
                mode = (mode == STATE.trage) ? STATE.numaitrage : STATE.trage;
                switchingState = false;
            }

            if(delta_velocity > 0 && Constants.turret.error <= 3 && Constants.outtake.act_outtake && timer.seconds() > 0.107) {
                outtake.deblocat();
                deschis = true;
            }

            if(gamepad1.triangle) {
                pinpoint.recalibration();
            }

            if(target_velocity != 1000 && act_outtake && delta_velocity < -150 && deschis) {
                outtake.reglare();
            }

            if(gamepad1.right_bumper) {
                decalation += 0.3;
            }

            else if(gamepad1.left_bumper) {
                decalation -= 0.3;
            }

            telemetry.addData("V", velocity);
            telemetry.addData("T", target_velocity);
            telemetry.addData("TT", final_target);
            telemetry.addData("Delta velocity", delta_velocity);
            telemetry.addData("Decalare", decalation);
            telemetry.addData("STATE", mode);
            telemetry.addData("Gra", turret.gra());
            telemetry.addData("Distance", distanta);
            telemetry.addData("VelocityX", velocityX);
            telemetry.addData("VelocityY", velocityY);
            telemetry.addData("Timp", timp_aer);
            telemetry.update();
        }
    }
}
