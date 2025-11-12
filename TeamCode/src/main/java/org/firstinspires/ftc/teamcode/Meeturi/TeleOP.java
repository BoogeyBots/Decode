package org.firstinspires.ftc.teamcode.Meeturi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import java.util.List;
@TeleOp
public class TeleOP extends LinearOpMode {
    SampleMecanumDrive drive = null;
    IntakeModule intake = null;
    OuttakeModule outtake = null;
    TurretModule turret = null;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        turret = new TurretModule(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.init_teleOP();
        outtake.init_teleOP();
        turret.init_teleOP();

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        boolean shooter = false;

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

          //  turret.update();

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

            if(gamepad1.a) {
                outtake.departe();
                timer.reset();
                shooter = true;
            }

            if(gamepad1.b) {
                outtake.aproape();
                timer.reset();
                shooter = true;
                intake.sus();
            }

            if(timer.seconds() > 1.7 && shooter) {
                outtake.deblocat();
                intake.trage(1);
                shooter = false;
                timer.reset();
            }


            if(gamepad1.y) {
                outtake.stop();
                outtake.blocat();
                intake.stop();
            }

            //tureta manuala
            if(gamepad1.right_bumper) {
                turret.manual(1);
            }

            else if(gamepad1.left_bumper) {
                turret.manual(-1);
            }

            else {
                turret.manual(0);
            }

            outtake.update();
        }
    }
}
