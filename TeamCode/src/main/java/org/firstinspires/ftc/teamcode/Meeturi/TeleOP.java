package org.firstinspires.ftc.teamcode.Meeturi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Meeturi.Module.Constants;
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
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        turret = new TurretModule(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.init();
        outtake.init();
        turret.init();

        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive()) {

            //cred ca ar fi ideal sa fac o functie read pentru fiecare modul, s-o apelez o data pe loop si apoi sa ma folosesc de variabilele alea
            //atp am doar update la pinpoint, deci nu e necesar

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
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

            if(gamepad1.a) {
                outtake.departe();
            }

            if(gamepad1.b) {
                outtake.aproape();
            }

            if(gamepad1.y) {
                outtake.stop();
            }

            outtake.update();

            telemetry.addData("Target: ", Constants.outtake.target_velocity);
            telemetry.update();
        }
    }
}
