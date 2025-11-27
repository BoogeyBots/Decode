package org.firstinspires.ftc.teamcode.Meeturi.Teste;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@TeleOp
public class Test_localizare extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;
    Limelight3A camera;
    @Override
    public void runOpMode() throws InterruptedException {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        camera = hardwareMap.get(Limelight3A.class, "camera");
        camera.pipelineSwitch(0);

        pinpoint.setOffsets(-130.0, -11.5, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        waitForStart();
        camera.start();

        while (opModeIsActive()) {
            pinpoint.update();

            double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

            camera.updateRobotOrientation(currentHeading);
            LLResult result = camera.getLatestResult();
            if(result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();

                telemetry.addLine("Camera");
                telemetry.addData("X", botpose.getPosition().x);
                telemetry.addData("Y", botpose.getPosition().y);
                telemetry.addData("A", botpose.getOrientation().getYaw());
            }


            telemetry.addLine("Pinpoint");
            telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("A", pinpoint.getHeading(AngleUnit.DEGREES));

            telemetry.update();



        }

    }
}
