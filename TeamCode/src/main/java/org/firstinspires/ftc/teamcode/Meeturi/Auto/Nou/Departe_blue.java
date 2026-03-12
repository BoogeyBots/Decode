package org.firstinspires.ftc.teamcode.Meeturi.Auto.Nou;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.act_outtake;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.auto;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.nominalvoltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.voltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.act_turret;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.error;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.turretCurrentPos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Meeturi.Module.IntakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.OuttakeModule;
import org.firstinspires.ftc.teamcode.Meeturi.Module.TurretModule;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous (group = "Blue")
public class Departe_blue extends OpMode {
    double delta_velocity;
    boolean target_atins = false;
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static double cat_trage = 0.7;
    public static double x_startPose = 49.87, y_startPose = 8.62, heading_startPose = 0;
    public static double x_colectare1 = 6.5, y_colectare1 = 11.5, heading_c1 = 27;
    public static double x_colectare11 = 6, y_colectare11 = 10.2, heading_c11 = 4;
    public static double x_rand3 = 12, y_rand3 = 36;
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_startPose));
    private final Pose colectare1 = new Pose(x_colectare1, y_colectare1, Math.toRadians(heading_c1));
    private final Pose colectare11 = new Pose(x_colectare11, y_colectare11, Math.toRadians(heading_c11));
    private final Pose collect3 = new Pose(x_rand3, y_rand3, Math.toRadians(heading_startPose));
    private final Pose cp_rand3 = new Pose(60, 40);


    private PathChain cycle1, cycle11, trage1, rand3, trage3;

    public void buildPaths() {
         cycle1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, colectare1))
                .setLinearHeadingInterpolation(startPose.getHeading(), colectare1.getHeading())
                .build();

         cycle11 = follower.pathBuilder()
                 .addPath(new BezierLine(colectare1, colectare11))
                 .setConstantHeadingInterpolation(colectare11.getHeading())
                 .build();

         trage1 = follower.pathBuilder()
                 .addPath(new BezierLine(colectare11, startPose))
                 .setTangentHeadingInterpolation()
                 .build();

         rand3 = follower.pathBuilder()
                 .addPath(new BezierCurve(startPose, cp_rand3, collect3))
                 .setConstantHeadingInterpolation(startPose.getHeading())
                 .setNoDeceleration()
                 .build();

         trage3 = follower.pathBuilder()
                 .addPath(new BezierCurve(collect3, cp_rand3, startPose))
                 .setConstantHeadingInterpolation(startPose.getHeading())
                 .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                act_outtake = true;
                act_turret = true;
                if (target_velocity < velocity + 5 && error <= 3 && act_outtake) {
                    outtake.deblocat();
                    setPathState(1);
                    target_atins = true;
                }

                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    intake.trage_intake(1);
                    intake.trage_transfer(nominalvoltage / voltage);
                    setPathState(2);
                }

                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(3);
                }

                break;

            case 3:
                follower.followPath(cycle1, true);
                setPathState(4);

                break;

            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(cycle11, true);
                    setPathState(5);
                }

                break;

            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(trage1, true);
                    intake.stop_intake();
                    intake.stop_transfer();
                    act_outtake = true;
                    outtake.rampa(0.72);
                    setPathState(6);
                }

                break;

            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 0.2) outtake.deblocat();
                if (!follower.isBusy() && target_velocity < velocity + 5 && error <= 3 && act_outtake) {
                    setPathState(7);
                    target_atins = true;
                }

                break;

            case 7:
                intake.trage_intake(1);
                intake.trage_transfer(nominalvoltage / voltage);
                setPathState(8);

                break;

            case 8:
                if (pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(9);
                }

                break;

            case 9:
                follower.followPath(rand3, true);
                setPathState(10);

                break;

            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(trage3, true);
                    setPathState(11);
                }

                break;

            case 11:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.stop_intake();
                    intake.stop_transfer();
                }

                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
                    outtake.deblocat();
                    setPathState(12);
                }

                break;

            case 12:


        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        turret = new TurretModule(hardwareMap);

        intake.init();
        outtake.init_auto_aproape();
        turret.init_auto();

        buildPaths();
        follower.setStartingPose(startPose);

        auto = true;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getPose().getHeading());
        distanta = Math.sqrt((0 - x) * (0 - x) + (144 - y) * (144 - y));
        turret.update_auto_blue(x, y, h);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        delta_velocity = velocity - target_velocity - 1;

        if(target_atins && delta_velocity < - 50) {
            outtake.reglare();
        }

        if(intake.get_pintake() == 0 && intake.get_ptransfer() == 0) outtake.deblocat();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", h);
//        telemetry.addData("Distanta", distanta);
        telemetry.addData("Eroare", turret.getError());
        telemetry.update();

        outtake.update_kinematics();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
        target_velocity = 1400;
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void numaitrag() {
        outtake.blocat();
        target_velocity = 1400;
        intake.trage_transfer(0.47);
        target_atins = false;
        act_outtake = false;
    }

    public void deblocare_automata() {
        intake.stop_transfer();
        intake.stop_intake();

    }
}
