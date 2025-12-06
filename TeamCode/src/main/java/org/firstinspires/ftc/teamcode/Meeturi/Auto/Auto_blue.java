package org.firstinspires.ftc.teamcode.Meeturi.Auto;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.activated;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.turret.error;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
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
@Autonomous
public class Auto_blue extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    static TelemetryManager panelsTelemetry;
    boolean turretActivated = false;
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static double cat_trage = 1.7; //în secunde
    public static double x_startPose = 118.651, y_startPose = 127.826, heading_startPose = 225;
    public static double x_preload = 80, y_preload = 83, heading_preload = 315;
    public static double x_collect1 = 123, y_collect1 = 83, heading_collect = 270;
    public static double x_trapa = 126, y_trapa = 75, heading_trapa = 0;
    public static double x_collect2 = 129, y_collect2 = 59;
    public static double x_collect3 = 129, y_collect3 = 35;
    public static double x_cp2 = 70.4, y_cp2 = 59;
    public static double x_cp3 = 82, y_cp3 = 29;
    private final Pose startPose = new Pose(x_startPose, y_startPose, Math.toRadians(heading_startPose)).mirror();
    private final Pose scorePose = new Pose(x_preload, y_preload, Math.toRadians(heading_preload)).mirror();
    private final Pose collect1 = new Pose(x_collect1, y_collect1, Math.toRadians(heading_collect)).mirror();
    private final Pose collect2 = new Pose(x_collect2, y_collect2, Math.toRadians(heading_collect)).mirror();
    private final Pose collect3 = new Pose(x_collect3, y_collect3, Math.toRadians(heading_collect)).mirror();
    private final Pose trapa = new Pose(x_trapa, y_trapa, Math.toRadians(heading_trapa)).mirror();
    private final Pose cp_rand2 = new Pose(x_cp2, y_cp2).mirror();
    private final Pose cp_rand3 = new Pose(x_cp3, y_cp3).mirror();

    private Path scorePreload;
    private PathChain rand1, trage1, spretrapa, rand2, trage2, rand3, trage3;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        rand1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, collect1))
                .setConstantHeadingInterpolation(Math.toRadians(heading_collect))
                .build();

        spretrapa = follower.pathBuilder()
                .addPath(new BezierLine(collect1, trapa))
                .setLinearHeadingInterpolation(Math.toRadians(heading_collect), Math.toRadians(heading_trapa))
                .build();

        trage1 = follower.pathBuilder()
                .addPath(new BezierLine(trapa, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(heading_trapa), Math.toRadians(heading_startPose)) //probabil Linear, nush, vedem
                .build();

        rand2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_rand2, collect2))
                .setConstantHeadingInterpolation(Math.toRadians(heading_collect))
                .build();

        trage2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(heading_collect), Math.toRadians(heading_startPose))
                .build();

        rand3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_rand3, collect3))
                .setConstantHeadingInterpolation(Math.toRadians(heading_collect))
                .build();

        trage3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3, scorePose))
                .setLinearHeadingInterpolation(Math.toRadians(heading_collect), Math.toRadians(heading_startPose))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1000);

                break;

            case 1:
                intake.sus();
                intake.trage(0.1);
                activated = true;
                setPathState(2);

                break;

            case 2:
                if(follower.atPose(scorePose, 2, 2)) {
                    if (target_velocity <= velocity - 20 && error <= 3 && activated) {
                        outtake.deblocat();
                        setPathState(3);
                    }
                }

                break;

            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.trage(1);
                    setPathState(4);
                }

                break;

            case 4:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(5);
                }

                break;


            case 5:
                follower.followPath(rand1, true);
                setPathState(6);

                break;

            case 6:
                if(follower.atPose(collect1, 1, 1)) {
                    follower.followPath(spretrapa, true);
                    intake.trage(0.1);
                    setPathState(7);
                }

                break;

            case 7:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2) {
                        follower.followPath(trage1, true);
                        setPathState(8);
                    }
                }

                break;

            case 8:
                if(follower.atPose(scorePose, 1,1)) {
                    activated = true;
                    intake.jos();
                    intake.scuipa(0.4);
                    setPathState(9);
                }

                break;

            case 9:
                if(target_velocity <= velocity - 20 && error <= 3 && activated) {
                    outtake.deblocat();
                    setPathState(10);
                }

                break;

            case 10:
                if(pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intake.sus();
                    intake.trage(1);
                    setPathState(11);
                }

                break;

            case 11:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(12);
                }

                break;

            case 12:
                follower.followPath(rand2, true);
                setPathState(13);

                break;

            case 13:
                if(!follower.isBusy()) {
                    intake.trage(0.1);
                    setPathState(14);
                }

                break;

            case 14:
                follower.followPath(trage2);
                setPathState(15);

                break;

            case 15:
                if(follower.atPose(scorePose, 1,1)) {
                    activated = true;
                    intake.jos();
                    intake.scuipa(0.4);
                    setPathState(16);
                }

                break;

            case 16:
                if(target_velocity <= velocity - 20 && error <= 3 && activated) {
                    outtake.deblocat();
                    setPathState(17);
                }

                break;

            case 17:
                if(pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intake.sus();
                    intake.trage(1);
                    setPathState(69);
                }

                break;

            case 69:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage) {
                    numaitrag();
                    setPathState(18);
                }

                break;

            case 18:
                follower.followPath(rand3, true);
                setPathState(19);

                break;

            case 19:
                if(!follower.isBusy()) {
                    intake.trage(0.1);
                    setPathState(20);
                }

                break;

            case 20:
                follower.followPath(trage3);
                setPathState(21);

                break;

            case 21:
                if(follower.atPose(scorePose, 1,1)) {
                    activated = true;
                    intake.jos();
                    intake.scuipa(0.4);
                    setPathState(22);
                }

                break;

            case 22:
                if(target_velocity <= velocity - 20 && error <= 3 && activated) {
                    outtake.deblocat();
                    setPathState(23);
                }

                break;

            case 23:
                if(pathTimer.getElapsedTimeSeconds() > 0.2) {
                    intake.trage(1);
                    setPathState(24);
                }

                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        intake = new IntakeModule(hardwareMap);
        outtake = new OuttakeModule(hardwareMap);
        turret = new TurretModule(hardwareMap);

        intake.init_auto();
        outtake.init_auto_aproape();
        turret.init_auto();

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        outtake.update();
        autonomousPathUpdate();

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double h = Math.toDegrees(follower.getPose().getHeading());
        distanta = Math.sqrt((0 - x) * (0 - x) + (144 - y) * (144 - y));
        turret.update_auto_blue(x, y, h);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", h);
        telemetry.addData("Distanta", distanta);
        telemetry.addData("error", turret.getErrore());
        telemetry.addData("power", turret.getPower());
        telemetry.addData("kp", turret.getkP());
        telemetry.addData("Grade", turret.gra());
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
        target_velocity = 0;
    }

    @Override
    public void init_loop() {
        //turret.update_auto(Math.toDegrees(follower.getPose().getHeading()));
    }

    @Override
    public void stop() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void numaitrag() {
        outtake.blocat();
        target_velocity = 0;
        activated = false;
    }
}
