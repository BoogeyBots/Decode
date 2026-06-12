package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.act_outtake;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.auto;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.target_velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.velocity;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.outtake.voltage;
import static org.firstinspires.ftc.teamcode.Meeturi.Module.Constants.pinpoint.distanta;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
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
public class auto_far_blue extends OpMode {
    IntakeModule intake;
    OuttakeModule outtake;
    TurretModule turret;
    boolean target_atins = false;
    private Follower follower;
    double m;
    private Timer pathTimer;
    private int pathState;
    public static double cat_trage = 0.4;
    /*   public static double x_startPose = 118.651, y_startPose = 127.826, heading_startPose = 225;
       public static double x_preload = 86, y_preload = 83, heading_preload = 225;
       public static double x_collect1 = 127.5, y_collect1 = 84, heading_collect = 180;
       public static double x_trapa = 126, y_trapa = 70, heading_trapa = 110;
       public static double x_collect2 = 125, y_collect2 = 57;
       public static double x_collect3 = 126, y_collect3 = 35;
       public static double x_cp2 = 70.4, y_cp2 = 59;
       public static double x_cp3 = 82, y_cp3 = 29;
       public static double x_cptrapa = 109, y_cptrapa = 66;
       public static double x_afara = 115, y_afara = 83;
       public static double x_colectare_gate = 10, y_colectare_gate = 61.2, heading_colectare_gate = -32; //-32 */
    private final Pose startPose = new Pose(54.908, 10.634, Math.toRadians(90));
    private final Pose scorePose = new Pose(54.53, 13.46, Math.toRadians(30));
  //  private final Pose collect1 = new Pose(31.5, 84.429, Math.toRadians(0));
  //  private final Pose cp_collect1 = new Pose(69.227, 81.442);
  //  private final Pose collect2 = new Pose(19.5, 59, Math.toRadians(0));
   // private final Pose cp_collect2 = new Pose(43.82, 60.48);
     private final Pose collect3 = new Pose(20.34, 35.52, Math.toRadians(0));
     private final Pose cp_collect3 = new Pose(66.31, 50.04);
     private final Pose hp = new Pose(9.5, 14.5, Math.toRadians(15));
     private final Pose hp1pe2 = new Pose(18.29, 10.66, Math.toRadians(0));
     private final Pose hpiar = new Pose(10.83, 8.86, Math.toRadians(5));
     private final Pose shrek = new Pose(9.79, 37.17, Math.toRadians(-90));
  //  private final Pose colectare_gate = new Pose(8.2, 59, Math.toRadians(-30));
  //  private final Pose colectare_gate2 = new Pose(8.2, 59, Math.toRadians(-32.5));
   // private final Pose colectare_gate3 = new Pose(8.2, 58.5, Math.toRadians(-33));
   // private final Pose cp_gate = new Pose(34.75, 57.47);
    //private final Pose cp_trage_gate = new Pose(44.40, 68.77);
    //private final Pose cp_rand2 = new Pose(x_cp2, y_cp2).mirror();
    //private final Pose cp_rand3 = new Pose(x_cp3, y_cp3).mirror();
    //private final Pose cp_trapa = new Pose(x_cptrapa, y_cptrapa).mirror();
    //private final Pose afara = new Pose(x_afara, y_afara).mirror();
    //private final Pose colectare_gate = new Pose(x_colectare_gate, y_colectare_gate, Math.toRadians(heading_colectare_gate));
    //private final Pose cp_gate = new Pose(40, 60);
    private final Pose parcare = new Pose(39.43, 12.89);
    private Path scorePreload;
    private PathChain human, humanintraiar, human2, human3, human4, fanta, fanta2, fanta21, fanta3, fanta31, fanta4, fanta41, trage_primulshrek, trage_aldoileashrek, trage_altreileashrek, trage_alpatruleashrek, trageprimuhp, rand1, trage1, spretrapa, rand2, trage2, rand3, trage3, sprecolt, trage4, sprecolt2, trage5, leave, spretrapa2, gate, gate2, gate3, trage_gate;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setTangentHeadingInterpolation();

        rand3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_collect3, collect3))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .setNoDeceleration()
                .build();

        trage3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3, scorePose))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(20))
                .build();

        human = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, hp))
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(15))
                .build();
        fanta = follower.pathBuilder()
                .addPath(new BezierLine(hp, hp1pe2))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        humanintraiar = follower.pathBuilder()
                .addPath(new BezierLine(hp1pe2, hpiar))
                .setConstantHeadingInterpolation(Math.toRadians(5))
                .build();

        trageprimuhp= follower.pathBuilder()
                .addPath(new BezierLine(hpiar, scorePose))
                .setTangentHeadingInterpolation()
                .build();


        fanta2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, hp))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        fanta21 = follower.pathBuilder()
                .addPath(new BezierLine(hp, shrek))
                .setConstantHeadingInterpolation(Math.toRadians(-80))
                .setNoDeceleration()
                .build();


        trage_primulshrek = follower.pathBuilder()
                .addPath(new BezierLine(shrek, scorePose))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(20))
                .build();

        fanta3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, hp))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .setReversed()
                .build();

        fanta31 = follower.pathBuilder()
                .addPath(new BezierLine(hp,shrek))
                .setConstantHeadingInterpolation(Math.toRadians(-80))
                .setNoDeceleration()
                .build();

        trage_aldoileashrek = follower.pathBuilder()
                .addPath(new BezierLine(shrek, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        fanta4 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, hp))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .setReversed()
                .build();

        fanta41 = follower.pathBuilder()
                .addPath(new BezierLine(hp, shrek))
                .setConstantHeadingInterpolation(Math.toRadians(-80))
                .setNoDeceleration()
                .build();

        trage_altreileashrek = follower.pathBuilder()
                .addPath(new BezierLine(shrek, scorePose))
                .setTangentHeadingInterpolation()
                .build();


        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parcare))
                .setTangentHeadingInterpolation()
                .build();


       /* spretrapa = follower.pathBuilder()
                .addPath(new BezierCurve(collect2, cp_trapa, trapa))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .build();


        rand3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, cp_rand3, collect3))
                .setConstantHeadingInterpolation(collect1.getHeading())
                .setNoDeceleration()
                .build();

        trage3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3, scorePose))
                .setTangentHeadingInterpolation()
                .build();







        */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                act_outtake = true;
                setPathState(1);

                break;

            case 1:
                if(target_velocity < velocity + 5){
                    outtake.deblocat();

                    setPathState(2);
                }
                break;

            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 0.15){
                    trage();
                    setPathState(3);
                }
                break;

            case 3:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage){
                    numaitrag();
                    intake.trage_transfer(0.5);
                    intake.trage_intake(1);
                    follower.followPath(rand3, true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    follower.followPath(trage3);
                    setPathState(5);
                }
                break;

            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 0.7){
                    deblocare();
                    setPathState(6);
                }
                break;

            case 6:
                if (follower.atPose(scorePose, 5, 5)){
                    trage();
                    setPathState(7);
                }
                break;

            case 7:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage){
                    numaitrag();
                    intake.trage_transfer(0.5);
                    intake.trage_intake(1);
                    follower.followPath(human, false);
                    setPathState(8);
                }
                break;

            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(fanta, true);
                    setPathState(9);
                }
                break;


            case 9:
                if(!follower.isBusy()){
                    follower.followPath(humanintraiar, false);
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 0.7){
                    follower.followPath(trageprimuhp);
                    setPathState(11);
                }
                break;

            case 11:

                if(follower.atPose(scorePose, 7, 7)){
                    deblocare();
                    setPathState(12);
                }
                break;

            case 12:

                if(follower.atPose(scorePose, 5, 5)){
                    trage();
                    setPathState(13);
                }
                break;

            case 13:
                if(pathTimer.getElapsedTimeSeconds() > cat_trage){
                    numaitrag();
                    trage();
                    follower.followPath(fanta2, false);
                    setPathState(14);
                }
                break;


            case 14:
                if(pathTimer.getElapsedTimeSeconds() > 0.6){
                    follower.followPath(fanta21);
                    setPathState(15);
                }
                break;


            case 15:
                if(pathTimer.getElapsedTimeSeconds() > 0.6){
                    follower.followPath(trage_primulshrek);
                    setPathState(16);
                }
                break;

            case 16:
                if(pathTimer.getElapsedTimeSeconds() > 0.7){
                    deblocare();
                    setPathState(17);
                }
                break;

            case 17:
                if (follower.atPose(scorePose, 5, 5)){
                    trage();
                    setPathState(18);
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

        intake.init();
        outtake.init();
        turret.init();

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

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();


        telemetry.addData("path state", pathState);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", h);
        telemetry.addData("Distanta", distanta);
        telemetry.addData("pathTimer", pathTimer.getElapsedTime());
        telemetry.update();

        outtake.update_kinematics();
        turret.update_blue_auto(x, y, h);

        if(target_atins && velocity - target_velocity < -50)
            outtake.reglare_departe();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
        target_velocity = 0;
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

    public void trage() {
        intake.trage_intake(1);
        intake.trage_transfer(1);
        target_atins = true;
    }

    public void numaitrag() {
        outtake.blocat();
        target_velocity = 1500;
        intake.trage_transfer(0.47);
        target_atins = false;
    }

    public void deblocare() {
        act_outtake = true;
        intake.stop_intake();
        intake.stop_transfer();
        outtake.deblocat();
    }
}