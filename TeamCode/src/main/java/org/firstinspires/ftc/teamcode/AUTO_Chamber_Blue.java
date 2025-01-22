package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Config
@Autonomous(name = "Auto To Corner", group = "Autonomous")
public class AUTO_Chamber_Blue extends LinearOpMode {


    HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        Pose2d initialPose = new Pose2d(0, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose)

                .strafeTo(new Vector2d(-20, -62));
                            Pose2d toSubEnd = new Pose2d(-12, -60, Math.toRadians(90));
                            // Headings: 180 = Left, 0 = Right, -90 = Down, 90=UP

        TrajectoryActionBuilder backFromSub = drive.actionBuilder(toSubEnd)

                .strafeTo(new Vector2d(0, -42));
        Pose2d backFromSubEnd = new Pose2d(0, -42, Math.toRadians(90));
        // Headings: 180 = Left, 0 = Right, -90 = Down, 90=UP

        TrajectoryActionBuilder toSamples = drive.actionBuilder(backFromSubEnd)
                //Travel Around submersible and toward submersible.
                .splineToSplineHeading(new Pose2d(-10,-30, Math.toRadians(45)), Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-29,-5, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(new Vector2d(-20, 0));

                          Pose2d toSamplesEnd = new Pose2d(30, -53, Math.toRadians(-90));

        TrajectoryActionBuilder toSub2 = drive.actionBuilder(toSamplesEnd)
                      //Move to submersible to deliver Spec1
                .waitSeconds(.15)
                .strafeTo(new Vector2d(25, -50))
                .strafeToLinearHeading(new Vector2d(0, -0.5),Math.toRadians(150))
                .waitSeconds(.25);
                         Pose2d toSub2End = new Pose2d(5, -19, Math.toRadians(180));


        TrajectoryActionBuilder toWall2 = drive.actionBuilder(toSub2End)
                //Return for Spec2
                .waitSeconds(.25)
                .strafeTo(new Vector2d(36, -50))
                .strafeToLinearHeading(new Vector2d(36, -55),Math.toRadians(-5))
                     //Press Wall for Spec2
                .strafeTo(new Vector2d(36, -65));
                         Pose2d toWall2End = new Pose2d(35, -65, Math.toRadians(-5));

        TrajectoryActionBuilder toSub3 = drive.actionBuilder(toWall2End)
                //.waitSeconds(.15)
                //Move to submersible to deliver Spec2
                .strafeTo(new Vector2d(30, -50))
                .strafeToLinearHeading(new Vector2d(0, -20),Math.toRadians(190)) ;

        Pose2d toSub3End = new Pose2d(0, -20, Math.toRadians(190));

        TrajectoryActionBuilder toWall3 = drive.actionBuilder(toSub3End)
                .waitSeconds(.25)
                //Return to the wall
                .strafeTo(new Vector2d(35, -50))
                .strafeToLinearHeading(new Vector2d(35, -55),Math.toRadians(0))
                //Pick up Spec3
                .strafeTo(new Vector2d(35, -61));
                         Pose2d toWall3End = new Pose2d(35, -61, Math.toRadians(0));

        TrajectoryActionBuilder toSub4 = drive.actionBuilder(toWall3End)
                 //Deliver Spec3
                .strafeTo(new Vector2d(30, -50))
                .strafeToLinearHeading(new Vector2d(3, -21),Math.toRadians(180));



        Action trajectoryActionCloseOut = toSub4.fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(rotate.rotateUp());




        telemetry.addData("Initialization Complete", "Awaiting Start Command");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
       // robot.servorelease.setPosition(.66);

        //liftUp (1, 30);
        //sleep  (2000);
        //extend(1, 1);
        //sleep(2000);
      //  robot.specimenClamp.setPosition(.4);


        Actions.runBlocking(
                new SequentialAction(
                        toSub.build()
                )
        );
       /* extend(1, -2);
        sleep(1000);
        robot.servorelease.setPosition(.46);
        sleep(500);
        liftUp(1, 5);

        Actions.runBlocking(
                new SequentialAction(
                        backFromSub.build()
                )
        );
        extend(1,5);
        liftUp(1,-5);
        sleep(2000);
        Actions.runBlocking(
                new SequentialAction(
                        toSamples.build()
                )
        );

*/
    }
    public void liftUp (double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;

        if (opModeIsActive()) {

           newLiftTarget = robot.liftup.getCurrentPosition() + (int) (-inches * (1140/(3.5 * 3.1415)));

           robot.liftup.setTargetPosition(newLiftTarget);

            robot.liftup.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
           robot.liftup.setPower(Math.abs(power));

           /*while (opModeIsActive() &&
                                   robot.liftup.isBusy()) {
                       telemetry.addData("Lift Up", "Running at %7d",
                                       robot.liftup.getCurrentPosition());
                    telemetry.update();

               }
          robot.liftup.setPower(0);
          robot.liftup.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

          }
     }


    public void extend (double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;

        if (opModeIsActive()) {

           newLiftTarget = -robot.liftout.getCurrentPosition() + (int) (inches * (1140/(3.5 * 3.1415)));

           robot.liftout.setTargetPosition(newLiftTarget);

            robot.liftout.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
          robot.liftout.setPower(Math.abs(power));

           /*while (opModeIsActive() &&
                                   robot.liftout.isBusy()) {
                       telemetry.addData("Lift", "Running at %7d",
                                       robot.liftout.getCurrentPosition());
                    telemetry.update();

               }
          robot.liftout.setPower(0);
          robot.liftout.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

        }
    }

}

