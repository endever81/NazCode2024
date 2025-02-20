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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Auto Blue Left", group = "Autonomous")
public class AUTO_Blue_Left extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();
    private volatile boolean servoOverrideActive = false; // Flag to manage servo control
    private  int zeroLiftH; // Int variable shared across all instances

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        Pose2d initialPose = new Pose2d(-18, -63, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        zeroLiftH = robot.liftH.getCurrentPosition();


        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose)

                .strafeTo(new Vector2d(0, -29));
        Pose2d toSubEnd = new Pose2d(0, -33.5, Math.toRadians(180));
        // Headings: 180 = Left, 0 = Right, -90 = Down, 90=UP

        TrajectoryActionBuilder toSample1 = drive.actionBuilder(toSubEnd)
                //Travel Around submersible and toward sample 1.
                .strafeToLinearHeading(new Vector2d(-55, -72),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-62, -75),Math.toRadians(50))

                .waitSeconds(2); // move back to sample 1

               // .splineToSplineHeading(new Pose2d(-57,-105, Math.toRadians(0)), Math.toRadians(-95))
                Pose2d toSample1End = new Pose2d(-62, -75, Math.toRadians(50));

        TrajectoryActionBuilder toHG1 = drive.actionBuilder(toSample1End)
                //Move to High Goal to deliver Sample 1

                .strafeToLinearHeading(new Vector2d(0, -70),Math.toRadians(-135))
                .splineToSplineHeading(new Pose2d(-62,-92, Math.toRadians(-135)), Math.toRadians(-45))
                .waitSeconds(1);

                Pose2d toHG1End = new Pose2d(-55, -82, Math.toRadians(-135));


        TrajectoryActionBuilder toSample2 = drive.actionBuilder(toHG1End)
                //Return for Sample 2
                .strafeToLinearHeading(new Vector2d(-41, -51),Math.toRadians(90));

                Pose2d toSample2End = new Pose2d(-41, -51, Math.toRadians(90));

        TrajectoryActionBuilder toHG2 = drive.actionBuilder(toSample2End)
                //Move to High Goal to deliver Sample 2
                .strafeToLinearHeading(new Vector2d(-85, -90),Math.toRadians(-135));

                Pose2d toHG2End = new Pose2d(-85, -90, Math.toRadians(-135));

        TrajectoryActionBuilder toSample3 = drive.actionBuilder(toHG2End)
                //Return for Sample 3
                .strafeToLinearHeading(new Vector2d(-41, -51),Math.toRadians(90));

            Pose2d toSample3End = new Pose2d(-41, -51, Math.toRadians(90));

        TrajectoryActionBuilder toHG3 = drive.actionBuilder(toSample3End)
                //Move to High Goal to deliver Sample 3
                .strafeToLinearHeading(new Vector2d(-85, -90),Math.toRadians(-135));

            Pose2d toHG3End = new Pose2d(-85, -90, Math.toRadians(-135));

        TrajectoryActionBuilder toSubContact = drive.actionBuilder(toHG3End)
                //Move to contact the submersible
                .strafeToLinearHeading(new Vector2d(-35, -15),Math.toRadians(180));


        Action trajectoryActionCloseOut = toSubContact.fresh()
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(rotate.rotateUp());




        telemetry.addData("Initialization Complete", "Awaiting Start Command");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        robot.servohang.setPosition(1);
        robot.specimenClamp.setPosition(0);
        robot.servorotate.setPosition(.1);
        lift (1, 18.5); //Lift to High Chamber height
                Actions.runBlocking(
                        new SequentialAction(
                                toSub.build() // To Submersible
                        )
                );
        lift (1, -6);  //lower lift to lock specimen to bar
        sleep(750);
        robot.specimenClamp.setPosition(.4);  //release servo
        lift (.5, -14); //lower lift to the ground
                Actions.runBlocking(
                        new SequentialAction(
                                toSample1.build()  //To Sample 1
                        )
                );
        robot.servoswing.setPosition(0.48);    // SET SWING ARM POSITION
        robot.servoart.setPosition(0.54);     // Set Articulator Position
        robot.servorotate.setPosition(0.3);  //moves the claw up and down 0.3 up / 0.6 Down
        robot.servointake.setPosition(0.3); // Closes the claw  0.3 open / 0.48 closed
        sleep(500);
        robot.servorotate.setPosition(0.6);  //moves the claw up and down 0.3 up / 0.6 Down
        sleep(1000);
        robot.servointake.setPosition(0.48); // Closes the claw  0.3 open / 0.48 closed
        sleep(500);
        controlServosAsync();  //automated servo transfer routine
        sleep(1500);

        lift (1, 50); //lift to high goal

        Actions.runBlocking(
                new SequentialAction(
                        toHG1.build()
                )
        );
        robot.bucketgrab.setPosition(0.2);   //Bucket claw 0.2 open/0.58 closed
        sleep(500);
        lift (1, -50);   //drop the lift

        Actions.runBlocking(
                new SequentialAction(
                        toSample2.build()
                )
        );
        robot.servoswing.setPosition(0.48);    // SET SWING ARM POSITION
        robot.servoart.setPosition(0.54);     // Set Articulator Position
        robot.servorotate.setPosition(0.3);  //moves the claw up and down 0.3 up / 0.6 Down
        robot.servointake.setPosition(0.3); // Closes the claw  0.3 open / 0.48 closed
        sleep(500);
        robot.servorotate.setPosition(0.6);  //moves the claw up and down 0.3 up / 0.6 Down
        sleep(500);
        robot.servointake.setPosition(0.48); // Closes the claw  0.3 open / 0.48 closed
        sleep(500);
        controlServosAsync();  //automated servo transfer routine
        sleep(1500);

        lift (1, 50); //lift to high goal

        Actions.runBlocking(
                new SequentialAction(
                        toHG2.build()
                )
        );
        robot.bucketgrab.setPosition(0.2);   //Bucket claw 0.2 open/0.58 closed
        sleep(500);
        lift (1, -50);   //drop the lift

        Actions.runBlocking(
                new SequentialAction(
                        toSample3.build()
                )
        );

        robot.servoswing.setPosition(0.48);    // SET SWING ARM POSITION
        robot.servoart.setPosition(0.54);     // Set Articulator Position
        robot.servorotate.setPosition(0.3);  //moves the claw up and down 0.3 up / 0.6 Down
        robot.servointake.setPosition(0.3); // Closes the claw  0.3 open / 0.48 closed
        sleep(500);
        robot.servorotate.setPosition(0.6);  //moves the claw up and down 0.3 up / 0.6 Down
        sleep(500);
        robot.servointake.setPosition(0.48); // Closes the claw  0.3 open / 0.48 closed
        sleep(500);
        controlServosAsync();  //automated servo transfer routine
        sleep(1500);

        lift (1, 50); //lift to high goal

        Actions.runBlocking(
                new SequentialAction(
                        toHG3.build()
                )
        );
        robot.bucketgrab.setPosition(0.2);   //Bucket claw 0.2 open/0.58 closed
        sleep(500);
        lift (1, -32);   //drop the lift to 18.5

        Actions.runBlocking(
                new SequentialAction(
                        toSubContact.build()
                )
        );

    }


    public void lift(double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;
        int newLiftTarget2;



        if (opModeIsActive()) {

            newLiftTarget = robot.liftV.getCurrentPosition() + (int) (inches * (-1140/(3.5 * 3.1415))*.717);
            newLiftTarget2 = robot.liftV2.getCurrentPosition() + (int) (inches * (-1140/(3.5 * 3.1415))*.717);

            robot.liftV.setTargetPosition(newLiftTarget);
            robot.liftV2.setTargetPosition(newLiftTarget2);

            robot.liftV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftV.setPower(Math.abs(power));
            robot.liftV2.setPower(Math.abs(power));

           /*while (opModeIsActive() &&
                                   robot.liftV.isBusy()) {
                       telemetry.addData("Lift", "Running at %7d",
                                       robot.liftV.getCurrentPosition());
                    telemetry.update();

               }
          robot.liftV.setPower(0);
          robot.liftV.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

        }
    }


    public void extend(double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;

        if (opModeIsActive()) {

            newLiftTarget = robot.liftH.getCurrentPosition() + (int) (inches * (1140/(3.5 * 3.1415)));

            robot.liftH.setTargetPosition(newLiftTarget);

            robot.liftH.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftH.setPower(Math.abs(power));

           /*while (opModeIsActive() &&
                                   robot.liftV.isBusy()) {
                       telemetry.addData("Lift", "Running at %7d",
                                       robot.liftV.getCurrentPosition());
                    telemetry.update();

               }
          robot.liftV.setPower(0);
          robot.liftV.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

        }
    }
    /**
     * Asynchronous method to control servos
     */
    private void controlServosAsync() {
        new Thread(() -> {
            try {
                servoOverrideActive = true; // Mark override as active
                robot.liftH.setTargetPosition(zeroLiftH);
                robot.liftH.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftH.setPower(1);

                setServoPosition(robot.servoart, 1); //ensure head is orented appropriately
                Thread.sleep(100);

                // Move servos through a sequence
                //LowerArm right
                setServoPosition(robot.servorotate, 0.1); //flip up the Sample
                setServoPosition(robot.servointake, 0.46); //ensure low claw says closed .68
                setServoPosition(robot.servoart, 1); //ensure head is orented appropriately
                setServoPosition(robot.servoswing, 0.3); //swing to reposition sample in claw
                setServoPosition(robot.bucketgrab, 0.2); //open the high claw
                Thread.sleep(700);
                Thread.interrupted();
                setServoPosition(robot.servointake, 0.42); //loosen low claw .68
                setServoPosition(robot.servorotate, 0.2); //put sample on the ground
                Thread.sleep(250);  // Wait for movement to complete

                setServoPosition(robot.servointake, 0.46); //tightly grip the Sample .7
                setServoPosition(robot.servoswing, 0.35); //move sample back slightly
                setServoPosition(robot.servorotate, 0.29); //move sample down slightly
                Thread.sleep(250);  // Wait for movement to complete

                //LowerArm back in
                //  setServoPosition(robot.servoswing, 0.65); //swing for transfer
                // Thread.sleep(500);

                //top arm out
                //  setServoPosition(robot.bucketart, 0.6);
                // Thread.sleep(10);

                //top arm down
                setServoPosition(robot.bucketrotate, 0.8);
                Thread.sleep(250);

                //top arm grab
                setServoPosition(robot.bucketgrab, 0.58);
                Thread.sleep(250);

                //lowerarm letgo
                setServoPosition(robot.servointake, 0.1); //.5
                Thread.sleep(500);

                //lowerarm left
                setServoPosition(robot.servoart, .5); //ensure head is orented appropriately
                setServoPosition(robot.servoswing, 0.5);
                Thread.sleep(250);

                //top arm up
                setServoPosition(robot.bucketrotate, 0.4);
                Thread.sleep(250);

                //top arm in
                // setServoPosition(robot.bucketart, 0.6);
                //Thread.sleep(1000);


                //Release hLift from power
                robot.liftH.setPower(0);
                robot.liftH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);








            } catch (Exception e) {
                // Handle interrupted exception properly
                Thread.currentThread().interrupt(); // Restore interrupt flag
            } finally {
                servoOverrideActive = false; // Release control back to manual mode
            }
        }).start();
    }

    /**
     * Safe method to set servo position, ensuring override is respected.
     */
    private synchronized void setServoPosition(com.qualcomm.robotcore.hardware.Servo servo, double position) {
        if (servoOverrideActive) {
            servo.setPosition(position);
        }
    }
}

