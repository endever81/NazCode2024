package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class HardwareRobot {

    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor liftH = null;
    public DcMotor liftV = null;
    public DcMotor liftV2 = null;
   // public DcMotor intake = null;

    public DcMotor hang = null;
    public Servo servorotate = null;
    public Servo servohang = null;
    public Servo servoart = null;
    public Servo servoswing = null;

    public Servo servosweep = null;
    public Servo servointake = null;
    public Servo specimenClamp = null;
    public Servo bucketgrab = null;
    public Servo bucketrotate = null;
    public Servo bucketart = null;
    public RevBlinkinLedDriver blinkinLedDriver = null;
    public DistanceSensor sensorDistance = null;




    //public Servo grabber = null;

    HardwareMap hwMap = null;
   
    private ElapsedTime period = new ElapsedTime();

    public HardwareRobot(){
    }
   
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap; //saves a reference Hardware Map

        leftFrontDrive = hwMap.get(DcMotor.class, "motor_front_left");
        rightFrontDrive = hwMap.get(DcMotor.class, "motor_front_right");
        leftRearDrive = hwMap.get(DcMotor.class, "motor_rear_left");
        rightRearDrive = hwMap.get(DcMotor.class, "motor_rear_right");
        liftH = hwMap.get(DcMotor.class, "liftH");
        liftV = hwMap.get(DcMotor.class, "liftV");
        liftV2 = hwMap.get(DcMotor.class, "liftV2");
        hang = hwMap.get(DcMotor.class, "hang");

        servohang = hwMap.get(Servo.class, "servo_hang");
        servorotate = hwMap.get(Servo.class, "servo_rotate");
        servoart = hwMap.get(Servo.class, "servo_art");
        servoswing = hwMap.get(Servo.class, "servo_swing");
        servointake = hwMap.get(Servo.class, "servo_intake");
        specimenClamp = hwMap.get(Servo.class, "specimenClamp");
        bucketgrab = hwMap.get(Servo.class, "bucket_grab");
        bucketart = hwMap.get(Servo.class, "bucket_articulate");
        bucketrotate = hwMap.get(Servo.class, "bucket_rotate");
        servosweep = hwMap.get(Servo.class, "servo_sweep");

        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_distance");


        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        liftH.setDirection(DcMotor.Direction.REVERSE);
        liftH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftV2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftV2.setDirection(DcMotor.Direction.REVERSE);
        liftH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftV2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        bucketgrab.setPosition(.3);
        bucketart.setPosition(.25);
        bucketrotate.setPosition(.8);

        servosweep.setPosition(0);
        servorotate.setPosition(.1);
        servointake.setPosition(0.3); //.5
        servoswing.setPosition(.81);
        servoart.setPosition(.5);

        specimenClamp.setPosition(0);  //Specimeins are on the wall
        //servohang.setPosition(1);


    }

    //create a separate method to carry a variable publicly throughout the greater program
    public double getDistance() {
        return sensorDistance.getDistance(DistanceUnit.INCH); // Get distance in millimeters
    }

}

