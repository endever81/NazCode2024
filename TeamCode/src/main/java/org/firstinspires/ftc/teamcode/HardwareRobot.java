package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareRobot {

    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;
    public DcMotor liftH = null;
    public DcMotor liftV = null;
    public DcMotor intake = null;

    public DcMotor hang = null;
    public Servo servorotate = null;
    public Servo servohang = null;
    public Servo servointake = null;
    public Servo specimenClamp = null;
    public CRServo rightPickup = null;
    public CRServo leftPickup = null;
    public RevBlinkinLedDriver blinkinLedDriver = null;




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
        intake = hwMap.get(DcMotor.class, "motor_intake");
        hang = hwMap.get(DcMotor.class, "hang");

        servohang = hwMap.get(Servo.class, "servo_hang");
        servorotate = hwMap.get(Servo.class, "servo_rotate");
        servointake = hwMap.get(Servo.class, "servo_intake");
        specimenClamp = hwMap.get(Servo.class, "specimenClamp");

        leftPickup = hwMap.get(CRServo.class, "servo_left_pickup");
        rightPickup = hwMap.get(CRServo.class, "servo_right_pickup");


        blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        liftH.setDirection(DcMotor.Direction.REVERSE);
        liftH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftV.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftV.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        servorotate.setPosition(.45);
        servointake.setPosition(0);
        specimenClamp.setPosition(0);
        servohang.setPosition(1);
    }

}

