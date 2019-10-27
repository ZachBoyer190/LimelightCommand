package frc.robot.models;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class PairedTalonSRX extends WPI_TalonSRX {

    private final WPI_TalonSRX follower;

    public PairedTalonSRX(int leaderDeviceNumber, int followerDeviceNumber) {
        super(leaderDeviceNumber);
        configFactoryDefault();

        follower = new WPI_TalonSRX(followerDeviceNumber);
        follower.configFactoryDefault();
        follower.follow(this);
        follower.setInverted(InvertType.FollowMaster);
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        super.setNeutralMode(neutralMode);
        follower.setNeutralMode(neutralMode);
    }

    public void configPIDF(int slotIdx, double P, double I, double D, double F) {
        config_kP(slotIdx, P);
        config_kI(slotIdx, I);
        config_kD(slotIdx, D);
        config_kF(slotIdx, F);
    }

    public WPI_TalonSRX getFollower() {
        return follower;
    }

}