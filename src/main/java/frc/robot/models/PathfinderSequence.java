/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.models;

import java.io.IOException;

import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

/**
 * Add your docs here.
 */
public enum PathfinderSequence {
    HAB1_To_Close_Rocket;

    public Trajectory getLeft() {
        try {
            return PathfinderFRC.getTrajectory(name() + ".right");
        } catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    public Trajectory getRight(){
        try {
            return PathfinderFRC.getTrajectory(name() + ".left");
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        return null;
    }


}
