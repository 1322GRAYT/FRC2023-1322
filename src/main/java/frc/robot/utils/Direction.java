// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

/** Add your docs here. */
public enum Direction{
    NONE(-1), UP(0), UPRIGHT(45), RIGHT(90), DOWNRIGHT(135), DOWN(180), DOWNLEFT(225), LEFT(270), UPLEFT(315);

    int direction;
    private static Map map = new HashMap<>();

    static {
      for (Direction direction : Direction.values()){
        map.put(direction.direction, direction);
      }
    }

    private Direction(int direction){
      this.direction = direction;
    }

    public static Direction valueOf(int direction){
      return (Direction) map.get(direction);
    }

    public int getValue(){
      return direction;
    }

  };
