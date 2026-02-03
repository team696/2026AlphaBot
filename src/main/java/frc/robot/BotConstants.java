// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;


/** Add your docs here. */
public class BotConstants {
    public static CANBus riobus;
    public static CANBus Canivore;
    public static TalonFXConfiguration cfg = new TalonFXConfiguration();
    
    static{
        riobus = new CANBus("rio");
        // Canivore = new CANBus("cv"); no canivore
    }

//All these values are temporary.
    public static class Intake{

        public static final int intakeID = 13;

        static{
            //Tis where the config will go, too lazy to write it rn
        }
    }

    public static class Hopper{
        public static final int HopperID = 3;
        public static final int MagazineID = 3;

        static{
            //Tis where the config will go, too lazy to write it rn
        }

        public static Object get() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'get'");
        }

        public Command fahhh() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'fahhh'");
        }
    }

    public static class Shooter{
        public static final int shooterflywheel_1_ID = 14;
        static{
            //Tis where the config will go, too lazy to write it rn
        }

    }

    public static class Hood{
        public static final  int Hood_ID = 6;

        static{
            //Tis where the config will go, too lazy to write it rn
        }
    }

    public static class Turret{
        public static final int Turret_ID = 7;
        public static final int Turret_BeamBreakID = 1;

        static{
            //Tis where the config will go, too lazy to write it rn
        }
    }

    public static class Climber{
        public static final int Climber_1_ID = 8;
        public static final int Climber_2_ID = 9;

        static{
            //Tis where the config will go, too lazy to write it rn
        }
    }

    

}
