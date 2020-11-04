package org.firstinspires.ftc.teamcode.robot.lib;

import org.firstinspires.ftc.teamcode.robot.mapping.InterlockMap;

import java.util.HashMap;

/***
 * Interlock class to require approval from other devices for operation.
 */
public class Interlock {
    private HashMap<Object, Boolean> interlocks = new HashMap<>();

    /***
     * Constructor with no arguments
     */
    public Interlock(){
        // public interlock constructor for no arguments
    }

    /***
     * Gets whether the interlock is locked or not
     * @return true if action is allowed
     */
    public boolean isUnlocked(){
        for(boolean lock : interlocks.values()){
            if(lock == false){
                return lock;
            }
        }

        return true;
    }

    /***
     * Register an lock with the interlock
     * @param lock the lock to register
     */
    public void registerInterlock(Object lock){
        this.interlocks.put(lock, false);
    }

    /***
     * Set the value for an interlock
     * @param lockObject the object to use as the lock
     * @param lock the state of the lock, true for locked
     */
    public void setInterlock(InterlockMap.Blockers lockObject, boolean lock){
        this.interlocks.put(lockObject, lock);
    }
}
