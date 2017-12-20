package edu.uml.cs.danrosjcxxprac;

import java.util.ArrayList;

public class UniversalDat
{
    private static ArrayList<String> rosLibArr = new ArrayList<>(); //List of loaded ros nodes (as *.so files)
    private static boolean destroyRequested = false;
    private static String rosMasterUri = "http://robot-brain2:11311"; /// ROS_MASTER_URI environment variable
    private static String rosIp = "10.0.7.145"; /// ROS_IP environment variable




    /**
     * Mutator
     * @see #rosMasterUri
     *
     * @param rosMasterUri
     */
    public static void setRosMasterUri(String rosMasterUri)
    {
        UniversalDat.rosMasterUri = rosMasterUri;
    }


    /**
     * Accessor
     * @see #rosMasterUri
     *
     * @return
     */
    public static String getRosMasterUri()
    {
        return rosMasterUri;
    }


    /**
     * Mutator
     * @see #rosIp
     *
     * @param rosIp
     */
    public static void setRosIp(String rosIp)
    {
        UniversalDat.rosIp = rosIp;
    }


    /**
     * Accessor
     * @see #rosIp
     *
     * @return
     */
    public static String getRosIp()
    {
        return rosIp;
    }


    /**
     * Load ros node.
     * Check to see if the requested library (e.g. ros node) has already been loaded.
     *
     * @param roslib
     */
    public static void loadRosNode(String roslib)
    {
        boolean isAlreadyLoaded = false;

        for(String element : rosLibArr )
        {
            if(!element.equalsIgnoreCase(roslib) )
            {
                isAlreadyLoaded = true;
                break;
            }
        }

        if(!isAlreadyLoaded)
        {
            System.loadLibrary(roslib);
            rosLibArr.add(roslib);
        }
    }


    public static void setDestroyRequested(boolean destroyRequested)
    {
        UniversalDat.destroyRequested = destroyRequested;
    }


    public static boolean destroyRequested()
    {
        return destroyRequested;
    }


    /**
     * Prints out the current values of the class member variables.
     *
     * @return
     */
    public String toString()
    {
        String tmp = "----------------UniversalDat.toString()----------------" +
                "\nROS_MASTER_URI: " + getRosMasterUri() +
                "\nROS_IP: " + getRosIp() +
                "\n-------------------------------------------------------";
        return tmp;
    }

}// End of class UniversalDat