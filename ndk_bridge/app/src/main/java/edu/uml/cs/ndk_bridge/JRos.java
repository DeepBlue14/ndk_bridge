/**
 * File: JRos.java
 * Author: James T. Kuczynski <jkuczyns@cs.uml.edu>
 * File Description: This class starts the ROS node on a background thread.  Since the overridden
 *                   run() method given over control of the thread to ROS (implicitly), it serves
 *                   only as a stage for launching the ROS node from Java.
 *
 * Created: 01/03/2017
 * Last Modified: 01/17/2017
 */

package edu.uml.cs.danrosjcxxprac;

import android.util.Log;

public class JRos implements Runnable
{
    private Thread m_thread;

    //load C/C++ shared library
    //TODO: load this from inside the constructor?
    /*static
    {
        System.loadLibrary("sample_app");
    }*/


    /**
     * Constructor.  Instantiates the thread that the ROS node will run on.
     * @param rosNodeName FIXME: this parameter is useless; the node name is hard-coded above
     */
    public JRos(String rosNodeName)
    {
        UniversalDat.loadRosNode(rosNodeName);

        m_thread = new Thread(this);
    }


    /**
     * Starts the ros node.
     */
    public void rosrun()
    {
        m_thread.start();
    }


    /**
     * Calls the ROS node's main function.  (This function will not return until the ROS node
     * exits).  Therefore, DO NOT place any code after the startRosNode() call.
     */
    @Override
    public void run()
    {
        startRosNode();
    }


    /**
     * This function calls the ros node's main() function.  It will NOT return until the ros node
     * dies.
     */
    private native void startRosNode();
}
