/**
 * File: JCxxComm.java
 * Author: James T. Kuczynski <jkuczyns@cs.uml.edu>
 * File Description: This class allows communication between the UI and the ROS node (thread).
 *                   This thread (Java) is initiating the communication by requesting data from
 *                   the UI and then sending it to the ROS node.
 *
 * Created: 01/10/2017
 * Last Modified: 01/17/2017
 */

package edu.uml.cs.danrosjcxxprac;

import android.annotation.TargetApi;
import android.os.Build;
import android.os.Handler;
import android.widget.EditText;
import android.widget.TextView;

import java.util.Objects;

public class JCxxComm implements Runnable
{
    private Thread m_thread;
    private TextView terminal; /// UI element displays data received from the ROS node
    private EditText userInputUi; /// UI element allows user to specify/change data to publish
    private String userText; /// The data to publish with ROS
    private Handler handler; /// Handler to run code on the UI thread


    JCxxComm(String rosNodeName, TextView terminal, EditText userInputUi)
    {
        UniversalDat.loadRosNode(rosNodeName);

        m_thread = new Thread(this);
        this.terminal = terminal;
        this.userInputUi = userInputUi;
        this.userText = new String("default message");
        handler = new Handler();
    }


    /**
     * Poll the UI element for new text to publish over ROS.  Else publish the old message.
     */
    @TargetApi(Build.VERSION_CODES.KITKAT)
    @Override
    public void run()
    {
        init(); // \pass instance of this class to C++ so it can call Java methods of this class.

        for(int i = 0; i < 50; i++)
        {
            try
            {
                Thread.sleep(1000);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }

            String tmp = userInputUi.getText().toString();
            if(!Objects.equals(userText, tmp))
            {
                userText = tmp;
                setMsgToPub(userText); // Update msg for ROS thread to publish to the rosmaster
            }
        }
    }


    /**
     * Runs procedures on the UI thread.
     * @param runnable
     */
    private void runOnUiThread(Runnable runnable)
    {
        handler.post(runnable);
    }


    /**
     *
     * @param msg
     */
    public void updateUi(final String msg)
    {
        runOnUiThread(new Runnable()
        {
            @Override
            public void run()
            {
                terminal.append("\n" + msg);
            }
        });
    }


    /**
     * Starts a thread, and runs "this" instance on it.
     */
    public void start()
    {
        m_thread.start();
    }



    public native void setMsgToPub(String str);

    private native void init();


} // END class CamClient