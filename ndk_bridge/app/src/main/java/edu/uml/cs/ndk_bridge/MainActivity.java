/**
 * File:   MainActivity.java
 * Author: James T. Kuczynski <jkuczyns@cs.uml.edu>
 * File Description: main activity of this project.
 */

package edu.uml.cs.danrosjcxxprac;

import android.app.Activity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.TextView;

public class MainActivity extends Activity
{
    private Menu menu;
    private NetworkConfig networkConfig = new NetworkConfig();


    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        // Start ROS node on separate thread.  ROS will update the UI whenever it recieves new
        // data (in its callback function)
        JRos sampleApp = new JRos("chatter_bot");
        sampleApp.rosrun();

        // Start (Java) thread to send data from the user to the ROS node.
        JCxxComm jCxxComm = new JCxxComm("chatter_bot", (TextView) findViewById(R.id.textView), (EditText) findViewById(R.id.editText) );
        jCxxComm.start();
    }


    @Override
    public boolean onCreateOptionsMenu(Menu menu)
    {
        getMenuInflater().inflate(R.menu.menu_main, menu);
        this.menu = menu;

        return super.onCreateOptionsMenu(menu);
    }


    public void onRadioButtonClicked(View view)
    {
        boolean checked = ((RadioButton) view).isChecked();

        switch(view.getId() )
        {
            case R.id.radioButton_auto:
                if(checked)
                {
                    //use auto
                }
                break;
            case R.id.radioButton_manual:
                if(checked)
                {
                    //use manual mode
                }
                break;
        }
    }


    @Override
    public boolean onOptionsItemSelected(MenuItem item)
    {
        /*switch(item.getItemId() )
        {
            case R.id.network:
                networkConfig.initNetworkConfigDialog(this, getLayoutInflater() );
                networkConfig.getNetworkDialog().show();
                break;
            default:
                return super.onOptionsItemSelected(item);
        }*/

        return true;
    }


    @Override
    protected void onDestroy()
    {
        UniversalDat.setDestroyRequested(true); // Notify ROS node to shut down

        super.onDestroy(); // Proceed with regular onDestry() magic
    }
}
