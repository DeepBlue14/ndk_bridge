package edu.uml.cs.danrosjcxxprac;


import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.EditText;

public class NetworkConfig
{
    private AlertDialog.Builder networkDialog; /** Is the dialog which will present networking options (if the user is logged in as administrator. */


    /**
     * Initializes the network configuration dialog (which can only be accessed by
     * an administrator).
     *
     * @param activity
     * @param layoutInflater
     */
    public void initNetworkConfigDialog(Activity activity, LayoutInflater layoutInflater)
    {
        LayoutInflater inflater = layoutInflater;
        final View view = inflater.inflate(R.layout.network_config, null);

        networkDialog = new AlertDialog.Builder(activity)
                .setTitle("ROS env var config")
                .setMessage("ROS env var config")
                .setView(view)
                .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener()
                {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        EditText rosMasterUi = (EditText) view.findViewById(R.id.ros_master_uri);
                        EditText rosIpUi = (EditText) view.findViewById(R.id.ros_ip);

                        if(!rosMasterUi.getText().toString().isEmpty() )
                        {
                            UniversalDat.setRosMasterUri(rosMasterUi.getText().toString());
                        }
                        if(!rosIpUi.getText().toString().isEmpty() )
                        {
                            UniversalDat.setRosIp(rosIpUi.getText().toString());
                        }

                        //setUrl(ipAddressStr, portInt);
                    }
                })
                .setNegativeButton(android.R.string.cancel, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        System.out.println("\"Cancel\" alert button selected");
                        //System.exit(0);
                    }
                })
                .setIcon(android.R.drawable.ic_dialog_alert);
    }


    /**
     * Mutator.
     * @see #networkDialog
     *
     * @param networkDialog
     */
    public void setNetworkDialog(AlertDialog.Builder networkDialog)
    {
        this.networkDialog = networkDialog;
    }


    /**
     * Accessor.
     * @see #networkDialog
     *
     * @return
     */
    public AlertDialog.Builder getNetworkDialog()
    {
        return networkDialog;
    }


    /**
     * TODO: implement this.
     *
     * @return
     */
    public String toString()
    {
        return "^^^*** METHOD STUB ***^^^";
    }


} // End of class NetworkConfig