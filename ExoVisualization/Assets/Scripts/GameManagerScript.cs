using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class GameManagerScript : MonoBehaviour {

    // graphics objects
    public Canvas main_menu;
    public TargetRingScript target_ring;
    public Canvas game_canvas;


    // internal controls
    public int dof; // current dof (0-5)
    public int mode; // current mode (0-3)
    

    // external controls
    public double[] scene_num; // single value; 0 for main menu and 1-24 for the other possible dof/mode combinations; written to MelShare, therefore must be array
    double prev_scene_num; // scene num value at previous update, used for detecting change in scene number
    public double[] current_target;
    double prev_target;
    double[] current_effort;

    // melshares
    MelShare ms_scene;
    MelShare ms_target;
    MelShare ms_effort;

    // Use this for initialization
    void Start () {

        main_menu.enabled = true;
        game_canvas.enabled = false;

        
        scene_num = new double[1];
        scene_num[0] = 0.0;
        prev_scene_num = 0.0;
        current_target = new double[1];
        current_target[0] = -1.0;
        prev_target = -1.0;
        current_effort = new double[1];
        current_effort[0] = 1.0;
        

        ms_scene = new MelShare("melshare_scene");
        ms_target = new MelShare("melshare_target");
        ms_effort = new MelShare("melshare_effort");
        ms_scene.WriteData(scene_num);

    }

    // Update is called once per frame
    void Update () {

        //============================================
        // BEGIN CODE EXECUTED WHEN SCENE SELECTED
        //============================================

        if (!main_menu.enabled) // check that main menu is disabled, meaning scene has been selected
        {
            // get latest data written to MelShare by robot control code
            current_target = ms_target.ReadData();
            current_effort = ms_effort.ReadData();

            if (current_target.Length > 0) // check that current target number was successfully read from MelShare (not resized) before reading from it
            {
                if (current_target[0] != prev_target) // check for change in target number read from MelShare
                {
                    if (current_target[0] >= 0 & current_target[0] <= 8) // check that current target number is within valid range of 0-8
                    {
                        // if current target number is within range 0-8
                        target_ring.set_target((int)current_target[0]); // reset all targets, then set selected target to be green with glow effect
                        target_ring.show_arrow((int)current_target[0]); // show the arrow pointing to the selected target
                    }
                    else
                    {
                        // if current target number is outside of range 0-8...
                        target_ring.reset_target_ring(); // reset target colors to black and turn off glow effect
                        target_ring.hide_arrows(); // hide all arrows
                    }
                }
                if (current_effort.Length > 0) // check that current effort level was successfully read from MelShare (not resized) before reading from it
                {
                    if (current_target[0] > 0 & current_target[0] <= 8) // check non-center target was selected based on target number in range of 1-8
                    {
                        target_ring.set_arrow_scale((int)current_target[0], current_effort[0]); // set size of current target arrow based on value of current effort
                    }
                }
            }
            
            // Check for keypress "m", meaning the user wants to return to main menu
            if (Input.GetKeyDown(KeyCode.M))
            {
                target_ring.hide_game();
                game_canvas.enabled = false;
                scene_num[0] = 0.0;
                current_target = new double[1];
                current_target[0] = -1.0;
                main_menu.enabled = true;
            }

        }

        //============================================
        // END CODE EXECUTED WHEN SCENE SELECTED
        //============================================



        //======================================
        // BEGIN CODE EXECUTED IN ALL SCENES
        //======================================

        // Check for a change in scene number, which would be triggered by a button press, and changed in the ModeButtonScript (or due to pressing "m" on the keyboard)
        if (scene_num[0] != prev_scene_num)
        {
            ms_scene.WriteData(scene_num); // write scene number to MelShare so that the robot knows the current scene, DoF, Mode
        }
        prev_scene_num = scene_num[0]; // update previous scene number as current scene number


        if (current_target.Length > 0) // check that current target number was successfully read from MelShare (not resized) before reading from it
        {
            prev_target = current_target[0]; // update previous target number as current target number
        }

        //======================================
        // END CODE EXECUTED IN ALL SCENES
        //======================================

    }








}
