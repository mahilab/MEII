using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ModeButtonScript : MonoBehaviour {

    public int mode; // mode that the button triggers, assigned by the DofMenuScript upon creation
    GameManagerScript game_manager; // reference to the Game Manager Script
    GameObject center_target; // reference to the center target
    GameObject[] outer_targets; // reference to the outer targets
    GameObject force_ring; // reference to the ring
    string[] dof_names = { "Elbow F/E", "Forearm P/S", "Wrist F/E", "Wrist R/U", "Elbow-Forearm", "Wrist Both" }; // names of the dofs to appear as subtitles of the submenus
    string[] mode_names = { "Calibration", "Training", "Blind Testing", "Full Testing" };// names of the modes to appear in the buttons

    void Awake()
    {
        GetComponent<Button>().onClick.AddListener(mode_button_callback); // set the function mode_button_callback to be exectued upon button press
    }

    void Start()
    {
        game_manager = GetComponentInParent<GameManagerScript>(); // assign the game manager script, which is the parent of all game objects
        center_target = game_manager.target_ring.center_target; // assign the center target object, which is a public variable of the TargetRing
        outer_targets = game_manager.target_ring.outer_targets; // assign the outer target objects, which is a public variable of the TargetRing
        force_ring = game_manager.target_ring.force_ring; // assign the ring object, which is a public variable of the TargetRing
    }

    // routine executed upon button press
    void mode_button_callback()
    {
        game_manager.dof = GetComponentInParent<DoFMenuScript>().dof; // set the dof variable at the highest level (GameManager) based on which submenu
        game_manager.mode = mode; // set the mode variable at the highest level (GameManager) based on which button

        game_manager.scene_num[0] = game_manager.dof * 4.0 + game_manager.mode + 1.0; // set the scene number variable at the highest level (GameManager)

        game_manager.main_menu.enabled = false; // hide the main menu
        game_manager.game_canvas.GetComponentInChildren<Text>().text = 
            string.Concat(dof_names[game_manager.dof], " \n", mode_names[game_manager.mode], " "); // set the text for the current scene
        game_manager.game_canvas.enabled = true; // make the text visible

        center_target.GetComponent<SpriteRenderer>().enabled = true; // make center target visible

        // make certain outer targets visible based on which DoF(s) selected
        switch (game_manager.dof)
        {
            case 0: // elbow f/e
                outer_targets[2].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[6].GetComponent<SpriteRenderer>().enabled = true;
                break;
            case 1: // forearm p/s
                outer_targets[0].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[4].GetComponent<SpriteRenderer>().enabled = true;
                break;
            case 2: // wrist f/e
                outer_targets[0].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[4].GetComponent<SpriteRenderer>().enabled = true;
                break;
            case 3: // wrist r/u
                outer_targets[2].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[6].GetComponent<SpriteRenderer>().enabled = true;
                break;
            case 4: // elbow-forearm
                outer_targets[1].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[3].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[5].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[7].GetComponent<SpriteRenderer>().enabled = true;
                break;
            case 5: // wrist both
                outer_targets[1].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[3].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[5].GetComponent<SpriteRenderer>().enabled = true;
                outer_targets[7].GetComponent<SpriteRenderer>().enabled = true;
                break;
        }

        // determine when we want to show the force ring
        if (mode == 0) // if calibration
        {
            force_ring.GetComponent<SpriteRenderer>().enabled = false;
        }
        else
        {
            force_ring.GetComponent<SpriteRenderer>().enabled = false;
        }
    }


}
