using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DoFMenuScript : MonoBehaviour {
    
    public GameObject mode_button_prefab; // generic mode button from which the elements of the array mode_buttons are built
    GameObject[] mode_buttons; // array of buttons, one for each mode
    public int dof; // dof number associated with this submenu
    int num_buttons = 4; // number of buttons in this submenu
    string[] mode_names = { "Calibration", "Training", "Blind Testing", "Full Testing"}; // names to appear in the buttons

    // Use this for initialization
    void Start () {

        // Create the submenu UI, which consists of a submenu title and a column of buttons
        mode_buttons = new GameObject[num_buttons]; // initialize the size of the button array
        for (int i = 0; i < num_buttons; ++i)
        {
            mode_buttons[i] = Instantiate(mode_button_prefab); // first generate each instance of the mode button prefab object, following lines set the properties
            mode_buttons[i].GetComponent<RectTransform>().SetParent(GetComponent<RectTransform>(), false); // set the parent of each button to be the submenu, using the RectTransform
            mode_buttons[i].GetComponent<RectTransform>().anchoredPosition = new Vector3(0.0f, 50.0f - i * 75.0f, 0.0f); // set the position of each button relative to the submenu
            mode_buttons[i].GetComponentInChildren<Text>().text = mode_names[i]; // set the mode name to appear inside button
            mode_buttons[i].GetComponent<ModeButtonScript>().mode = i; // set the mode number associated with each button

        }
		
	}
	

}
