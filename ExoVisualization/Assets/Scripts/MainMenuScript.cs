using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MainMenuScript : MonoBehaviour {

    public GameObject submenu_prefab; // generic submenu from which the elements of the array submenus are built
    GameObject[] submenus; // array of submenus, one for each DoF
    int num_dof = 6; // number of submenus
    string[] dof_names = { "Elbow F/E", "Forearm P/S", "Wrist F/E", "Wrist R/U", "Elbow-Forearm", "Wrist Both" }; // names to appear as subtitles of the submenus

    // Use this for initialization
    void Start()
    {
        // Create the main menu (UI canvas) by generating submenus, each parented to the main menu.
        submenus = new GameObject[num_dof]; // initialize the size of the submenu array
        for (int i = 0; i < num_dof; ++i)
        {
            submenus[i] = Instantiate(submenu_prefab); // first generate each instance of the submenu prefab object, following lines set the properties
            submenus[i].GetComponent<RectTransform>().SetParent(GetComponent<RectTransform>(), false); // set the parent of each submenu to be the main menu, using the RectTransform
            submenus[i].GetComponent<Text>().text = dof_names[i]; // set the dof name (appearing as the subtitle) for each submenu
            submenus[i].GetComponent<RectTransform>().anchoredPosition = 
                new Vector3(-600.0f + Mathf.Floor(i / 2.0f) * 600.0f, 200.0f - (i - 2.0f * Mathf.Floor(i / 2.0f)) * 450.0f, 0.0f); // set the position of each submenu relative to the main menu
            submenus[i].GetComponent<DoFMenuScript>().dof = i; // set the DoF number associate with each submenu
        }
       
    }

}
