using System.Collections;
using System.Collections.Generic;
using UnityEngine.SceneManagement;
using UnityEngine;

public class ModeIdentifier : MonoBehaviour {

    public int scenenum;

	// Update is called once per frame
	void Update () {
    
        scenenum = SceneManager.GetActiveScene().buildIndex;
    }
}
