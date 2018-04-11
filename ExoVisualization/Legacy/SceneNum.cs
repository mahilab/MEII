using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.SceneManagement;


public class SceneNum : MonoBehaviour {

    public int result;
    public int scenenum = 0;
    public int scenenum_prev;
    public double[] sceneBuff = new double[1];
    private MelNet scenenum_share;
    private MelShare ms_scene;


    // Use this for initialization
    void Start () {
       scenenum_share = new MelNet(50001, 50000, IPAddress.Parse("10.98.64.109"), false);
       scenenum_share.SendMessage("scene_change");
       sceneBuff[0] = scenenum;
       scenenum_share.SendData(sceneBuff);

    }
	
	// Update is called once per frame
	void Update () {
        scenenum = SceneManager.GetActiveScene().buildIndex;

        if (scenenum != scenenum_prev)
        {
            scenenum_share.SendMessage("scene_change");
            sceneBuff[0] = (double)scenenum;
            scenenum_share.SendData(sceneBuff);
            scenenum_prev = scenenum;
        }

        //result = MelShare.WriteMap("scene_num", sceneBuff);


    }
}
