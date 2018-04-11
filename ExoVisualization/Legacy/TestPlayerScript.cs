using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class TestPlayerScript : MonoBehaviour {

    // public float speed = 1000; // speed at which the player can move

    private double target_dist = 450;
    public double horiz;
    public double vert;
    private double horiz_left = -1;
    private double horiz_n = 0;
    private double horiz_right = 1;
    private double vert_up = 1;
    private double vert_n = 0;
    private double vert_down = -1;

    private double el_up = -5;
    private double el_n = -35;
    private double el_down = -65;

    private double fa_left = 30;
    private double fa_n = 0;
    private double fa_right = -30;

    private double wf_left = 15;
    private double wf_n = 0;
    private double wf_right = -15;

    private double wr_up = 15;
    private double wr_n = 0;
    private double wr_down = -15;


    public int result;

    [Header("EXO Joint Positions")]
    public double[] jointAngle = new double[5];
    public int scenenum;
    public int[] sceneBuff = new int[1];

    //private float horiz;
    //private float vert;

    // Use this for initialization
    void Start () {


    }

    // Update is called once per frame
    void Update() {

        scenenum = SceneManager.GetActiveScene().buildIndex;
        //sceneBuff[0] = scenenum;

        //result = MelShare.WriteMap("scene_num", sceneBuff);

        if (scenenum > 1 && scenenum <= 4)
        {
            horiz = 0;
            vert = jointAngle[0];
            vert_up = el_up;
            vert_n = el_n;
            vert_down = el_down;
        }

        if (scenenum > 4 && scenenum <= 7)
        {
            horiz = jointAngle[1];
            vert = 0;
            horiz_left = fa_left;
            horiz_n = fa_n;
            horiz_right = fa_right;
        }

        if (scenenum > 7 && scenenum <= 10)
        {
            horiz = jointAngle[2];
            vert = 0;
            horiz_left = wf_left;
            horiz_n = wf_n;
            horiz_right = wf_right;
        }

        if (scenenum > 10 && scenenum <= 13)
        {
            horiz = 0;
            vert = jointAngle[3];
            vert_up = wr_up;
            vert_n = wr_n;
            vert_down = wr_down;

        }

        if (scenenum > 13 && scenenum <= 16)
        {
            horiz = jointAngle[1];
            horiz_left = fa_left;
            horiz_n = fa_n;
            horiz_right = fa_right;
            vert = jointAngle[0];
            vert_up = el_up;
            vert_n = el_n;
            vert_down = el_down;
        }
        
        if (scenenum > 16 && scenenum <= 19)
        {
            horiz = jointAngle[2];
            horiz_left = wf_left;
            horiz_n = wf_n;
            horiz_right = wf_right;
            vert = jointAngle[3];
            vert_up = wr_up;
            vert_n = wr_n;
            vert_down = wr_down;
        }

        horiz = target_dist / ((horiz_right - horiz_left) / 2) * (horiz * 180 / 3.141592653589793 - horiz_n);
        vert = target_dist/ ((vert_up - vert_down) / 2) * (vert * 180 / 3.141592653589793 - vert_n);

        // read in the current pendulum state
        //result = MelShare.ReadMap("MEII_pos", jointAngle);
        if (result > 0)
        {
            transform.position = new Vector3((float)horiz, (float)vert, 0);
            print(jointAngle);
        }

    }

    void OnTriggerEnter2D(Collider2D other)
    {
        if (other.gameObject.CompareTag("target"))
        {
            other.GetComponent<SpriteRenderer>().color = new Color(0.388235229f, 0.3372549f, 1f);
        }    
    }

    void OnTriggerExit2D(Collider2D other)
    {
        if (other.gameObject.CompareTag("target"))
        {
            other.GetComponent<SpriteRenderer>().color = new Color(0f, 0f, 0f);
        }
    }
}
