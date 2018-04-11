using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using UnityEngine;
using UnityEngine.SceneManagement;

public class CircleManager : MonoBehaviour {

    public GameObject centerCircle;
    public GameObject centerGlow;
    public GameObject[] outerCircles;
    private GameObject currentOuterCircle;
    public GameObject[] arrows;
    private GameObject currentArrow;
    public GameObject[] glow;
    private GameObject currentGlow;
    private MelNet hand_select;
    private MelNet viz_target_num_share;
    private MelNet state_net;
    private MelNet force_mag_share;
    private MelNet scenenum_share;

    public double[] sceneBuff = new double[1];
    public int scenenum;
    public int scenenum_prev;

    [Header("Targets")]
    public double[] get_targ = new double[1];
    public int c_target;
    private int c_int_target;
    public int u_target=1;
    public string hand;

    [Header("Force Sensor")]
    public double[] wrist_force = new double[1];
    private float arrow_size;

    public double[] is_present_target;

    // Use this for initialization
    void Start () {
        currentOuterCircle = outerCircles[0];
        currentArrow = arrows[0];
        currentGlow = glow[0];
        scenenum_share = new MelNet(50001, 50000, IPAddress.Parse("10.98.64.109"), false);
        hand_select = new MelNet(50007, 50006, IPAddress.Parse("10.98.64.109"), false);
        viz_target_num_share = new MelNet(50003, 50002, IPAddress.Parse("10.98.64.109"), false);
        state_net = new MelNet(50011, 50010, IPAddress.Parse("10.98.64.109"), false);
        force_mag_share = new MelNet(50005, 50004, IPAddress.Parse("10.98.64.109"), true);
    }

    // Update is called once per frame
   void Update()
    {
        //hand = MelShare.ReadMessage("hand");
        if (hand_select.ReceiveMessage() == "hand_flag")
        {
            hand = hand_select.ReceiveMessage();
        }

        if (Input.GetKeyDown(KeyCode.M))
        {
            SceneManager.LoadScene(0); //returns to main menu when M is pressed
        }

        scenenum = SceneManager.GetActiveScene().buildIndex;

        if (scenenum != scenenum_prev)
        {
            scenenum_share.SendMessage("scene_change");
            sceneBuff[0] = (double) scenenum;
            scenenum_share.SendData(sceneBuff);
            scenenum_prev = scenenum;
        }
        //MelShare.ReadMap("target", get_targ);

        if (viz_target_num_share.ReceiveMessage() == "target_flag")
        {
            get_targ = viz_target_num_share.ReceiveData();
        }
        c_target = (int) get_targ[0];

        if (state_net.ReceiveMessage() == "state_flag")
        {
            is_present_target = state_net.ReceiveData();
        }

        //MelShare.ReadMap("force_mag", wrist_force);

        if (is_present_target[0] == 1)
        {
            force_mag_share.SendMessage("send_force");
            wrist_force = force_mag_share.ReceiveData();

        }
        arrow_size = (float)wrist_force[0] / 50;


        if (c_target == 1)
        {
            if (hand == "R" && ((scenenum > 5 && scenenum < 14) || scenenum > 17))
            {
                u_target = 1;
            }

            else
            {
                u_target = 0;
            }

            OuterTarget(u_target);
        }

        if (c_target == 2)
        {
            if (hand == "R" && ((scenenum > 5 && scenenum < 14) || scenenum > 17))
            {
                u_target = 0;
            }

            else
            {
                u_target = 1;
            }
            OuterTarget(u_target);
        }

        if (c_target == 3)
        {
            if (hand == "R" &&  scenenum > 17)
            {
                u_target = 3;
            }
            else
            {
                u_target = 2;
            }
            OuterTarget(u_target);
        }

        if (c_target == 4)
        {
            if (hand == "R" && scenenum > 17)
            {
                u_target = 2;
            }
            else
            {
                u_target = 3;
            }
            OuterTarget(u_target);
        }

        //if (Input.GetKey(KeyCode.Space))
        //{
        //    currentArrow.transform.localScale = new Vector3(0, (float)wrist_force[0], 0); //scales the length of the arrow when space pressed
        //}

        if (c_target == 0)
        {
            CenterCircle(u_target);
            
        }

        //scenenum_prev = scenenum;

    }


    void OnTriggerEnter2D(Collider2D other)
    {
        if (other.gameObject.CompareTag("Player"))
        {
            GetComponent<SpriteRenderer>().color = new Color(0.388235229f, 0.3372549f, 1f); //turn target blue when player is inside it
        }
    }


    void OnTriggerExit2D(Collider2D other)
    {
        if (other.gameObject.CompareTag("Player"))
        {
            GetComponent<SpriteRenderer>().color = new Color(0f, 0f, 0f); //returns target to black when player exits it
        }

    }

    void OuterTarget(int u_target)
    {
        centerCircle.GetComponent<SpriteRenderer>().color = Color.black; // set the center circle color to black
        centerGlow.SetActive(false);     //turn off the glow for the center target
        currentOuterCircle = outerCircles[u_target]; // pick a new random outer circle
        currentOuterCircle.GetComponent<SpriteRenderer>().color = Color.green; // set the outer circle color to green
        currentArrow = arrows[u_target];
        currentArrow.SetActive(true); // activates arrowfor target
        currentGlow = glow[u_target];
        currentGlow.SetActive(true); // activates the glow background for target
        currentArrow.transform.localScale = new Vector3(50, arrow_size, 0); //scales the length of the arrow when space pressed
    }

    void CenterCircle(int u_target)
    {
        currentOuterCircle.GetComponent<SpriteRenderer>().color = Color.black; // set the outer circle color to black
        centerCircle.GetComponent<SpriteRenderer>().color = Color.green; // set the center circle color to green
        centerGlow.SetActive(true);     //turn on the glow for the center target
        currentArrow.SetActive(false);  //turn off the current arrow
        currentGlow.SetActive(false);   //turn off the target glow background
        currentArrow.transform.localScale = new Vector3(50, 45, 1);  //reset the last arrow to the normal size
    }

}
