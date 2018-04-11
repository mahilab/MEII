using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class TargetRingScript : MonoBehaviour {

    public GameObject center_target_prefab;
    public GameObject center_target;
    public GameObject center_glow_prefab;
    public GameObject center_glow;
    public GameObject outer_target_prefab;
    public GameObject[] outer_targets;
    public GameObject outer_glow_prefab;
    public GameObject[] outer_glows;
    public GameObject arrow_prefab;
    public GameObject[] arrows;
    public GameObject ring_prefab;
    public GameObject force_ring;
    float R = 450.0f; // radius of target ring from center target
    float r = 70.0f; // radius of arrow base position from center target
    float R_corner = 1.0f; // correction factor for when the targets are in the corners (multi-dof)

    // Use this for initialization
    void Start()
    {
        // draw target ring
        center_target = Instantiate(center_target_prefab); // generate an instance of the center target prefab object
        center_target.transform.SetParent(gameObject.transform, false); // set the parent of the center target object to be the TargetRing game object using the transform
        center_glow = Instantiate(center_glow_prefab); // generate an instance of the center glow object
        center_glow.transform.SetParent(gameObject.transform, false); // set the parent of the center glow object to be the TargetRing game object using the transform
        outer_targets = new GameObject[8]; // initialize the size of the outer_targets array
        outer_glows = new GameObject[8]; // initialize the size of the outer_glows array
        arrows = new GameObject[8]; // initialize the size of the arrows array
        for (int i = 0; i < 8; ++i)
        {
            outer_targets[i] = Instantiate(outer_target_prefab); // generate each instance of the outer target prefab object
            outer_targets[i].transform.SetParent(gameObject.transform, false); // set the parent of each outer target object to be the TargetRing game object using the transform
            outer_targets[i].transform.position = new Vector3(Mathf.Round(Mathf.Cos(i * 2 * Mathf.PI / 8.0f)) * R,
                Mathf.Round(Mathf.Sin(i * 2 * Mathf.PI / 8.0f)) * R, 0.0f); // set the position of each outer target relative to the TargetRing
            outer_targets[i].transform.eulerAngles = new Vector3(0.0f, 0.0f, -90.0f + i * 45.0f); // set the orientation of each outer target relative to the TargetRing

            outer_glows[i] = Instantiate(outer_glow_prefab); // generate each instance of the outer glow prefab object
            outer_glows[i].transform.SetParent(gameObject.transform, false); // set the parent of each outer glow object to be the TargetRing game object using the transform
            outer_glows[i].transform.position = new Vector3(Mathf.Round(Mathf.Cos(i * 2 * Mathf.PI / 8.0f)) * R,
                Mathf.Round(Mathf.Sin(i * 2 * Mathf.PI / 8.0f)) * R, 0.0f); // set the position of each outer glow relative to the TargetRing
            outer_glows[i].transform.eulerAngles = new Vector3(0.0f, 0.0f, -90.0f + i * 45.0f); // set the orientation of each outer glow relative to the TargetRing

            arrows[i] = Instantiate(arrow_prefab); // generate each instance of the arrow prefab object
            arrows[i].transform.SetParent(gameObject.transform, false); // set the parent of each arrow object to be the TargetRing game object using the transform
            arrows[i].transform.position = new Vector3(Mathf.Cos(i * 2 * Mathf.PI / 8.0f) * r,
                Mathf.Sin(i * 2 * Mathf.PI / 8.0f) * r, 0.0f); // set the position of each arrow relative to the TargetRing
            arrows[i].transform.eulerAngles = new Vector3(0.0f, 0.0f, -90.0f + i * 45.0f); // set the orientation of each arrow relative to the TargetRing
        }
        force_ring = Instantiate(ring_prefab); // generate a single instance of the ring prefab object
        force_ring.transform.SetParent(gameObject.transform, false); // set the parent of the ring object to be the TargetRing game object using the transform

        hide_game(); // hide all of the graphics objects that are children of the TargetRing

    }

    // reset target colors to black and turn off glow effect
    public void reset_target_ring()
    {
        center_glow.GetComponent<SpriteRenderer>().enabled = false;
        center_target.GetComponent<SpriteRenderer>().color = Color.black;
        for (int i = 0; i < outer_glows.Length; ++i)
        {
            outer_glows[i].GetComponent<SpriteRenderer>().enabled = false;
            outer_targets[i].GetComponent<SpriteRenderer>().color = Color.black;
        }
    }

    // make all targets invisible
    public void hide_target_ring()
    {
        reset_target_ring();
        center_target.GetComponent<SpriteRenderer>().enabled = false;
        center_glow.GetComponent<SpriteRenderer>().enabled = false;
        for (int i = 0; i < outer_targets.Length; ++i)
        {
            outer_targets[i].GetComponent<SpriteRenderer>().enabled = false;
            outer_glows[i].GetComponent<SpriteRenderer>().enabled = false;
        }
    }

    // reset all targets, then set selected target to be green with glow effect
    public void set_target(int target)
    {
        reset_target_ring();
        if (target == 0)
        {
            center_glow.GetComponent<SpriteRenderer>().enabled = true;
            center_target.GetComponent<SpriteRenderer>().color = Color.green;
        }
        else
        {
            outer_glows[target - 1].GetComponent<SpriteRenderer>().enabled = true;
            outer_targets[target - 1].GetComponent<SpriteRenderer>().color = Color.green;
        }
    }

    // hide all arrows
    public void hide_arrows()
    {
        for (int i = 0; i < arrows.Length; ++i)
        {
            arrows[i].GetComponent<SpriteRenderer>().enabled = false;
        }
    }

    // show an individual arrow, specified by integer from 1-8, corresponding to the target its pointing towards
    public void show_arrow(int arrow)
    {
        hide_arrows();
        if (arrow > 0)
        {
            arrows[arrow - 1].GetComponent<SpriteRenderer>().enabled = true;
        }
    }

    // scale an individual arrow, where 0 corresponds to 0 length and 1 corresponds to on the target
    public void set_arrow_scale(int arrow, double scale)
    {
        if (arrow > 0)
        {
            if (arrow % 2 == 0)
                R_corner = Mathf.Sqrt(2);
            else
                R_corner = 1.0f;
            arrows[arrow - 1].transform.localScale = new Vector3(50.0f, (float)scale * R_corner * 85.0f, 1.0f);
        }
    }

    // make the force ring invisible
    public void hide_force_ring()
    {
        force_ring.GetComponent<SpriteRenderer>().enabled = false;
    }

    // make all game graphics invisible
    public void hide_game()
    {
        hide_target_ring();
        hide_arrows();
        hide_force_ring();
    }
}


