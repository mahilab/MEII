// Evan Pezent (epezen@rice.edu)
// June 2017

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum TargetState { Inactive, Go, Stop, Score, PerfectScore };

public class TargetScript : MonoBehaviour {

    /*// PART 1
    public Sprite[] outerSpriteLayers; // an array of sprites we will use for outer targets
    public Sprite[] centerSpriteLayers; // an array of sprites we will use for the center target

    public SpriteRenderer layer0; // the SR on layer0
    public SpriteRenderer layer1; // the SR on layer1

    // various colors we may want targets to be
    public Color transparent = new Color(1,1,1,0);
    public Color inactiveColor = Color.white;
    public Color goColor = Color.green;
    public Color stopColor = Color.magenta;
    public Color scoreColor = Color.cyan;
    public Color perfScoreColor = Color.yellow;

    public bool isCenterTarget = false;  // a flag to indicate if this is the center target

    // the current state of the target
    // this is an example of a C# propoerty
    // state is private, but can be accessed through State which is public
    // useful for when we want to do things (UpdateColor, UpdateCollider) when at variable (state) is changed
    private TargetState state;
    public TargetState State
    {
        get { return state; }
        set { state = value; UpdateColor(); UpdateCollider(); }
    }

    private Collider2D col; // references to other components attached to our target GameObject
    //*/

    /*// PART AUDIO
    // various sounds we want the target to play
    public AudioClip activateSound;
    public AudioClip hitSound;
    public AudioClip missSound;

    private AudioSource audSrc; // references to other components attached to our target GameObject
    //*/

    // Use this for initialization
    void Awake()
    {
        // PART 1:        col = GetComponent<Collider2D>();
    }
    void Start () {
        /*// PART 1
        // get references to other components
        
        // set the sprites based on whether this is the center target or not
        if (isCenterTarget)
        {
            layer0.sprite = centerSpriteLayers[0];
            layer1.sprite = centerSpriteLayers[1];
        }
        else
        {
            layer0.sprite = outerSpriteLayers[0];
            layer1.sprite = outerSpriteLayers[1];
        }
        // set both layers to be transparent on startup
        layer0.color = transparent;
        layer1.color = transparent;        
        State = TargetState.Inactive; // set initial state as Inactive
        //*/

        /*// PART AUDIO
        audSrc = GetComponent<AudioSource>();
        //*/ 

        /*// PART ANIMATION 1
        StartCoroutine(MyFunctionClass.FadeSpriteColor(layer0, inactiveColor, 1.0f)); // fade in layer0
        //*/
    }

    // Update is called once per frame
    void Update () {
        /*// PART FINAL TOUCHES
        // if the target is inactive or the center target, make it dance!
        if ((State == TargetState.Inactive) || isCenterTarget)
        {
            float scale = 1 + 0.2f * GetComponentInParent<GameManagerScript>().beatIntensity;
            transform.localScale = new Vector3(scale, scale, 0);
        }
        //*/
	}

    /*// PART AUDIO
    // this is an inhereted function that gets called when a collision occurs with this GameObject
    void OnCollisionEnter2D(Collision2D collision)
    {
        // if the other object is the Player and this isn't the center target, play hit sound
        if (collision.gameObject.GetComponent<PlayerScript>() && !isCenterTarget)
            audSrc.PlayOneShot(hitSound);
    }

    // this is an inhereted function that gets called when a collision occurs with this GameObect AND it's collider is set to Trigger
    void OnTriggerEnter2D(Collider2D collision)
    {
        // if the other object is the Player and this isn't the center target, play miss sound
        if (collision.gameObject.GetComponent<PlayerScript>() && !isCenterTarget)
            audSrc.PlayOneShot(missSound);
    }
    // */

    /*// PART 1 
    // Updates the color of the target based on the current State
    void UpdateColor()
    {
        if (state == TargetState.Inactive)
        {
            layer0.color = inactiveColor;
            layer1.color = transparent;
            // PART ANIMATION 1:             StartCoroutine(MyFunctionClass.FadeSpriteColor(layer1, transparent, 0.1f));
        }
        else if (state == TargetState.Go)
        {
            layer1.color = goColor;
            // PART ANIMATION:             StartCoroutine(MyFunctionClass.FadeSpriteColor(layer1, goColor, 0.1f));

            //PART AUDIO:             if (!isCenterTarget)
                // PART AUDIO:                  audSrc.PlayOneShot(activateSound);
            
        }
        else if (state == TargetState.Stop)
        {
            layer1.color = stopColor;
            // PART ANIMATION 1:             StartCoroutine(MyFunctionClass.FadeSpriteColor(layer1, stopColor, 0.1f));            
        }
        else if (state == TargetState.Score)
        {
            layer1.color = scoreColor;
            // PART ANIMATION 1:             StartCoroutine(MyFunctionClass.FadeSpriteColor(layer1, scoreColor, 0.1f));
        }
        else if (state == TargetState.PerfectScore)
        {
            layer1.color = perfScoreColor;
            // PART ANIMATION 1:             StartCoroutine(MyFunctionClass.FadeSpriteColor(layer1, perfScoreColor, 0.1f));
        }
    }
    //*/

    // updates the target Collider based on the state
    /*// PART 1
    void UpdateCollider()
    {
        if (state == TargetState.Inactive || isCenterTarget)
            col.isTrigger = true;
        else
            col.isTrigger = false;
    }
    //*/

    /*// PART KILL
    // fades out both layers and disables the collider
    public void Kill()
    {
        col.enabled = false;
        StartCoroutine(MyFunctionClass.FadeSpriteColor(layer0, transparent, 0.25f));
        StartCoroutine(MyFunctionClass.FadeSpriteColor(layer1, transparent, 0.25f));
    }
    //*/


}
