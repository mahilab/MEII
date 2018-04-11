// Evan Pezent (epezen@rice.edu)
// June 2017

using System.Collections;
using UnityEngine;

public class TargetManagerScript : MonoBehaviour {

    /*// PART 1
    public GameObject targetPrefab; // our target prefab

    public float startRadius = 450.0f; // the starting radius of the targets
    public float currentRadius = 450.0f; // the current radius of the targets
    public float endRadius = 93.0f; // the final radius of the targets (when GAMEOVER)

    private TargetScript centerTarget; // the center target 
    private TargetScript[] outerTargets = new TargetScript[8]; // array of the outer targets

    // how long a target stays green = random
    public float greenDurationMin = 2;
    public float greenDurationMax = 3;
    // how long a target stays red = f(currentRadius)
    public float redDurationMin = 0.25f;
    public float redDurationMax = 1f;
    public float holdDuration = 2.0f;

    public int roundCount = 0; // the number of rounds passed
    public int perfectCount = 0; // the number of perfect returns

    public float squeezeForce = 500; // the inward force applied to targets
    public float spingStiffness = 1000; // stiffness of the virtual spring that keeps targets on trajectory

    private TargetScript currentOuterTarget; // the outer target currently active
    private bool roundInProgress = false;

    private PlayerScript player;
    private bool targetsLoaded; // flag to indicate that targets have been loaded
    //*/

    /*//PART AUDIO
    public AudioClip scoreSound;
    public AudioClip perfectSound;
    private AudioSource audSrc;  // references to other components attached to the TargetManager GameObject
    //*/

    // Use this for initialization
    void Start () {
        /*// PART 1
        // get references to other components
        player = FindObjectOfType<PlayerScript>(); // this finds the first instance of a PlayerScript in the scene (there should only be one, so OK)
        currentRadius = startRadius;
        //LoadTargets();
        //*/

        // PART AUDIO:         audSrc = GetComponent<AudioSource>();
        // PART ANIMATION 2:         StartCoroutine(LoadTargets()); // load our targets

    }

    /*// PART COUROUTINE
    void FixedUpdate()
    {
        if (targetsLoaded)
        {
            if (currentRadius > endRadius)
            {
                if (!roundInProgress)
                {
                    StartCoroutine(NewRound());
                }
                // PART SQUEEZE:                 SqueezeTargets();                
            }
            else // game over condition
            {
                currentRadius = endRadius;
                // PART KILL:                 KillTargets();
            }
        }
    }
    //*/

    /*// PART 1
    // loads outer and center targets
    void LoadTargets()
    // PART ANIMATION 2:     IEnumerator LoadTargets()
    {
        for (int i = 0; i < outerTargets.Length; i++) // for every target
        {
            // calculalte axis
            float angle = i * 2.0f * Mathf.PI / outerTargets.Length; 
            Vector2 axis = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));
            outerTargets[i] = Instantiate(targetPrefab).GetComponent<TargetScript>();
            outerTargets[i].transform.eulerAngles = new Vector3(0, 0, -90+45*i);
            outerTargets[i].transform.position = axis * startRadius;
            outerTargets[i].transform.parent = this.transform;
            // PART ANIMATION 2:             yield return new WaitForSeconds(0.8f);
        }
        centerTarget = Instantiate(targetPrefab).GetComponent<TargetScript>();
        centerTarget.transform.position = new Vector3(0, 0, 0);
        centerTarget.transform.parent = this.transform;
        centerTarget.isCenterTarget = true;
        // PART ANIMATION 2:         yield return new WaitForSeconds(1.0f);
        targetsLoaded = true;
    } 
    //*/

    /*// PART COROUTINE 
    // starts a NewRound sequence
    IEnumerator NewRound()
    {
        roundInProgress = true;
        currentOuterTarget = outerTargets[Random.Range(0, outerTargets.Length)]; // pick a new random outer target
        currentOuterTarget.State = TargetState.Go;
        yield return new WaitForSeconds(Random.Range(greenDurationMin, greenDurationMax)); // yield control to Update() for greenDuration seconds
        currentOuterTarget.State = TargetState.Stop;
        float t = (endRadius - currentRadius) / (startRadius - endRadius) + 1;
        yield return new WaitForSeconds(Mathf.Lerp(redDurationMax, redDurationMin, t));
        currentOuterTarget.State = TargetState.Inactive; // set the outer target color to black
        centerTarget.State = TargetState.Go; // set the center target color to blue
        // wait for the player to return to the center target
        bool perfect = true;
        while (!player.isNearCenter)
        {
            perfect = false;
            yield return null;
        }
        if (perfect)
        {
            perfectCount += 1;
            centerTarget.State = TargetState.PerfectScore;
            // PART AUDIO:             audSrc.PlayOneShot(perfectSound);
        }
        else
        {
            centerTarget.State = TargetState.Score;
            // PART AUDIO:             audSrc.PlayOneShot(scoreSound);
        }
        roundCount += 1; // increment round count
        squeezeForce += 1; // increment Squeeze force for increasing difficultly
        yield return new WaitForSeconds(holdDuration);
        centerTarget.GetComponent<TargetScript>().State = TargetState.Inactive;
        roundInProgress = false;
    }
    //*/

    /*//PART SQUEEZE
    // pushes the targets toward the center
    void SqueezeTargets()
    {
        currentRadius = currentOuterTarget.transform.position.magnitude; // set current radius based on distance to active outer target
        for (int i = 0; i < outerTargets.Length; i++) // for every target
        {
            // calculate axis
            float angle = i * 2.0f * Mathf.PI / outerTargets.Length; 
            Vector2 axis = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));           
            if (outerTargets[i] == currentOuterTarget)  // if this is the active outer target, we will apply forces to it
            {                
                currentOuterTarget.GetComponent<Rigidbody2D>().AddForce((-currentOuterTarget.transform.position).normalized * squeezeForce);
                // determine normal distance between center point of active outer target and its axis
                float num = axis.y * currentOuterTarget.transform.position.x - axis.x * currentOuterTarget.transform.position.y;
                float den = Mathf.Sqrt(axis.y * axis.y + axis.x * axis.x);
                float dist = num / den;
                Vector3 perp = new Vector2(axis.y, -axis.x); // get vector perpendicular to target axis                
                currentOuterTarget.GetComponent<Rigidbody2D>().AddForce(-spingStiffness * dist * perp); // add spring force to target so it stays on the axis
            }
            else // if this is not the active target, we will update its distance based on the active outer target
            {
                outerTargets[i].transform.position = axis * currentRadius;
            }
        }        
    }    
    //*/

    /*//PART KILL
    // kills all Targets
    public void KillTargets()
    {
        StopAllCoroutines();
        foreach (TargetScript target in outerTargets)
            target.GetComponent<TargetScript>().Kill();
        centerTarget.GetComponent<TargetScript>().Kill();
    }
    //*/

}
