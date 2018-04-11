using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class getsize : MonoBehaviour {

    private Vector3 size;

	// Use this for initialization
	void Start () {

        size = GetComponent<Renderer>().bounds.size;

    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
