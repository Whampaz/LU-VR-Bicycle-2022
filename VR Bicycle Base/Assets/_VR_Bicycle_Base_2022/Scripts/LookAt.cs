using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 
This scripts only purpose is to "look at" the rotating ViveTracker
Then the angle between this object and the ViveTracker is calculated 
in VR_Controller and is used to detect pedaling, not pedaling & breaking. 

 */
public class LookAt : MonoBehaviour
{
    private GameObject ViveTracker;
    void Awake()
    {
        //Find the ViveTracker object from the Unity Scene
        ViveTracker = GameObject.Find("Vive Tracker");
    }

    void FixedUpdate()
    {
        //Orient this object to look at the ViveTracker.
        this.transform.LookAt(ViveTracker.transform);
    }
}
