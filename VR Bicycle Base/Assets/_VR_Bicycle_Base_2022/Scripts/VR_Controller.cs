using System.Collections;
using System.Collections.Generic;
using System.IO;
using System;
using UnityEditor;
using UnityEngine;
using Valve.VR;

public class VR_Controller : MonoBehaviour
{
    #region Time Checks
    private float gametime_clock = 0;
    private float f_dirAngle_clock;
    private List<Vector3> f_prev_positions_2 = new List<Vector3>();
    private Vector3 stoppos = Vector3.zero;
    private Vector3 stopposIM = Vector3.zero;
    private Vector3 stopposBR = Vector3.zero;
    private bool stopmaybe = false;
    #endregion
    #region Parameters
    [Header("GameObjects")]
    [SerializeField] private GameObject ViveController; //the right hand ViveController  as a gameObject in the scene. 
    [SerializeField] private GameObject ViveTracker; //the ViveTracker as a gameObject in the scene. 
    [SerializeField] private GameObject TrackerLookat; //The object that rotates locally to calculate rotation of Tracker
    [SerializeField] private GameObject SteeringParent; //the parent object holding the frontwheel & handlebar

    [Header("RigidBody")] 
    [SerializeField] private Rigidbody rigidBody;

    [Header("Models")] 
    [SerializeField] private GameObject frontWheel; //the 3d model of the frontwheel (ony applies rotation)
    [SerializeField] private GameObject backWheel; //the 3d model of the backwheel (ony applies rotation)
    [SerializeField] private GameObject BicycleModel; //the 3d model of the bicycle

    [Header("Wheelcolliders")]
    [SerializeField] private WheelCollider frontWC;
    [SerializeField] private WheelCollider backWC;

    [Header("Constant SimParameters")]
    [SerializeField] private float c_brakeForce = 3;
    [SerializeField] private float c_windBrake = 5;
    [SerializeField] private float c_speedMultiplier = 0.4f;
    [SerializeField] private float c_maxAcceleration = 10;
    [SerializeField] private float c_PhysicalCrankRadius = 0.17f;

    private float brakeTorque;
    private Vector3 frontWheelStartRotation; // initial rotation of the frontwheel
    private float VIVEControllerStartRotationY; //initial rotationvalue of the controller on the handlebar
    #endregion

    private void Awake() //reference & initialize all the used active items in scene if not set in Editor
    {
        //Find Vive components in scene if not set in Editor
        if (ViveController == null) 
            ViveController = GameObject.Find("Controller");
        if (ViveTracker == null) 
            ViveTracker = GameObject.Find("Vive Tracker");
        if (TrackerLookat == null) 
            TrackerLookat = GameObject.Find("TrackerLookat");

        //find model compoments in scene if not set in Editor
        if (SteeringParent == null) 
            SteeringParent = GameObject.Find("SteeringParent");      
        if (BicycleModel == null) 
            BicycleModel = GameObject.Find("Bicycle");

        //find colliders in scene if not set in Editor
        if (frontWC == null) 
            frontWC = GameObject.Find("w_col_front").GetComponent<WheelCollider>();
        if (backWC == null) 
            backWC = GameObject.Find("w_col_back").GetComponent<WheelCollider>();

        //Visuals if not set in Editor. 
        if (frontWheel == null)
            frontWheel  = GameObject.Find("frontWheel");
        if (backWheel == null)
            backWheel  = GameObject.Find("backWheel");

    }
    private void Start() //set starting parameters.
    {
        //Deactivate the mesh of Vivecontroller, remove to visualize position of controller for alignment purposes. 
        ViveController.GetComponent<MeshRenderer>().enabled = false;
        
        frontWheelStartRotation = SteeringParent.transform.rotation.eulerAngles;

        //start rotation of the right hand vive controller. On handlebar
        VIVEControllerStartRotationY = ViveController.transform.rotation.eulerAngles.y;

        if (rigidBody == null ) rigidBody = GetComponent<Rigidbody>();

        BicycleModel.transform.rotation = Quaternion.Euler(0, ViveController.transform.rotation.eulerAngles.y, 0);

        pedaling = false;
        brake = false;
        brakeonce = false;
        braketwice = false;
        brakeTorque = 10000f;
        stopmaybe = true;
        stoppos = ViveTracker.transform.localPosition;
        stopposIM = ViveTracker.transform.localPosition;
        stopposBR = ViveTracker.transform.localPosition;

    }


    #region FixedUpdate Parameters 
    [System.NonSerialized] public bool pedaling;
    [System.NonSerialized] public bool brake,brakeonce,braketwice;

    private Vector3 previousTrackerVector = Vector3.zero;
    [Header("Update Frequency")]
    [SerializeField, Tooltip("Set the desired update frequency, calulations will run at 50/update_frequency hz.")] 
    private int c_updateFrequency = 3;
    private int update_UpdateFreq = 0;

    float TrackerPedalValue = 0f;

    Vector3 v_currAngleDir = Vector3.zero;
    Vector3 v_prevAngleDir = Vector3.zero;
    Vector3 p_prevPos = Vector3.zero;
    float f_dirAngle;
    #endregion
    private void FixedUpdate() 
    {
        gametime_clock += Time.deltaTime; //keep track of gametime. in seconds. 
        #region Setup
        //Make sure to connect VIVE controller and throw old speed measurements.
        if (VIVEControllerStartRotationY == 0)
        {
            VIVEControllerStartRotationY = ViveController.transform.rotation.eulerAngles.y;

        }

        // Calibrate bicycle orientation at the beginning of simulation 
        if (gametime_clock > 3f && gametime_clock < 4f) 
        {
            BicycleModelSetup();
        }
        #endregion
        #region Update
        //Visuals();
        update_UpdateFreq += 1;
        if (update_UpdateFreq >= c_updateFrequency) //update frequecy, we dont want to evaluate the traveled tracker distance/angle to often to get better readings. at running at estimated 60 fps 3=20hz.
        {
            UpdateValues();
            StateManager();
            Physics_Speed();
            update_UpdateFreq = 0; //reset counter
        }       
        SteeringCalculation();
        
        TrackerRotationCalibration(); //keep trackertracker level and autocalibrate troughout simulation
        Visuals();
        #endregion
    }

    #region UpdateMethods
    private void UpdateValues()
    {
        v_currAngleDir = ViveTracker.transform.localPosition - p_prevPos; //current direction vector of the vive tracker
        p_prevPos = ViveTracker.transform.localPosition; //update previous position of the vive tracker
        f_dirAngle = Vector3.Angle(v_currAngleDir, v_prevAngleDir); //float, the angle between the current tracker dir and last tracker dir; if f_dirangle < 90deg we are most likley pedaling forward. 
        v_prevAngleDir = v_currAngleDir; //update previous angle

        f_prev_positions_2.Add(ViveTracker.transform.localPosition);
        if (f_prev_positions_2.Count > 15)
        {
            f_prev_positions_2.RemoveAt(0);
        }


        //currAngle, the angle between the vector that goes trackerlookat->vivetracker, and the same vector from last iteration.
        float currAngle = Vector3.Angle((ViveTracker.transform.localPosition - TrackerLookat.transform.localPosition), previousTrackerVector);

        TrackerPedalValue = currAngle;
        previousTrackerVector = (ViveTracker.transform.localPosition - TrackerLookat.transform.localPosition);

    }
    private void StateManager()
    {
        #region brake/pedal/no-pedal defined
        if (f_dirAngle > 90 && !stopmaybe)
        {
            f_dirAngle_clock = gametime_clock;
            // identify a previous position when potential braking is found, this gives us a direction that braking occurs in
            stoppos = Vector3.zero;
            for (int i = 0; i < f_prev_positions_2.Count; i++)
            {
                stoppos.x += f_prev_positions_2[i].x;
                stoppos.y += f_prev_positions_2[i].y;
                stoppos.z += f_prev_positions_2[i].z;
            }
            stoppos.x = stoppos.x / f_prev_positions_2.Count;
            stoppos.y = stoppos.y / f_prev_positions_2.Count;
            stoppos.z = stoppos.z / f_prev_positions_2.Count;

            stopposIM = ViveTracker.transform.localPosition;

            stopmaybe = true;
            pedaling = false;

        }

        float dist11 = Vector3.Distance(ViveTracker.transform.localPosition, stoppos);
        float distIM = Vector3.Distance(ViveTracker.transform.localPosition, stopposIM);
        //Debug.Log("Dist " + dist11 + " | distIM "+ distIM);

        if (distIM > 0.08f && stopmaybe)
        {

            if (dist11 > distIM)
            {
                pedaling = true;
                rigidBody.AddForce(backWC.transform.forward * 5f, ForceMode.Impulse);
                brake = false;

                stopmaybe = false;
            }
            else
            {
                pedaling = false;
                brake = true;
                stopposBR = ViveTracker.transform.localPosition;
                stopmaybe = false;
            }
        }

        if (brake)
        {
            float dist22 = Vector3.Distance(ViveTracker.transform.localPosition, stopposBR);

            if (dist22 > 0.075f)
            {
                pedaling = true;
                rigidBody.AddForce(backWC.transform.forward * 5f, ForceMode.Impulse);
                brake = false;

                stopmaybe = false;
            }
        }


        float stateChangeClock = gametime_clock - f_dirAngle_clock;
        #endregion

    }
    private void Physics_Speed()
    {
        //apply logic to alter speed depending on pedaling and brake
        if (brake == true)
        {
            //brakeCheck.SetActive(true);
            if (TrackerPedalValue > brakeTorque)
            {

                brakeTorque = TrackerPedalValue;
            }
            Acceleration(backWC, 0f);
            Brake(backWC, brakeTorque + c_brakeForce);
        }
        if (pedaling == true)
        {
            //brakeCheck.SetActive(false);
            brakeTorque = 0;
            Brake(backWC, brakeTorque);
            Acceleration(backWC, c_speedMultiplier * (TrackerPedalValue * (c_maxAcceleration - rigidBody.velocity.magnitude)));
        }
        else
        {
            Brake(backWC, brakeTorque);
            Acceleration(backWC, 0f);
        }

        float windBrake = (1.2f * 0.6f * Mathf.Pow(rigidBody.velocity.magnitude, 2f)) * 0.5f;
        rigidBody.AddForce(-windBrake * c_windBrake * backWC.transform.forward);

    }
    private void SteeringCalculation()
    {
        float steerAngle = ViveController.transform.localRotation.eulerAngles.y;

        Steering(frontWC, steerAngle);
        Steering(backWC, BicycleModel.transform.localEulerAngles.y); //apply steering to backwheel to keep wheelcollider in the same orientation as bicycle model

    }
    private void Visuals()
    {
        //Apply rotation to handlebar/front wheel model, for visual feedback of turning
        SteeringParent.transform.rotation = Quaternion.Euler(
            frontWheelStartRotation.x,
            (ViveController.transform.rotation.eulerAngles.y) + frontWheelStartRotation.y,
            frontWheelStartRotation.z); //Rotate handlebar


        //Rotate(spin-up) wheels in relation to speed
        frontWheel.transform.Rotate(Vector3.up, -rigidBody.velocity.magnitude);
        backWheel.transform.Rotate(Vector3.up, -rigidBody.velocity.magnitude);
    }
    private void TrackerRotationCalibration()
    {
        float currDistance = Vector3.Distance(ViveTracker.transform.localPosition, TrackerLookat.transform.localPosition);
        TrackerLookat.transform.position += TrackerLookat.transform.forward * (currDistance - c_PhysicalCrankRadius);
    }

    #region Variables for ViveTrackerTracker()
    private Vector3 prevTrackerPos = new Vector3(0, 0, 0);
    private float currAngle = 0;
    private float prevAngle = 0;
    private int trackerUpdate = 0;
    private int trackerUpdateRate = 5;
    private float trackerReturn = 0;
    #endregion
    private float ViveTrackerTracker()
    {
        trackerUpdate++;
        if (trackerUpdate >= trackerUpdateRate)
        {
            trackerUpdate = 0;

            Vector3 currTrackerPos = ViveTracker.transform.localPosition;

            //calculate angle between tracker and lookatObject (0-360)
            float y1 = currTrackerPos.y + 10f;
            float Cy = TrackerLookat.transform.localPosition.y + 10f;
            float z1 = currTrackerPos.z + 10f;
            float Cz = TrackerLookat.transform.localPosition.z + 10f;

            float radian = Mathf.Atan2(y1 - Cy, z1 - Cz);
            float angle = radian * (180f / Mathf.PI);
            if (angle < 0.0f)
                angle += 360.0f;

            currAngle = angle;

            //set the return value to be delta Angle.
            if ((currAngle - 180f) > prevAngle)
                trackerReturn = ((prevAngle + 360f) - currAngle);
            else
                trackerReturn = (prevAngle - currAngle);

            prevAngle = currAngle;
        }
        return trackerReturn;
    }
    #endregion

    #region  SetupMethods
    private void BicycleModelSetup()
    {
        //rotate entire bicycle so that the bicycle-forward is set to be the same direction as VIVE controller forward.
        BicycleModel.transform.rotation = Quaternion.Euler(
            0, 
            ViveController.transform.rotation.eulerAngles.y, 
            0);

        //hardcoded distance for handlebar position to be correct. Needs to be updated depending on the demensions of physical bicycle. 
        BicycleModel.transform.position = new Vector3(
            ViveController.transform.position.x,
            BicycleModel.transform.position.y,
            ViveController.transform.position.z) - BicycleModel.transform.forward * 0.4f;

        

    }
    #endregion

    #region Methods for wheelColliderPhysics
    private void Acceleration(WheelCollider wc, float power)
    {        
        wc.motorTorque = (power);
    }
    private void Steering(WheelCollider wc, float angle)
    {
        wc.steerAngle = angle;
    }
    private void Brake(WheelCollider wc, float brake_power)
    {
        wc.brakeTorque = brake_power * 3f;
    }
    #endregion
    
}
