using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelManager : MonoBehaviour
{
    public List<WheelCollider> wheels;
    /*
    public float spring;
    public float damper;
    public float targetPosition;
    */
    public float wheelDampingRate = 0.25f;
    public float radius = 0.5f;
    public float mass = 20f;
    public float suspensionDistance = 0.3f;

    // Start is called before the first frame update
    void Start()
    {
        foreach (WheelCollider wheel in wheels){
            wheel.mass = mass;
            wheel.radius = radius;
            wheel.wheelDampingRate = wheelDampingRate;
            wheel.suspensionDistance = suspensionDistance;

        }
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
