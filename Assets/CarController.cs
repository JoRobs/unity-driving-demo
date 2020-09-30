using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour {
    public List<AxleInfo> axleInfos; // the information about each individual axle
    public Rigidbody rb;
    public float maxMotorTorque; // maximum torque the motor can apply to wheel
    public float maxSteeringAngle; // maximum steer angle the wheel can have
    public float antiRollStiffness;
    public float downForceCoeff;
    public Vector3 downForceOffset;
    
    
    // finds the corresponding visual wheel
    // correctly applies the transform
    public void ApplyLocalPositionToVisuals(WheelCollider collider, Transform visWheel)
    {
          
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
     
        visWheel.position = position;
        visWheel.rotation = rotation;
    }

    public void ApplyDownForce(){
        float lift = -downForceCoeff * rb.velocity.sqrMagnitude;
        rb.AddForceAtPosition(lift * transform.up, transform.position);
    }

    public void ApplyAntiRoll(WheelCollider _wheelL, WheelCollider _wheelR){
        WheelHit hitL, hitR;
        float travelL, travelR;        
        bool groundedL = _wheelL.GetGroundHit(out hitL);
        bool groundedR = _wheelR.GetGroundHit(out hitR);
        
        if (groundedL)
            travelL = (-_wheelL.transform.InverseTransformPoint(hitL.point).y - _wheelL.radius) / _wheelL.suspensionDistance;
        else
            travelL = 1.0f;
        
        if (groundedR)
            travelR = (-_wheelR.transform.InverseTransformPoint(hitR.point).y - _wheelR.radius) / _wheelR.suspensionDistance;
        else
            travelR = 1.0f;

        float antiRollForce = (travelL-travelR)*antiRollStiffness;

        if (groundedL)
            rb.AddForceAtPosition(_wheelL.transform.up * -antiRollForce, _wheelL.transform.position);  
        if (groundedR)
            rb.AddForceAtPosition(_wheelR.transform.up * antiRollForce, _wheelR.transform.position);
    }

    public void FixedUpdate()
    {
        float motor = maxMotorTorque * Input.GetAxis("Vertical");
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");
            
        foreach (AxleInfo axleInfo in axleInfos) {
            if (axleInfo.steering) {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }

            ApplyLocalPositionToVisuals(axleInfo.leftWheel, axleInfo.leftWheelModel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel, axleInfo.rightWheelModel);
            ApplyAntiRoll(axleInfo.leftWheel, axleInfo.rightWheel);
        }
        ApplyDownForce();
        
    }

    
}


    
[System.Serializable]
public class AxleInfo {
    public WheelCollider leftWheel;
    public Transform leftWheelModel;
    public WheelCollider rightWheel;
    public Transform rightWheelModel;
    public bool motor; // is this wheel attached to motor?
    public bool steering; // does this wheel apply steer angle?
}
