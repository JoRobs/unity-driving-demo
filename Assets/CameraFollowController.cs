using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollowController : MonoBehaviour
{
    public Transform objectToFollow;
    public float followSpeed = 10;
    public float lookSpeed = 10;
    public Vector3 offset;
    
    void lookAtTarget(){
        Vector3 _lookDirection = objectToFollow.position - transform.position; 
        Quaternion _quat = Quaternion.LookRotation(_lookDirection, Vector3.up);
        transform.rotation = Quaternion.Lerp(transform.rotation, _quat, lookSpeed);
    }

    void followTarget(){
        Vector3 _targetPosition = objectToFollow.position + 
                                  objectToFollow.up * offset.y +
                                  objectToFollow.right * offset.x +
                                  objectToFollow.forward * offset.z;

        transform.position = Vector3.Lerp(transform.position, _targetPosition, followSpeed);                          
    }

    void FixedUpdate()
    {
        lookAtTarget();
        followTarget();
    }
}
