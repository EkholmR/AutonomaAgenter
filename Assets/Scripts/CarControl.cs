using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarControl : MonoBehaviour
{
    public float enginePower = 2000.0f; // Engine power
    public float turnSpeed = 25.0f; // Maximum turn speed
    public float turnSmoothness = 5.0f; // Turn smoothness
    public Transform[] wheels; // Array of Wheel Collider components
    public Transform[] wheelMeshes; // Array of wheel meshes
    public Transform centerOfMass; // Center of Mass point
    public GameObject steeringWheel;

    [Header("Agent control (set by RoverAgent)")]
    public bool externalControl = false;
    public float externalSteer = 0f;     // -1 (left) to 1 (right)
    public float externalThrottle = 0f;  // -1 (reverse) to 1 (forward)
    public float brakeTorque = 3000f;    // applied when the agent gives no throttle

    private Rigidbody rb; // Car's Rigidbody component
    private float currentTurnAngle = 0.0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.centerOfMass = centerOfMass.localPosition; // Set center of mass
    }

    void FixedUpdate()
    {
        float horizontalInput = 0.0f;

        if (Input.GetKey(KeyCode.LeftArrow))
        {
            horizontalInput = -1.0f;
        }

        else if (Input.GetKey(KeyCode.RightArrow))
        {
            horizontalInput = 1.0f;
        }

        float throttle = 0.0f;

        if (Input.GetKey(KeyCode.UpArrow))
        {
            throttle = 1.0f;
        }

        else if (Input.GetKey(KeyCode.DownArrow))
        {
            throttle = -1.0f;
        }

        // The agent overrides the keyboard
        if (externalControl)
        {
            horizontalInput = externalSteer;
            throttle = externalThrottle;
        }

        // Brake only in agent mode, and only when no throttle is given
        float brake = (externalControl && Mathf.Abs(throttle) < 0.05f) ? brakeTorque : 0f;

        // Calculate turn angle
        float targetTurnAngle = horizontalInput * turnSpeed;
        currentTurnAngle = Mathf.Lerp(currentTurnAngle, targetTurnAngle, Time.deltaTime * turnSmoothness);

        steeringWheel.transform.localEulerAngles = new Vector3(-64, 0, currentTurnAngle * 3);

        // Wheels rotation and steering
        for (int i = 0; i < wheels.Length; i++)
        {
            WheelCollider wheelCollider = wheels[i].GetComponent<WheelCollider>();
            if (i < 2) // First two wheels steer
            {
                wheelCollider.steerAngle = currentTurnAngle;
            }
            else
            {
                wheelCollider.steerAngle = 0f; // Other two wheels remain straight
            }

            // Wheels driving (forward/backward movement)
            wheelCollider.motorTorque = throttle * enginePower;
            wheelCollider.brakeTorque = brake;

            // Rotate wheel meshes
            Quaternion wheelRotation;
            Vector3 wheelPosition;
            wheelCollider.GetWorldPose(out wheelPosition, out wheelRotation);
            wheelMeshes[i].position = wheelPosition;
            wheelMeshes[i].rotation = wheelRotation;
        }
    }
}