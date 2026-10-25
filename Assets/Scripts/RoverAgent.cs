using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[RequireComponent(typeof(CarControl))]
public class RoverAgent : Agent
{
    public Transform meetingPoint;
    public float arrivalRadius = 3f;   // how close counts as "at the meeting point"
    public float stopSpeed = 0.3f;     // below this speed counts as stopped
    public int stopDecisions = 30;     // decisions in a row it has to stay stopped
    public float maxSpeed = 15f;       // only used to scale the observations
    public float maxDistance = 50f;    // only used to scale the observations

    private CarControl car;
    private Rigidbody rb;
    private Vector3 startPos;
    private Quaternion startRot;
    private float prevDist;
    private int stoppedCount;

    public override void Initialize()
    {
        car = GetComponent<CarControl>();
        rb = GetComponent<Rigidbody>();
        car.externalControl = true; // the agent drives, not the keyboard
        startPos = transform.position;
        startRot = transform.rotation;
    }

    public override void OnEpisodeBegin()
    {
        transform.SetPositionAndRotation(startPos, startRot);
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        stoppedCount = 0;
        prevDist = FlatDistance();
    }

    private float FlatDistance()
    {
        Vector3 d = meetingPoint.position - transform.position;
        d.y = 0f;
        return d.magnitude;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Everything is relative to the truck itself, so it also works with random positions later
        Vector3 toTarget = meetingPoint.position - transform.position;
        toTarget.y = 0f;
        float dist = toTarget.magnitude;
        Vector3 localDir = transform.InverseTransformDirection(toTarget.normalized);
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);

        sensor.AddObservation(localDir.x);                            // 1: target left/right
        sensor.AddObservation(localDir.z);                            // 2: target ahead/behind
        sensor.AddObservation(Mathf.Clamp01(dist / maxDistance));     // 3: distance
        sensor.AddObservation(localVel.x / maxSpeed);                 // 4: sideways speed
        sensor.AddObservation(localVel.z / maxSpeed);                 // 5: forward speed
        sensor.AddObservation(rb.angularVelocity.y);                  // 6: turning rate
        sensor.AddObservation(Vector3.Dot(transform.up, Vector3.up)); // 7: how upright it is
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        car.externalSteer = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        car.externalThrottle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        float dist = FlatDistance();
        float speed = rb.linearVelocity.magnitude;

        // Reward getting closer, with a small penalty for every step
        AddReward((prevDist - dist) * 0.1f);
        AddReward(-0.001f);
        prevDist = dist;

        if (dist < arrivalRadius)
        {
            // Inside the meeting area: slow down and stay put
            AddReward(-speed * 0.002f);

            if (speed < stopSpeed) stoppedCount++;
            else stoppedCount = 0;

            if (stoppedCount >= stopDecisions)
            {
                AddReward(2f);
                EndEpisode();
                return;
            }
        }
        else
        {
            stoppedCount = 0;
        }

        // Failure: flipped over or fell off the map
        if (Vector3.Dot(transform.up, Vector3.up) < 0.3f || transform.position.y < startPos.y - 10f)
        {
            AddReward(-1f);
            EndEpisode();
        }
    }

    // Manual driving through the agent, for testing (Behavior Type = Heuristic Only)
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var a = actionsOut.ContinuousActions;
        a[0] = Input.GetKey(KeyCode.LeftArrow) ? -1f : Input.GetKey(KeyCode.RightArrow) ? 1f : 0f;
        a[1] = Input.GetKey(KeyCode.UpArrow) ? 1f : Input.GetKey(KeyCode.DownArrow) ? -1f : 0f;
    }
}