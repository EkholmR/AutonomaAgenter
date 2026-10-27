using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[RequireComponent(typeof(CarControl))]
public class RoverAgent : Agent
{
    
    public Transform meetingPoint;
    public Terrain terrain;               // used to find the spawn area and the ground height
    public TerrainGenerator generator;    // builds new hills every episode (can be left empty)
    public Transform[] buildings;         // always present, random position every episode
    public Transform[] rocks;             // a pool: a random number of them is used every episode
    public int minRocks = 1;
    public int maxRocks = 8;
    public float obstacleGap = 6f;        // minimum free space between two obstacles
    public float obstacleClearance = 8f;  // keep the truck and the target this far from obstacles
    public float edgeMargin = 10f;        // keep spawns this far from the terrain edge
    public float minSpawnDistance = 20f;  // minimum distance between start and meeting point
    public float arrivalRadius = 4f;      // how close counts as "at the meeting point"
    public float stopSpeed = 0.5f;        // below this speed counts as stopped
    public int stopDecisions = 15;        // decisions in a row it has to stay stopped
    public float maxSpeed = 15f;          // only used to scale the observations
     public bool demoMode = false;         // true: don't reset the scene when the rover has stopped
    private float maxDistance = 100f;     // set from the terrain size in Initialize
    private CarControl car;
    private Rigidbody rb;
    private float roverHeightOffset;
    private float targetHeightOffset;
    private float prevDist;
    private int stoppedCount;

    // Everything below is measured once in Initialize, with all obstacles switched on
    private Transform[] all;              // buildings first, then rocks
    private Collider[] colliders;
    private Vector2[] extents;            // half size of each footprint (x, z)
    private Vector2[] centerOffset;       // footprint center relative to the pivot (x, z)
    private float[] radii;
    private float[] heightOffsets;

    public override void Initialize()
    {
        car = GetComponent<CarControl>();
        rb = GetComponent<Rigidbody>();
        car.externalControl = true; // the agent drives, not the keyboard

        // Remember how high above the ground the truck and the meeting point were placed
        Vector3 p = transform.position;
        roverHeightOffset = p.y - GroundY(p.x, p.z);
        Vector3 m = meetingPoint.position;
        targetHeightOffset = m.y - GroundY(m.x, m.z);

        // Measure every obstacle while all of them are switched on
        foreach (Transform r in rocks) r.gameObject.SetActive(true);
        Physics.SyncTransforms();

        var list = new List<Transform>();
        list.AddRange(buildings);
        list.AddRange(rocks);
        all = list.ToArray();

        colliders = new Collider[all.Length];
        extents = new Vector2[all.Length];
        centerOffset = new Vector2[all.Length];
        radii = new float[all.Length];
        heightOffsets = new float[all.Length];

        for (int i = 0; i < all.Length; i++)
        {
            colliders[i] = all[i].GetComponentInChildren<Collider>(true);
            Vector3 pos = all[i].position;
            if (colliders[i] != null)
            {
                Bounds b = colliders[i].bounds;
                extents[i] = new Vector2(b.extents.x, b.extents.z);
                centerOffset[i] = new Vector2(b.center.x - pos.x, b.center.z - pos.z);
            }
            else
            {
                extents[i] = new Vector2(1f, 1f);
            }
            radii[i] = Mathf.Max(extents[i].x, extents[i].y);
            heightOffsets[i] = pos.y - LowestGround(i, pos.x, pos.z);
        }

        Vector3 size = terrain.terrainData.size;
        maxDistance = new Vector2(size.x, size.z).magnitude; // the diagonal, so the scaling is 0..1
    }

    private float GroundY(float x, float z)
    {
        return terrain.SampleHeight(new Vector3(x, 0f, z)) + terrain.transform.position.y;
    }

    // The lowest ground height under an obstacle's footprint if its pivot is at (px, pz)
    private float LowestGround(int i, float px, float pz)
    {
        float cx = px + centerOffset[i].x;
        float cz = pz + centerOffset[i].y;
        float low = float.MaxValue;
        for (int ix = 0; ix <= 2; ix++)
        {
            for (int iz = 0; iz <= 2; iz++)
            {
                float x = cx + Mathf.Lerp(-extents[i].x, extents[i].x, ix / 2f);
                float z = cz + Mathf.Lerp(-extents[i].y, extents[i].y, iz / 2f);
                low = Mathf.Min(low, GroundY(x, z));
            }
        }
        return low;
    }

    // Tilts the truck to match the slope under it, with a random heading
    private Quaternion GroundRotation(float x, float z, float yaw)
    {
        Vector3 o = terrain.transform.position;
        Vector3 s = terrain.terrainData.size;
        Vector3 normal = terrain.terrainData.GetInterpolatedNormal((x - o.x) / s.x, (z - o.z) / s.z);
        return Quaternion.FromToRotation(Vector3.up, normal) * Quaternion.Euler(0f, yaw, 0f);
    }

    private Vector3 RandomPoint(float margin)
    {
        Vector3 o = terrain.transform.position;
        Vector3 s = terrain.terrainData.size;
        float x = Random.Range(o.x + margin, o.x + s.x - margin);
        float z = Random.Range(o.z + margin, o.z + s.z - margin);
        return new Vector3(x, 0f, z);
    }

    // True if the point is clear of every active obstacle (compared on the ground plane)
    private bool IsClear(Vector3 p)
    {
        for (int i = 0; i < all.Length; i++)
        {
            if (!all[i].gameObject.activeInHierarchy || colliders[i] == null) continue;
            Bounds b = colliders[i].bounds;
            b.Expand(obstacleClearance * 2f);
            if (p.x > b.min.x && p.x < b.max.x && p.z > b.min.z && p.z < b.max.z) return false;
        }
        return true;
    }

    private Vector3 RandomClearPoint()
    {
        Vector3 p = RandomPoint(edgeMargin);
        for (int i = 0; i < 100 && !IsClear(p); i++) p = RandomPoint(edgeMargin);
        return p;
    }

    private bool InsideTerrain(Vector3 p)
    {
        Vector3 o = terrain.transform.position;
        Vector3 s = terrain.terrainData.size;
        return p.x > o.x && p.x < o.x + s.x && p.z > o.z && p.z < o.z + s.z;
    }

    private void PlaceObstacles()
    {
        // A random number of rocks is used this episode
        int rockCount = Mathf.Clamp(Random.Range(minRocks, maxRocks + 1), 0, rocks.Length);
        for (int i = 0; i < rocks.Length; i++) rocks[i].gameObject.SetActive(i < rockCount);

        var placed = new List<int>();
        for (int i = 0; i < all.Length; i++)
        {
            if (!all[i].gameObject.activeSelf) continue;

            // Find a spot that doesn't overlap the obstacles placed so far
            Vector3 pos = Vector3.zero;
            for (int t = 0; t < 100; t++)
            {
                pos = RandomPoint(edgeMargin + radii[i]);
                bool free = true;
                foreach (int j in placed)
                {
                    Vector2 a = new Vector2(pos.x, pos.z);
                    Vector2 b = new Vector2(all[j].position.x, all[j].position.z);
                    if (Vector2.Distance(a, b) < radii[i] + radii[j] + obstacleGap) { free = false; break; }
                }
                if (free) break;
            }

            all[i].position = new Vector3(pos.x, LowestGround(i, pos.x, pos.z) + heightOffsets[i], pos.z);
            placed.Add(i);
        }
        Physics.SyncTransforms(); // so the colliders are in place before the truck spawns
    }

    public override void OnEpisodeBegin()
    {
        // New hills first, then the obstacles, and last the truck and the target
        if (generator != null) generator.Generate();
        PlaceObstacles();

        // Pick a start and a target in free spots, and not too close to each other
        Vector3 start, target;
        int tries = 0;
        do
        {
            start = RandomClearPoint();
            target = RandomClearPoint();
            tries++;
        }
        while (Vector3.Distance(start, target) < minSpawnDistance && tries < 50);

        meetingPoint.position = new Vector3(target.x, GroundY(target.x, target.z) + targetHeightOffset, target.z);

        Quaternion rot = GroundRotation(start.x, start.z, Random.Range(0f, 360f));
        transform.SetPositionAndRotation(
            new Vector3(start.x, GroundY(start.x, start.z) + roverHeightOffset + 0.2f, start.z), rot);

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
        // Everything is relative to the truck itself, so it works from any start position
        Vector3 toTarget = meetingPoint.position - transform.position;
        toTarget.y = 0f;
        float dist = toTarget.magnitude;
        Vector3 localDir = transform.InverseTransformDirection(toTarget.normalized);
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);

        sensor.AddObservation(localDir.x);                             // 1: target left/right
        sensor.AddObservation(localDir.z);                             // 2: target ahead/behind
        sensor.AddObservation(Mathf.Clamp01(dist / maxDistance));      // 3: distance
        sensor.AddObservation(localVel.x / maxSpeed);                  // 4: sideways speed
        sensor.AddObservation(localVel.z / maxSpeed);                  // 5: forward speed
        sensor.AddObservation(rb.angularVelocity.y);                   // 6: turning rate
        sensor.AddObservation(Vector3.Dot(transform.up, Vector3.up));  // 7: how upright it is
        sensor.AddObservation(transform.forward.y);                    // 8: nose up (+) or down (-) a slope
        sensor.AddObservation(transform.right.y);                      // 9: leaning right (+) or left (-)
        // The obstacle rays come from the Ray Perception Sensor component, not from here
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        car.externalSteer = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        car.externalThrottle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        float dist = FlatDistance();
        float speed = rb.linearVelocity.magnitude;
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);

        // Reward getting closer, with time pressure
        AddReward((prevDist - dist) * 0.1f);
        prevDist = dist;
        AddReward(-0.001f);

        if (dist > arrivalRadius)
        {
            stoppedCount = 0;

            // Small reward for facing the target, so it turns instead of looping
            Vector3 toTarget = meetingPoint.position - transform.position;
            toTarget.y = 0f;
            Vector3 fwd = transform.forward;
            fwd.y = 0f;
            if (toTarget.sqrMagnitude > 0.01f && fwd.sqrMagnitude > 0.01f)
            {
                AddReward(Vector3.Dot(fwd.normalized, toTarget.normalized) * 0.002f);
            }

            // Small penalty for reversing
            if (localVel.z < 0f) AddReward(localVel.z * 0.004f);

            // Slow down on the approach: the closer it gets, the lower the allowed speed
            float slowRadius = 15f;
            if (dist < slowRadius)
            {
                float allowed = Mathf.Max(1f, 8f * dist / slowRadius);
                AddReward(-Mathf.Max(0f, speed - allowed) * 0.01f);
            }
        }
        else
        {
            // Being in the meeting area is rewarded, and speed is punished harder
            AddReward(0.005f);
            AddReward(-speed * 0.01f);

            if (speed < stopSpeed) stoppedCount++;
            else stoppedCount = 0;

            if (stoppedCount >= stopDecisions)
            {
                AddReward(2f);
                if (!demoMode) EndEpisode();   // in the demo the rover just stays parked
                return;
            }
        }

        // Failure: flipped over, fell off, or left the terrain
        Vector3 pos = transform.position;
        bool flipped = Vector3.Dot(transform.up, Vector3.up) < 0.3f;
        bool fell = pos.y < GroundY(pos.x, pos.z) - 10f;
        if (flipped || fell || !InsideTerrain(pos))
        {
            AddReward(-3f);
            EndEpisode();
        }
    }

    // Hitting a building or a rock ends the episode
    private void OnCollisionEnter(Collision collision)
    {
        Transform hit = collision.collider.transform;
        foreach (Transform t in all)
        {
            if (hit == t || hit.IsChildOf(t))
            {
                AddReward(-3f);
                EndEpisode();
                return;
            }
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