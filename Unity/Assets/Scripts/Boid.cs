using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Boid : MonoBehaviour {

    BoidSettings settings;

    // State
    [HideInInspector]
    public Vector3 position;
    [HideInInspector]
    public Vector3 forward;
    public Vector3 velocity;

    // To update:
    Vector3 acceleration;
    [HideInInspector]
    public Vector3 avgFlockHeading;
    [HideInInspector]
    public Vector3 avgAvoidanceHeading;
    [HideInInspector]
    public Vector3 centreOfFlockmates;
    [HideInInspector]
    public int numPerceivedFlockmates;
    [HideInInspector]
    public int numPerceivedSepmates;

    // Cached
    Material material;
    Transform cachedTransform;
    Transform target;

    void Awake()
    {
        material = transform.GetComponentInChildren<MeshRenderer>().material;
        cachedTransform = transform;
    }

    public void Initialize (BoidSettings settings, Transform target) {
        this.target = target;
        this.settings = settings;

        position = cachedTransform.position;
        forward = cachedTransform.forward;

        float startSpeed = (settings.minSpeed + settings.maxSpeed) / 2;
        velocity = transform.forward * startSpeed;
    }

    public void SetColour (Color col) {
        if (material != null) {
            material.color = col;
        }
    }
    
    public void UpdateBoid () {
        Vector3 acceleration = Vector3.zero;
        float dt = 1;
        float totalWeight = 0;
        List<float> magnitudes = new List<float> { 0, 0, 0, 0, 0 };

        if (target != null) {
            Vector3 offsetToTarget = (target.position - position);                      // local position
            acceleration = SteerTowards (offsetToTarget, dt) * settings.targetWeight;   // position to acceleration
            totalWeight += settings.targetWeight;
            magnitudes[0] = acceleration.magnitude;
        }

        if (numPerceivedFlockmates != 0) {
            avgFlockHeading /= numPerceivedFlockmates;      // velocity
            centreOfFlockmates /= numPerceivedFlockmates;   // position
            if (numPerceivedSepmates != 0)
            {
                avgAvoidanceHeading /= numPerceivedFlockmates;  // position (MAY BE ZERO if all flockmates offset > avoidRadius)
                Vector3 offsetToAvoidanceHeading = (avgAvoidanceHeading + position);

                Debug.Log("sep go to: " + offsetToAvoidanceHeading);
                var seperationForce = SteerTowards(avgAvoidanceHeading, dt) * settings.seperateWeight;    // position to acceleration
                acceleration += seperationForce;
                magnitudes[3] = seperationForce.magnitude;
                totalWeight += settings.seperateWeight;
            }
            

            Vector3 offsetToFlockmatesCentre = (centreOfFlockmates - position);
            var alignmentForce = (avgFlockHeading - velocity) / dt * settings.alignWeight;                  // velocity to acceleration
            var cohesionForce = SteerTowards (offsetToFlockmatesCentre, dt) * settings.cohesionWeight;      // position to acceleration

            acceleration += alignmentForce;
            acceleration += cohesionForce;
            
            totalWeight += (settings.alignWeight + settings.cohesionWeight);
            magnitudes[1] = alignmentForce.magnitude;
            magnitudes[2] = cohesionForce.magnitude;
        }

        if (IsHeadingForCollision ()) {
            Debug.Log("Collision Ahead");
            Vector3 collisionAvoidDir = ObstacleRays (); // goal position wrt self position
            Debug.Log("AltDir Found");
            Vector3 collisionAvoidForce = SteerTowards (collisionAvoidDir, dt) * settings.avoidCollisionWeight; // position to acceleration
            acceleration = collisionAvoidForce;
            totalWeight = settings.avoidCollisionWeight;
            magnitudes[4] = collisionAvoidForce.magnitude;
            magnitudes[0] = 0;
            magnitudes[1] = 0;
            magnitudes[2] = 0;
            magnitudes[3] = 0;
        }

        if (totalWeight != 0f)
        {
            acceleration /= totalWeight;
        }


        Vector3 boundedAcc = AccBound.ProjectAcceleration(acceleration, velocity, dt, settings.maxAccel, settings.maxSpeed, settings.minSpeed, settings.numPP);
        // bool sameSign = Vector3.Dot(acceleration, boundedAcc) >= 0;
        var positionN = position + velocity * dt + 0.5f * dt * dt * boundedAcc;
        bool inBoundN = !(Mathf.Abs(positionN.x) > 10 || Mathf.Abs(positionN.y - 4f) > 4 || Mathf.Abs(positionN.z) > 10);
        // bool isV = (velocity.magnitude < settings.maxSpeed + 0.001f);
        // bool isA = (boundedAcc.magnitude < settings.maxAccel + 0.001f);

        Debug.Log("s = " + position +
                  " v = " + velocity +
                  " a = " + acceleration +
                  " ~ " + boundedAcc +
                  // " isV " + isV + " isA " + isA +
                  " SepF = " + magnitudes[3] +
                  " ObsF = " + magnitudes[4] +
                   // " M = [" + string.Join(", ", magnitudes) + "]" +
                   // " aSameSign = " + sameSign +
                   " inBoundN = " + inBoundN
                  );

        cachedTransform.position += velocity * dt + 0.5f * dt * dt * boundedAcc;
        position = cachedTransform.position;




        velocity += boundedAcc * dt;

        if (velocity.magnitude != 0)
        {
            Vector3 dir = velocity.normalized;
            cachedTransform.forward = dir;
            forward = dir;
        }
    }

    bool IsHeadingForCollision () {
        RaycastHit hit;
        if (Physics.SphereCast (position, settings.boundsRadius, velocity, out hit, settings.collisionAvoidDst, settings.obstacleMask)) {
            return true;
        } else { }
        return false;
    }

    Vector3 ObstacleRays () {
        Vector3[] rayDirections = BoidHelper.directions;

        for (int i = 0; i < rayDirections.Length; i++) {
            Vector3 dir = cachedTransform.TransformDirection (rayDirections[i]);
            Ray ray = new Ray (position, dir);
            if (!Physics.SphereCast (ray, settings.boundsRadius, settings.collisionAvoidDst, settings.obstacleMask)) {
                // Debug.Log("AltLocDir = " + rayDirections[i] + " idx = " + i);
                return dir;
            }
        }
        return forward;
    }

    Vector3 SteerTowards (Vector3 offset, float dt) {
        Vector3 a = 2 * (offset - velocity * dt) / (dt * dt);
        return a;
    }

}