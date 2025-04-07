using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoidManager : MonoBehaviour {

    const int threadGroupSize = 1024;

    public BoidSettings settings;
    public ComputeShader compute;
    Boid[] boids;

    void Start () {
        boids = FindObjectsByType<Boid>(FindObjectsSortMode.None);
        foreach (Boid b in boids) {
            b.Initialize (settings, null);
        }

        Time.fixedDeltaTime = 1.0f / settings.FPS;

    }

    void FixedUpdate () {
        if (boids != null) {

            int numBoids = boids.Length;
            var boidData = new BoidData[numBoids];

            for (int i = 0; i < boids.Length; i++) {
                boidData[i].position = boids[i].position;
                boidData[i].direction = boids[i].forward;
                boidData[i].velocity = boids[i].velocity;
            }

            var boidBuffer = new ComputeBuffer (numBoids, BoidData.Size);
            boidBuffer.SetData (boidData);

            compute.SetBuffer (0, "boids", boidBuffer);
            compute.SetInt ("numBoids", boids.Length);
            compute.SetFloat ("viewRadius", settings.perceptionRadius);
            compute.SetFloat ("avoidRadius", settings.avoidanceRadius);
            compute.SetFloat("safeRadius", settings.safeRadius);

            int threadGroups = Mathf.CeilToInt (numBoids / (float) threadGroupSize);

            compute.Dispatch (0, threadGroups, 1, 1);

            boidBuffer.GetData (boidData);

            for (int i = 0; i < boids.Length; i++) {
                boids[i].avgFlockHeading = boidData[i].flockHeading;
                boids[i].centreOfFlockmates = boidData[i].flockCentre;
                boids[i].avgAvoidanceHeading = boidData[i].separationHeading;
                boids[i].numPerceivedFlockmates = boidData[i].numFlockmates;
                boids[i].numPerceivedSepmates = boidData[i].numSepmates;

                boids[i].UpdateBoid ();
            }

            boidBuffer.Release ();
        }
    }

    public struct BoidData {
        public Vector3 position;
        public Vector3 direction;
        public Vector3 velocity;


        public Vector3 flockHeading;
        public Vector3 flockCentre;
        public Vector3 separationHeading;
        public int numFlockmates;
        public int numSepmates;

        public static int Size {
            get {
                return sizeof (float) * 3 * 6 + sizeof (int) * 2;
            }
        }
    }
}