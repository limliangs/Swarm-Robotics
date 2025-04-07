using UnityEngine;
using System.Collections.Generic;

public static class AccBound
{
    public static Vector3 ProjectAcceleration(Vector3 a, Vector3 v_t, float deltaT, float a_max, float v_max, float v_min, int numPoints)
    {
        Vector3 centre = -v_t / deltaT;
        float v_max_scaled = v_max / deltaT;
        float v_min_scaled = v_min / deltaT;
        float error = 0f;
        List<Vector3> candidates = new List<Vector3>();

        var c = a;
        Vector3 shifted = c - centre;
        if (c.magnitude <= a_max + error && shifted.magnitude <= v_max_scaled + error && shifted.magnitude >= v_min_scaled - error)
        {
            bool sameSign = Vector3.Dot(a, c) >= 0;
            //Debug.Log("ORIGI " + sameSign);
            return c;
        }

        if (a.magnitude > a_max)
        {
            c = a.normalized * a_max;
            shifted = c - centre;
            if (c.magnitude <= a_max + error && shifted.magnitude <= v_max_scaled && shifted.magnitude >= v_min_scaled)
            {
                bool sameSign = Vector3.Dot(a, c) >= 0;
                //Debug.Log("PROJ_A " + sameSign);
                return c;
            }
        }

        Vector3 shifted_a = a - centre;
        if (shifted_a.magnitude > v_max_scaled)
        {
            c = v_max_scaled * shifted_a.normalized + centre;
            shifted = c - centre;
            if (c.magnitude <= a_max + error && shifted.magnitude <= v_max_scaled + error && shifted.magnitude >= v_min_scaled - error)
            {
                bool sameSign = Vector3.Dot(a, c) >= 0;
                //Debug.Log("PROJ_B " + sameSign);
                return c;
            }
        }

        if (shifted_a.magnitude < v_min_scaled)
        {
            c = v_min_scaled * shifted_a.normalized + centre;
            shifted = c - centre;
            if (c.magnitude <= a_max + error && shifted.magnitude <= v_max_scaled + error && shifted.magnitude >= v_min_scaled - error)
            {
                bool sameSign = Vector3.Dot(a, c) >= 0;
                //Debug.Log("PROJ_C " + sameSign);
                return c;
            }
        }

        candidates.AddRange(ComputeRingIntersection(a_max, v_max_scaled, centre, numPoints));
        candidates.AddRange(ComputeRingIntersection(a_max, v_min_scaled, centre, numPoints));

        return FindBestCandidate(candidates, a, centre, a_max, v_max_scaled, v_min_scaled);
    }

    private static List<Vector3> ComputeRingIntersection(float r1, float r2, Vector3 center2, int numPoints)
    {
        List<Vector3> intersections = new List<Vector3>();
        float d = center2.magnitude;

        if (d == 0 || d > r1 + r2 || d < Mathf.Abs(r1 - r2))
            return intersections;

        float a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        float h = Mathf.Sqrt(r1 * r1 - a * a);

        Vector3 P0 = center2.normalized * a;
        Vector3 perp1 = Vector3.Cross(center2, Vector3.up);
        if (perp1.sqrMagnitude < 0.000001f)
            perp1 = Vector3.Cross(center2, Vector3.forward);
        perp1 = perp1.normalized * h;
        Vector3 perp2 = Vector3.Cross(center2, perp1).normalized * h;

        for (int i = 0; i < numPoints; i++)
        {
            float angle = (i / (float) numPoints) * 2f * Mathf.PI;
            intersections.Add(P0 + Mathf.Cos(angle) * perp1 + Mathf.Sin(angle) * perp2);
        }

        return intersections;
    }

    private static Vector3 FindBestCandidate(List<Vector3> candidates, Vector3 a, Vector3 centre, float a_max, float v_max_scaled, float v_min_scaled)
    {
        Vector3 best = candidates.Count > 0 ? candidates[0] : Vector3.zero;
        float minDist = float.MaxValue;
        float error = 0f;
        bool foundValidCandidate = false;

        int i = 0;
        int j = 0;
        foreach (var c in candidates)
        {
            Vector3 shifted = c - centre;
            bool sameSign = Vector3.Dot(a, c) >= 0;

            if (sameSign == true && c.magnitude <= a_max + error && shifted.magnitude <= v_max_scaled + error && shifted.magnitude >= v_min_scaled - error)
            {
                float dist = (c - a).sqrMagnitude;
                if (dist < minDist)
                {
                    minDist = dist;
                    best = c;
                    foundValidCandidate = true;
                    j = i;
                }
            }
            i++;
        }
        //Debug.Log("INTSC " + " found? " + foundValidCandidate + " @" + j);
        if (foundValidCandidate) { return best; }
        else { 
            Debug.Log("NOT FOUND");
            return Vector3.zero;
        }
    }
}