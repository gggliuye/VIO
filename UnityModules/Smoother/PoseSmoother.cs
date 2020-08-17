using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PoseSmoother
{
    private Quaternion current_r;
    private Vector3 current_t;
    private float current_weight;
    // could change this parameter
    private int count = 0;
    private float time_factor = 0.9f;

    private bool is_first = true;

    public PoseSmoother()
    {
        current_weight = 1.0f;
        count = 0;
    }

    public void SetTimeFactor(float new_factor)
    {
        time_factor = new_factor;
    }

    public void UpdatePose(ref Quaternion in_r, ref Vector3 in_t, float in_weight)
    {
        count++;
        if (is_first)
        {
            current_r = in_r;
            current_t = in_t;
            current_weight = in_weight;
            is_first = false;
            return;
        }

        // check outlier if have enough observations
        if(count > 8)
        {
            // check distance
            Vector3 distance_t = current_t - in_t;
            if(distance_t.magnitude > 3.0f)
            {
                in_r = current_r;
                in_t = current_t;
                return;
            }

            // check rotation
            Vector3 distance_q = new Vector3(current_r.x-in_r.x, current_r.y - in_r.y, current_r.z - in_r.z);
            if (distance_q.magnitude > 0.5f)
            {
                in_r = current_r;
                in_t = current_t;
                return;
            }
            //Debug.Log(distance_q.magnitude + " " + distance_t.magnitude);
        }

        float ratio = in_weight / (in_weight + current_weight);
        Quaternion output_r = Quaternion.Lerp(current_r, in_r, ratio);
        Vector3 output_t = Vector3.Lerp(current_t, in_t, ratio);

        current_weight = time_factor * (current_weight + in_weight);

        current_r = output_r;
        current_t = output_t;
        in_r = output_r;
        in_t = output_t;
    }

    public void ResetState()
    {
        is_first = true;
    }



}
