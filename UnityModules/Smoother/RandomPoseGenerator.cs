using System.Collections;
using System.Collections.Generic;
using UnityEngine;


//[ExecuteInEditMode]
public class RandomPoseGenerator : MonoBehaviour
{

    public float seed_magnitude = 1.0f;
    public Vector3 mean_euler;
    public Vector3 mean_position;
    public bool bUseSmoother = false;

    private PoseSmoother tPoseSmoother;
    // Start is called before the first frame update
    void Start()
    {
        tPoseSmoother = new PoseSmoother();
    }


    // Update is called once per frame
    void Update()
    {
        mean_position = mean_position + new Vector3(0.01f, 0, 0);
        mean_euler = mean_euler + new Vector3(0.05f,0,0);

        float random_magnitude = Random.value * seed_magnitude;

        float noise_euler = 2 * random_magnitude;
        Vector3 noised_euler = mean_euler + new Vector3(Random.value * noise_euler, Random.value * noise_euler, Random.value * noise_euler);

        float noise_position = 0.1f * random_magnitude;
        Vector3 noised_position = mean_position + new Vector3(Random.value * noise_position, Random.value * noise_position, Random.value * noise_position);


        Quaternion rotation = Quaternion.Euler(noised_euler);
        Vector3 position = noised_position;

        if (bUseSmoother)
        {
            tPoseSmoother.UpdatePose(ref rotation, ref position, random_magnitude);
        }

        this.transform.localRotation = rotation;
        this.transform.localPosition = position;

    }

    public void SetRandomVariable()
    {

    }

    public void ResetState()
    {
        tPoseSmoother.ResetState();
    }
}
