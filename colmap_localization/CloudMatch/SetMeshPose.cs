using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[ExecuteInEditMode]
public class SetMeshPose : MonoBehaviour
{
    private float[,] relativePose = new float[4, 4];

    // Start is called before the first frame update
    public void InitPose()
    {

        float[] relativePoseArray = new float[16] {
0.44148853065346166f, -0.8969043435743231f, -0.025504426654194855f, 12.310358951183794f,
-0.10950857000724631f, -0.02564840744604299f, -0.993654932353477f, 1.7681489351641213f,
0.8905591595082349f, 0.4414801681147646f, -0.10954216313511526f, -5.588568650747423f,
0.0f, 0.0f, 0.0f, 1.0f,

        };

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                relativePose[i, j] = relativePoseArray[i * 4 + j];
                //Debug.Log(relativePose[i, j]);
            }
        }
        SetPose();
    }

    public void SetPose()
    {
        bool ifSlam = true;

        Vector3 position = new Vector3(relativePose[0, 3], relativePose[1, 3], relativePose[2, 3]);

        if (ifSlam)
        {
            position.y = -position.y;
        }
        //Debug.Log(position);

        // calculate quaternion from rotation matrix
        Quaternion quaternion = Quaternion.identity;
        quaternion.w = Mathf.Sqrt(1 + relativePose[0, 0] + relativePose[1, 1] + relativePose[2, 2]) / 2;
        quaternion.x = (relativePose[2, 1] - relativePose[1, 2]) / (4 * quaternion.w);
        quaternion.y = (relativePose[0, 2] - relativePose[2, 0]) / (4 * quaternion.w);
        quaternion.z = (relativePose[1, 0] - relativePose[0, 1]) / (4 * quaternion.w);

        if (ifSlam)
        {
            quaternion.x = -quaternion.x;
            quaternion.z = -quaternion.z;

        }

        this.transform.localPosition = position;
        this.transform.localRotation = quaternion;

        this.transform.localScale = new Vector3(-1,-1,1);
    }
}
