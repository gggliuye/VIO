using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JuderTester : MonoBehaviour
{
    private SenderJudgerFeatures tSenderJudgerFeatures;
    // Start is called before the first frame update
    void Start()
    {
        tSenderJudgerFeatures = new SenderJudgerFeatures();
    }

    // Update is called once per frame
    void Update()
    {
        tSenderJudgerFeatures.ReadFeatures("Assets/StreamingAssets/cloud_sparse.ply", 1.0f);
    }
}
