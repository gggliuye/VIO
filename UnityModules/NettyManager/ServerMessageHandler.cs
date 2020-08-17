using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;

public class ServerMessageHandler : MonoBehaviour
{

    [DllImport("RenderingPlugin")]
    private static extern void AR_SetNettyParameters(int a, int b, int c, int d, int port, int mapid, int devicetype);

    [DllImport("RenderingPlugin")]
    private static extern void AR_InitNetty();

    [DllImport("RenderingPlugin")]
    private static extern void AR_SendImageNetty();

    // object to calculate camera pose
    public InitCameraPose initCameraPose;

    public GameObject cameraParent;

    private bool nettyinited = false;

    public int[] ip = new int[5];
    public int map_id;
    public int device_type;

    public Text txt;

    private int count = 0;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    public void SendImage()
    {
        if (nettyinited)
        {
            initCameraPose.RecordCureentPoseWhenSentImage();
            AR_SendImageNetty();
        }
        else
        {
            txt.text = "Init Netty " + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3] + ":" + ip[4];
            AR_SetNettyParameters(ip[0], ip[1], ip[2], ip[3], ip[4], map_id, device_type);
            AR_InitNetty();
            txt.text = "Init Done";
            nettyinited = true;
        }
    }

    void MessageCallback(string message)
    {
        float[] matrix = new float[16];
        matrix = GetFloatArrayFromJson(message);

        count = count + 1;

        if (!initCameraPose.ReceivePoseFromServer(matrix))
        {
            txt.text = count + "[LOST]";
            return;
        }

        cameraParent.transform.localRotation = initCameraPose.convertMatrixRelative.GetQuaternion();
        cameraParent.transform.localPosition = initCameraPose.convertMatrixRelative.GetPosition();

        txt.text = count + " [SUCCESS]";

    }

    /// <summary>
    /// decode json message to float array
    /// </summary>
    /// <param name="message"></param>
    /// <returns></returns>
    private float[] GetFloatArrayFromJson(string message)
    {
        PoseJson poseJson;

        try
        {
            poseJson = JsonUtility.FromJson<PoseJson>(message);
        }
        catch (Exception e)
        {
            LogManager.Singleton.PrintLog(e.Message, true);
            return null;
        }

        return poseJson.DATA_KEY;
    }

}
