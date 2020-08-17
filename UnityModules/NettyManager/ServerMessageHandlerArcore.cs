using GoogleARCore;
using GoogleARCoreInternal;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;

public class ServerMessageHandlerArcore : MonoBehaviour
{

    [DllImport("TextureDecoder")]
    private static extern void SetTextureFromUnity(IntPtr textureHandle);

    [DllImport("TextureDecoder")]
    private static extern void AR_SetNettyParameters(int a, int b, int c, int d, int port, double focus, int mapid, int devicetype);

    [DllImport("TextureDecoder")]
    private static extern void AR_InitNetty();

    [DllImport("TextureDecoder")]
    private static extern void AR_SendImageNetty(byte[] data, int size);

    //[DllImport("TextureDecoder")]
    //private static extern void AR_SendImageNettyOpenGL();

    //public Material BackgroundMaterial;

    public InputField input_mapid;
    public InputField input_ip;

    // object to calculate camera pose
    public InitCameraPose initCameraPose;

    public GameObject cameraParent;

    private bool nettyinited = false;

    public int[] ip = new int[5];
    public double focus;
    public int map_id;
    public int device_type;

    public Text txt;
    //public Text txt_camera;

    private int count = 0;
    private PoseSmoother tPoseSmoother;
    private SenderJudgerFeatures tSenderJudgerFeatures;
    public Text txt_sender;
    // Start is called before the first frame update
    void Start()
    {
        PrintParameters();
        tPoseSmoother = new PoseSmoother();
        tSenderJudgerFeatures = new SenderJudgerFeatures("cloud_sparse.ply", initCameraPose.colmapScale);
    }

    // Update is called once per frame
    void Update()
    {
        SendCheck();
        //PrintParameters();
    }

    private bool SendCheck()
    {
        Quaternion q_camera = cameraParent.transform.localRotation * initCameraPose.slamCamera.transform.localRotation;
        Vector3 p_camera = cameraParent.transform.localRotation * initCameraPose.slamCamera.transform.localPosition
            + cameraParent.transform.localPosition;

        int count = tSenderJudgerFeatures.CheckToSendImage(q_camera, p_camera, (float)focus, 640, 480);
        txt_sender.text = count + " "+ tSenderJudgerFeatures.GetN() + " points in map.";
        return true;
    }


    public void ChangeMapId()
    {
        string mapidstring = input_mapid.text;
        map_id = Convert.ToInt32(mapidstring);
        txt.text = "Netty " + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3] + ":" + ip[4] + ", map id : " + map_id;
    }

    public Slider slider_time_factor;
    public void SetTimeFactor()
    {
        tPoseSmoother.SetTimeFactor(slider_time_factor.value);
    }

    public void ChangeIp()
    {
        string mapidstring = input_ip.text;
        string[] sArray = mapidstring.Split('.');
        if(sArray.Length >= 5)
        {
            for (int i = 0; i < 5; i++)
            {
                ip[i] = Convert.ToInt32(sArray[i]);
            }
            //map_id = Convert.ToInt32(mapidstring);
            txt.text = "Netty " + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3] + ":" + ip[4] + ", map id : " + map_id;
        }
    }

    void PrintParameters()
    {
        var nativeSession = LifecycleManager.Instance.NativeSession;
        var cameraHandle = nativeSession.FrameApi.AcquireCamera();
        CameraIntrinsics result =
            nativeSession.CameraApi.GetImageIntrinsics(cameraHandle);
        txt.text = result.FocalLength + " " + result.PrincipalPoint;
        focus = result.FocalLength.x;
        //Debug.Log(result.FocalLength);
        //Debug.Log(result.PrincipalPoint);
        //Debug.Log(Frame.CameraImage.Texture.texelSize);
        //Debug.Log(Frame.CameraImage.Texture.height + " " + Frame.CameraImage.Texture.width);
    }


    private bool is_waiting = false;
    public void SendImage()
    {
        if (nettyinited)
        {
            if (is_waiting)
            {
                txt.text = "Waiting for result";
                return;
            }
            //SetTextureFromUnity(Frame.CameraImage.Texture.GetNativeTexturePtr());
            // "using" will automaticly release the memory of the object
            using (GoogleARCore.CameraImageBytes image = Frame.CameraImage.AcquireCameraImageBytes())
            {
                if (!image.IsAvailable)
                {
                    txt.text = "Image Not available.";
                    return;
                }

                // Move raw data into managed buffer.
                int bufferSize = image.YRowStride * image.Height;
                byte[] s_ImageBuffer = new byte[bufferSize];

                // Move raw data into managed buffer.
                System.Runtime.InteropServices.Marshal.Copy(image.Y, s_ImageBuffer, 0, bufferSize);
                initCameraPose.RecordCureentPoseWhenSentImage();

                AR_SendImageNetty(s_ImageBuffer, bufferSize);
                txt.text = "Sent image " + bufferSize;
                is_waiting = true;
            }
        }
        else
        {
            SetTextureFromUnity(Frame.CameraImage.Texture.GetNativeTexturePtr());
            txt.text = "Init Netty " + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3] + ":" + ip[4];
            AR_InitNetty();
            AR_SetNettyParameters(ip[0], ip[1], ip[2], ip[3], ip[4], focus, map_id, device_type);
            txt.text = "Init Done";
            nettyinited = true;
        }
    }

    public void SendImageCallFromScript()
    {
        if (nettyinited)
        {
            if (is_waiting)
            {
                txt.text = "Waiting for result";
                return;
            }
            //SetTextureFromUnity(Frame.CameraImage.Texture.GetNativeTexturePtr());
            // "using" will automaticly release the memory of the object
            using (GoogleARCore.CameraImageBytes image = Frame.CameraImage.AcquireCameraImageBytes())
            {
                if (!image.IsAvailable)
                {
                    txt.text = "Image Not available.";
                    return;
                }

                // Move raw data into managed buffer.
                int bufferSize = image.YRowStride * image.Height;
                byte[] s_ImageBuffer = new byte[bufferSize];

                // Move raw data into managed buffer.
                System.Runtime.InteropServices.Marshal.Copy(image.Y, s_ImageBuffer, 0, bufferSize);
                initCameraPose.RecordCureentPoseWhenSentImage();

                AR_SendImageNetty(s_ImageBuffer, bufferSize);
                txt.text = "Sent image " + bufferSize;
                is_waiting = true;
            }
        }
    }
    [HideInInspector]
    public bool bHaveLocalized = false;
    void MessageCallback(string message)
    {
        float[] matrix = new float[16];
        matrix = GetFloatArrayFromJson(message);

        count = count + 1;

        if (!initCameraPose.ReceivePoseFromServer(matrix))
        {
            txt.text = count + "[LOST]";
            is_waiting = false;
            return;
        }

        Quaternion rotation = initCameraPose.convertMatrixRelative.GetQuaternion();
        Vector3 position = initCameraPose.convertMatrixRelative.GetPosition();

        tPoseSmoother.UpdatePose(ref rotation, ref position, 1.0f);

        cameraParent.transform.localRotation = rotation;
        cameraParent.transform.localPosition = position;

        txt.text = count + " [SUCCESS]";
        is_waiting = false;
        bHaveLocalized = true;
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
