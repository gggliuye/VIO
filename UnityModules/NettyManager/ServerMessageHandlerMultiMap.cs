using GoogleARCore;
using GoogleARCoreInternal;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;

public class ServerMessageHandlerMultiMap : MonoBehaviour
{
    [DllImport("TextureDecoder")]
    private static extern void SetTextureFromUnity(IntPtr textureHandle);

    [DllImport("TextureDecoder")]
    private static extern void AR_SetNettyParameters(int a, int b, int c, int d, int port, double focus, int mapid, int devicetype);

    [DllImport("TextureDecoder")]
    private static extern void AR_InitNetty();

    [DllImport("TextureDecoder")]
    private static extern void AR_SetMapId(int mapid);

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
    public Text txt_camera;

    private int count = 0;
    // Start is called before the first frame update
    void Start()
    {
        PrintParameters();
    }

    // Update is called once per frame
    void Update()
    {
        //PrintParameters();
    }

    public void ChangeMapId()
    {
        string mapidstring = input_mapid.text;
        map_id = Convert.ToInt32(mapidstring);
        txt.text = "Netty " + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3] + ":" + ip[4] + ", map id : " + map_id;
    }

    public void ChangeIp()
    {
        string mapidstring = input_ip.text;
        string[] sArray = mapidstring.Split('.');
        if (sArray.Length >= 5)
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

    public void SendImage()
    {
        if (nettyinited)
        {
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
