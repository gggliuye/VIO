using GoogleARCore;
using GoogleARCoreInternal;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;

public class ServerScaleEstimator : MonoBehaviour
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

    // object to calculate camera pose
    public ScaleEstimator initCameraPose;

    public GameObject cameraParent;

    public bool sendAutomatic;

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
    }

    // Update is called once per frame
    void Update()
    {
        //PrintParameters();
    }

    void PrintParameters()
    {
        var nativeSession = LifecycleManager.Instance.NativeSession;
        var cameraHandle = nativeSession.FrameApi.AcquireCamera();
        CameraIntrinsics result =
            nativeSession.CameraApi.GetImageIntrinsics(cameraHandle);
        txt_camera.text = result.FocalLength + " " + result.PrincipalPoint;
        //Debug.Log(result.FocalLength);
        //Debug.Log(result.PrincipalPoint);
        //Debug.Log(Frame.CameraImage.Texture.texelSize);
        //Debug.Log(Frame.CameraImage.Texture.height + " " + Frame.CameraImage.Texture.width);
    }

    private bool isSending = false;
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
        }
        else
        {
            cameraParent.transform.localRotation = initCameraPose.convertMatrixRelative.GetQuaternion();
            cameraParent.transform.localPosition = initCameraPose.convertMatrixRelative.GetPosition();

            txt.text = count + " [SUCCESS]";
        }
        if (sendAutomatic)
        {
            IEnumerator coroutine = WaitAndSendImage(0.5f);
            StartCoroutine(coroutine);
        }
    }


    private IEnumerator WaitAndSendImage(float waitTime)
    {
        yield return new WaitForSeconds(waitTime);
        SendImage();
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
