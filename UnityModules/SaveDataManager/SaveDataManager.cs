using GoogleARCore;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;


/*
 *  std::string runtimeFileName = "/storage/emulated/0/SLAM/trajdata/Trajectory.txt";
 *  std::string imagePath = "/storage/emulated/0/SLAM/trajdata/images/";
 */

public class SaveDataManager : MonoBehaviour
{
    [DllImport("SaveDataManager")]
    private static extern void SaveData_InitDataSaver();

    [DllImport("SaveDataManager")]
    private static extern void SaveData_SavePose(float px, float py, float pz,
        float qw, float qx, float qy, float qz, bool flag, int key);

    [DllImport("SaveDataManager")]
    private static extern void SaveData_SaveImage(byte[] data, int size, int key);

    [DllImport("SaveDataManager")]
    private static extern void SaveData_CreateNewPointCloudFile(int key);

    [DllImport("SaveDataManager")]
    private static extern void SaveData_SavePoint(float px, float py, float pz, float confidence, int key);

    public GameObject ArCoreFirstViewCamera;

    public Text debug_txt;

    public int interval = 1;
    private int counter = 0;
    private int save_key = 0;

    private bool manager_inited = false;

    // Start is called before the first frame update
    void Start()
    {
        debug_txt.text = "Wait for start.";
    }

    // Update is called once per frame
    void Update()
    {
        if (manager_inited)
        {
            SendImage();
        }
    }

    public void StartRecord()
    {
        SaveData_InitDataSaver();
        manager_inited = true;
    }

    public void ReSetRecord()
    {
        SaveData_InitDataSaver();
        manager_inited = true;
        save_key = 0;
    }

    public void StopRecord()
    {
        debug_txt.text = "Stop Record Images.";
        manager_inited = false;
    }

    public void SendImage()
    {
        counter = counter + 1;
        if (manager_inited && counter%interval == 0)
        {
            counter = 0;

            //SetTextureFromUnity(Frame.CameraImage.Texture.GetNativeTexturePtr());
            // "using" will automaticly release the memory of the object
            using (GoogleARCore.CameraImageBytes image = Frame.CameraImage.AcquireCameraImageBytes())
            {
                if (!image.IsAvailable)
                {
                    debug_txt.text = "Image Not available.";
                    return;
                }

                // Move raw data into managed buffer.
                int bufferSize = image.YRowStride * image.Height;
                byte[] s_ImageBuffer = new byte[bufferSize];

                // Move raw data into managed buffer.
                System.Runtime.InteropServices.Marshal.Copy(image.Y, s_ImageBuffer, 0, bufferSize);

                Quaternion q_camera = ArCoreFirstViewCamera.transform.localRotation;
                Vector3 p_camera = ArCoreFirstViewCamera.transform.localPosition;

                save_key = save_key + 1;
                SaveData_SavePose(p_camera.x, p_camera.y, p_camera.z, q_camera.w, q_camera.x, q_camera.y, q_camera.z, true, save_key);
                SaveData_SaveImage(s_ImageBuffer, bufferSize, save_key);
                SavePointCloud(save_key);
                debug_txt.text = "Saved image " + save_key;
            }
        }
    }

    private void SavePointCloud(int key)
    {
        if (Frame.PointCloud.IsUpdatedThisFrame)
        {
            SaveData_CreateNewPointCloudFile(key);
            for (int i = 0; i < Frame.PointCloud.PointCount; i++)
            {
                PointCloudPoint point = Frame.PointCloud.GetPointAsStruct(i);
                if (point.Id > 0)
                {
                    SaveData_SavePoint(point.Position.x, point.Position.y, point.Position.z, point.Confidence, key);
                }
            }
        }
    }


}
