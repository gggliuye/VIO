using Locus.ImageRecognition;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;


/*
 *  std::string runtimeFileName = "/storage/emulated/0/SLAM/MapAnalysis/Map.dat";
 */

public class MapSenderJudger : MonoBehaviour
{
    [DllImport("SaveDataManager")]
    private static extern bool MapSenderManager_Init(string map_path, int check_range);

    [DllImport("SaveDataManager")]
    private static extern bool MapSenderManager_CheckFrame(float px, float py, float pz,
        float qw, float qx, float qy, float qz);

    [DllImport("SaveDataManager")]
    private static extern bool MapSenderManager_CheckFrameAndDraw(float px, float py, float pz,
        float qw, float qx, float qy, float qz);

    public GameObject ArCoreCamera;
    public GameObject ArCoreCameraParent;
    public ServerMessageHandlerArcore pServerMessageHandlerArcore;
    public Text debug_txt;

    private bool InitDone = false;
    public int checkInteval = 10;
    private int checkInterval_use = 10;

    private bool isComplete = false;
    // Start is called before the first frame update
    void Start()
    {
        DatMove dat = new DatMove();
        // Android/data/包名/files
        StartCoroutine(dat.IEMoveDat("Map.dat", () => {
            isComplete = true;
            string MapPath = Application.persistentDataPath + "/Map.dat";
            int check_range = 2;
            InitDone = MapSenderManager_Init(MapPath, check_range);
            checkInterval_use = checkInteval;
        }));
    }

    private int count = 0;
    // Update is called once per frame
    void Update()
    {
        if (!isComplete)
        {
            return;
        }
        count++;
        if (pServerMessageHandlerArcore.bHaveLocalized)
        {
            checkInterval_use = checkInteval * 5;
        }
        if (InitDone && (count% checkInterval_use) ==0)
        {
            Quaternion q_camera = ArCoreCameraParent.transform.localRotation * ArCoreCamera.transform.localRotation;
            Vector3 p_camera = ArCoreCameraParent.transform.localRotation * ArCoreCamera.transform.localPosition
                + ArCoreCameraParent.transform.localPosition;
            // send the transformed pose to native
            MapSenderManager_CheckFrameAndDraw(p_camera.x, -p_camera.y, p_camera.z, q_camera.w, -q_camera.x, q_camera.y, -q_camera.z);
            
            if(MapSenderManager_CheckFrame(p_camera.x, -p_camera.y, p_camera.z, q_camera.w, -q_camera.x, q_camera.y, -q_camera.z))
            {
                pServerMessageHandlerArcore.SendImageCallFromScript();
                debug_txt.text = "Will Send Image";
            }
            else
            {
                debug_txt.text = "Won't Send Image";
            }
            count = 0;
        }
    }
}
