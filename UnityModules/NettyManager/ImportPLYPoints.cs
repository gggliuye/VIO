using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

[ExecuteInEditMode]
public class ImportPLYPoints : MonoBehaviour {

    public GameObject objPrefab;
    public float scale = 1.0F;
    public int interval = 1;
    public string filename = "FeaturePoints_optimized.ply";

    private List<GameObject> mapPoints;
    // Use this for initialization
    void Start () {
        mapPoints = new List<GameObject>();

    }

    public void ReadString()
    {
        string file_path = "Assets/Resources/"+ filename;

        Debug.Log("Start read file.");

        StreamReader inp_stm = new StreamReader(file_path);

        // skip the first 10 (20 to assure no bug)lines, which are ply file format infomations
        int inn = 0;
        while (inn < 20)
        {
            string inp_ln = inp_stm.ReadLine();
            inn = inn + 1;
        }

        inn = 0;
        while (!inp_stm.EndOfStream)
        {
            string inp_ln = inp_stm.ReadLine();
            string[] messageElements = inp_ln.Split(' ');
            for (int j = 0; j < interval; j++) {
                inp_ln = inp_stm.ReadLine();
            }
            //Debug.Log(messageElements[0] +" "+ messageElements[1] +" "+ messageElements[2]);
            float x = float.Parse(messageElements[0]) * scale;
            float y = - float.Parse(messageElements[1]) * scale;
            float z = float.Parse(messageElements[2]) * scale;
            // TODO： 找一个更效率的点云加载方式（比如particle system？）？
            GameObject instancePoint = Instantiate(objPrefab, new Vector3(0, 0, 0), Quaternion.identity);
            instancePoint.transform.SetParent(this.transform);
            instancePoint.transform.localPosition = new Vector3(x, y, z);
            mapPoints.Add(instancePoint);

            inn = inn + 1;
            
        }

        Debug.Log("There are " + inn + " points.");


        inp_stm.Close();

    }

    public void clearAll()
    {
        Debug.Log("Prepare to delete : "+mapPoints.Count+" points.");
        for (int i = 0; i < mapPoints.Count; i ++)
        {
            DestroyImmediate(mapPoints[i]);
        }
        foreach (Transform child in this.transform)
        {
            GameObject.DestroyImmediate(child.gameObject);
        }
    }

}
