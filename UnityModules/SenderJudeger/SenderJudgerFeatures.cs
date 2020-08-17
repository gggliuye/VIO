using Locus.ImageRecognition;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class SenderJudgerFeatures
{
    private bool InitDone = false;
    private int N_pts = 0;
    private List<Vector3> vFeatures;

    public SenderJudgerFeatures()
    {
        vFeatures = new List<Vector3>();
    }

    public SenderJudgerFeatures(string file_path_unity, float scale)
    {
        vFeatures = new List<Vector3>();
        InitSystem(file_path_unity, scale);
    }

    public int GetN()
    {
        return N_pts;
    }

    private int NCOLS = 32;
    private int NROWS = 24;
    public int CheckToSendImage(Quaternion q_wc, Vector3 p_wc, float focus, int cols, int rows)
    {
        if (!InitDone)
        {
            return -1;
        }

        Quaternion q_cw = Quaternion.Inverse(q_wc);
        Vector3 p_cw = -(q_cw* p_wc);

        // initialize the grid
        int num_grids = NCOLS * NROWS;
        bool[] mGrid = new bool[num_grids];
        for (int i =0;i < num_grids; i++)
        {
            mGrid[i] = false;
        }

        int s_rows = rows / NROWS;
        int s_cols = cols / NCOLS;
        // peoject points to camera view
        for (int i = 0; i < N_pts; i++)
        {
            Vector3 pt_c = q_cw * vFeatures[i] + p_cw;
            if(pt_c.z < 0.2)
            {
                continue;
            }
            float inv_z = 1 / pt_c.z;
            float u = pt_c.x * focus * inv_z + cols / 2;
            float v = pt_c.y * focus * inv_z + rows / 2;
            int grid_i = (int)(u / s_cols);
            int grid_j = (int)(v / s_rows);
            if(grid_i < 0 || grid_i > NCOLS)
            {
                continue;
            }
            if (grid_j < 0 || grid_j > NROWS)
            {
                continue;
            }
            int id = grid_i + grid_j * NCOLS;
            if(id < num_grids)
            {
                mGrid[id] = true;
            }
        }

        int count = 0;
        for (int i = 0; i < num_grids; i++)
        {
            if (mGrid[i])
            {
                count++;
            }
        }

        return count;
    }

    public string MapPath;
    public void InitSystem(string file_path_unity, float scale)
    {
        /*
        DatMove dat = new DatMove();
        Debug.Log("read cloud from : "+ file_path_unity);
        dat.IEMoveDat(file_path_unity, () =>
        {
            string MapPath = Application.persistentDataPath + "/" + file_path_unity;
            ReadFeatures(MapPath, scale);
            InitDone = true;
        });
        */
        MapPath = Application.persistentDataPath + "/" + file_path_unity;
        ReadFeatures(MapPath, scale);
        InitDone = true;
    }

    public void ReadFeatures(string file_path, float scale)
    {
        vFeatures.Clear();

        int interval = 5;

        string[] lines = File.ReadAllLines(file_path);
        if(lines == null || lines.Length <= 0)
        {
            return;
        }

        int inn = 0;
        for(int i = 20; i < lines.Length; i = i + interval)
        {
            string inp_ln = lines[i];
            string[] messageElements = inp_ln.Split(' ');
            //Debug.Log(messageElements[0] +" "+ messageElements[1] +" "+ messageElements[2]);
            float x = float.Parse(messageElements[0]) * scale;
            float y = -float.Parse(messageElements[1]) * scale;
            float z = float.Parse(messageElements[2]) * scale;

            vFeatures.Add(new Vector3(x,y,z));
            inn = inn + 1;
        }
        N_pts = inn;
        Debug.Log("There are " + inn + " points.");

    }

}
