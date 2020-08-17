using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Locus.ImageRecognition;

public class FileMove : MonoBehaviour
{
    [HideInInspector]
    public bool isComplete = false;
    

    private void Awake()
    {
        DatMove dat = new DatMove();
        // Android/data/包名/files
        StartCoroutine(dat.IEMoveDat(Application.streamingAssetsPath + "/Map.dat", () => isComplete = true));
    }
}
