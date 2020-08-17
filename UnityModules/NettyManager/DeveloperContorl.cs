using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DeveloperContorl : MonoBehaviour
{
    //public InitCameraPose initCameraPose;
    //public GameObject ARscene;
    public GameObject points;
    public GameObject models;
    public GameObject occ;

    //public Slider slider;


    public void HidePoints()
    {
        if (points.activeSelf)
        {
            points.SetActive(false);
            models.SetActive(false);
            occ.SetActive(true);
        } else {
            points.SetActive(true);
            models.SetActive(true);
            occ.SetActive(false);
        }
    }

    void Update()
    {
        //initCameraPose.colmapScale = slider.value;
        //ARscene.transform.localScale = Vector3.one * (slider.value/2);
    }


}
