using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ScaleEstimator : MonoBehaviour
{
    public GameObject slamCamera;

    public bool ifOptimizeScale = false;

    public Slider slider;
    public GameObject mesh;
    public GameObject points;

    public float colmapScale;
    private int poseCount;
    private float DistanceLocal;
    private float DistanceLocalTmp;
    private Vector3 lastLocalPose;
    private float DistanceGlobal;
    bool ifFirst = true;
    public Text txtScale;

    private bool ifWaitingForResponse;
    private bool systemInited;

    // the pose from shadow creator -> in its coordinate system
    private Quaternion quaternionWhenSendImageSC;
    private Vector3 positionWhenSendImageSC;
    private ConvertMatrix convertMatrixWhenSendImagSC;

    // pose from server ORBSLAM -> in world coordinate system
    private ConvertMatrix convertMatrixWhenSendImagSLAM;

    // pose transform for calcualte current pose
    private ConvertMatrix convertMatrixNow;

    // relative pose
    private int totalNumber = 0;
    [HideInInspector]
    public ConvertMatrix convertMatrixRelative;

    public ScaleEstimator()
    {
        ifWaitingForResponse = false;
        systemInited = false;
        colmapScale = 1.0F;
        poseCount = 0;
        DistanceLocal = 0;
        DistanceGlobal = 0;

        quaternionWhenSendImageSC = Quaternion.identity;
        positionWhenSendImageSC = new Vector3();
        lastLocalPose = new Vector3();
    }

    /// <summary>
    /// save the current pose of the camera when sent the image
    /// ATTENTION : should not send the next image unless we get the former response
    ///             other wise we will need to add timestamp gestion (also need timestamp in message)
    /// </summary>
    /// <returns></returns>
    public bool RecordCureentPoseWhenSentImage()
    {
        if (ifWaitingForResponse)
        {
            return false;
        }

        if (!ifFirst)
        {
            Vector3 tmp = slamCamera.transform.localPosition - lastLocalPose;
            DistanceLocalTmp = tmp.magnitude;
        }

        // call directly the camera pose. while it may cause unwanted effect.
        quaternionWhenSendImageSC = slamCamera.transform.localRotation;
        positionWhenSendImageSC = slamCamera.transform.localPosition;

        ifWaitingForResponse = true;
        return true;
    }

    public void ReceiveLostPoseFromServer()
    {
        ifWaitingForResponse = false;
    }

    /// <summary>
    /// need to be called after received the message to the tell the system pose of world coordinate
    /// so that we can calcualte the difference between ShadowCreator coordinate system and world coordinate system
    /// </summary>
    /// <param name="cameraPoseInput"></param>
    /// <returns></returns>
    public bool ReceivePoseFromServer(float[] cameraPoseInput)
    {
        if (!ifWaitingForResponse)
        {
            return false;
        }


        if (cameraPoseInput[15] == 0)
        {
            ifWaitingForResponse = false;
            return false;
        }

        Vector3 tmp = new Vector3(0,0,0);
        if (!ifFirst)
        {
            tmp = convertMatrixWhenSendImagSLAM.GetPosition();
        }

        convertMatrixWhenSendImagSC = new ConvertMatrix(quaternionWhenSendImageSC, positionWhenSendImageSC);
        convertMatrixWhenSendImagSLAM = new ConvertMatrix(cameraPoseInput, true, colmapScale);

        if (!ifFirst)
        {
            lastLocalPose = positionWhenSendImageSC;
            Vector3 dis = tmp - convertMatrixWhenSendImagSLAM.GetPosition();
            DistanceGlobal += dis.magnitude;
            DistanceLocal += DistanceLocalTmp;
            UpdateEstimatorScale();
        }
        else
        {
            lastLocalPose = positionWhenSendImageSC;
            ifFirst = false;
        }

        //convertMatrixWhenSendImagSLAM.Inverse();

        UpdateRelativeTransform();

        systemInited = true;
        ifWaitingForResponse = false;

        return true;
    }

    private void UpdateRelativeTransform()
    {
        convertMatrixRelative = ConvertMatrix.LeftMultiply(convertMatrixWhenSendImagSC, convertMatrixWhenSendImagSLAM);
        convertMatrixRelative.Inverse();
    }

    private void UpdateEstimatorScale()
    {
        if (ifOptimizeScale)
        {
            colmapScale = DistanceLocal / DistanceGlobal;
            txtScale.text = "Scale: " + colmapScale;
        }
    }

    private void Update()
    {
        //updateScaleFromSlider();
    }

    private void updateScaleFromSlider()
    {
        if (!ifOptimizeScale)
        {
            colmapScale = slider.value;
            txtScale.text = "Scale: " + colmapScale;
            mesh.transform.localScale = Vector3.one * colmapScale;
            points.transform.localScale = Vector3.one * colmapScale;
        }
    }

    /// <summary>
    /// poseOutput.Inverse = poseShadowCreator_t.Inverse * poseShadowCreator_old * poseSLAM_old.Inverse();
    /// </summary>
    /// <param name="quaternion"></param>
    /// <param name="position"></param>
    /// <param name="outputQuaternion"></param>
    /// <param name="outputPosition"></param>
    public bool CalculateCameraPoseInWorldCoordinate(ref Quaternion outputQuaternion, ref Vector3 outputPosition)
    {
        //modifyPose = "";
        if (!systemInited)
        {
            //Debug.Log("System Not initialized.");
            return false;
        }
        Quaternion quaternion = slamCamera.transform.localRotation;
        Vector3 position = slamCamera.transform.localPosition;

        // pose of the object with respect to the local coordinate system (shadow creator local coordinate system)
        convertMatrixNow = new ConvertMatrix(quaternion, position);

        ConvertMatrix convertMatrixOutput = ConvertMatrix.LeftMultiply(convertMatrixRelative, convertMatrixNow);

        outputQuaternion = convertMatrixOutput.GetQuaternion();
        outputPosition = convertMatrixOutput.GetPosition();

        return true;
    }

}
