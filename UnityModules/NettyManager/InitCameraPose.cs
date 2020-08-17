using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InitCameraPose : MonoBehaviour
{
    public GameObject slamCamera;

    public float colmapScale = 2.00F;

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

    public InitCameraPose()
    { 
        ifWaitingForResponse = false;
        systemInited = false;

        quaternionWhenSendImageSC = Quaternion.identity;
        positionWhenSendImageSC = new Vector3();
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
        if (!ifWaitingForResponse){
            return false;
        }


        if (cameraPoseInput[15] == 0){
            ifWaitingForResponse = false;
            return false;
        }

        convertMatrixWhenSendImagSC = new ConvertMatrix(quaternionWhenSendImageSC, positionWhenSendImageSC); 
        convertMatrixWhenSendImagSLAM = new ConvertMatrix(cameraPoseInput, true, colmapScale);

        //convertMatrixWhenSendImagSLAM.Inverse();

        UpdateRelativeTransform();

        systemInited = true;
        ifWaitingForResponse = false;

        return true;
    }

    private IEnumerator coroutine;
    private Quaternion quaternionRelativeFrom;
    private Quaternion quaternionRelativeTo;
    private Vector3 positionRelativeFrom;
    private Vector3 positionRelativeTo;

    private void UpdateRelativeTransform()
    {
        convertMatrixRelative = ConvertMatrix.LeftMultiply(convertMatrixWhenSendImagSC, convertMatrixWhenSendImagSLAM);
        convertMatrixRelative.Inverse();

        /*
        ConvertMatrix convertMatrixRelativeNew = ConvertMatrix.LeftMultiply(convertMatrixWhenSendImagSC, convertMatrixWhenSendImagSLAM);
        convertMatrixRelativeNew.Inverse();

        // direct assign if not initialized
        if (!systemInited)
        {
            convertMatrixRelative = convertMatrixRelativeNew;
            return;
        }

        Vector3 changed_position = (convertMatrixRelative.GetPosition() - convertMatrixRelativeNew.GetPosition());
        if(changed_position.magnitude > 1.0)
        {
            return;
        }

        // we choose to use smooth adjustment to offer better user experience
        // init variables for coroutine
        quaternionRelativeFrom = convertMatrixRelative.GetQuaternion();
        quaternionRelativeTo = convertMatrixRelativeNew.GetQuaternion();
        positionRelativeFrom = convertMatrixRelative.GetPosition();
        positionRelativeTo = convertMatrixRelativeNew.GetPosition();

        // stop the old coroutine if exist
        if (coroutine != null)
        {
            StopCoroutine(coroutine);
        }

        // start the new coroutine
        coroutine = WaitAndRotate(0.2f);
        StartCoroutine(coroutine);
        */
    }

    private IEnumerator WaitAndRotate(float waitTime)
    {
        for (int i = 0; i < 30; i++)
        {
            float t = i * 0.1f;
            Quaternion tmpQ = Quaternion.Lerp(quaternionRelativeFrom, quaternionRelativeTo, t);
            Vector3 tmpP = Vector3.Lerp(positionRelativeFrom, positionRelativeTo, t);
            convertMatrixRelative = new ConvertMatrix(tmpQ, tmpP);
            yield return new WaitForSeconds(waitTime);
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
