using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConvertMatrix
{
    public float[,] cameraPose = new float[4,4];
    public float[] cameraArray;
    public Quaternion quaternion;
    public Vector3 position;

    public ConvertMatrix(Quaternion quaternionN, Vector3 positionN)
    {
        quaternion = quaternionN;
        position = positionN;
        cameraArray = TransformToMatrixArray(quaternionN, positionN);
        TransformArrayToMatrix(cameraArray);
    }

    public ConvertMatrix(float[] cameraPoseInput, bool ifSlam = true, float scale = 1.0f)
    {
        if (cameraPoseInput.Length != 16)
        {
            Debug.Log("ERROR ! Input length not equal to 16.");
            return;
        }
        cameraArray = cameraPoseInput;

        TransformArrayToMatrix(cameraPoseInput);

        position = new Vector3(cameraPose[0,3] * scale, cameraPose[1,3] * scale, cameraPose[2,3] * scale);
        if(ifSlam)
        {
            position.y = -position.y;
        }

        // calculate quaternion from rotation matrix
        quaternion = Quaternion.identity;
        quaternion.w = Mathf.Sqrt(1 + cameraPose[0,0] + cameraPose[1,1] + cameraPose[2,2]) / 2;
        quaternion.x = (cameraPose[2,1] - cameraPose[1,2]) / (4 * quaternion.w);
        quaternion.y = (cameraPose[0,2] - cameraPose[2,0]) / (4 * quaternion.w);
        quaternion.z = (cameraPose[1,0] - cameraPose[0,1]) / (4 * quaternion.w);

        if(ifSlam)
        {
            quaternion.x = -quaternion.x;
            quaternion.z = -quaternion.z;
            
        }
    }

    public void TransformArrayToMatrix(float[] cameraPoseArray)
    {
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                cameraPose[i, j] = cameraPoseArray[j * 4 + i];
            }
        }
    }

    static public float[] TransformMatrixToArray(float[,] cameraPoseMat)
    {
        float[] output = new float[16];
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                output[j * 4 + i] = cameraPoseMat[i, j];
            }
        }
        return output;
    }

    public float[] TransformToMatrixArray(Quaternion quaternion, Vector3 position)
    {
        float[] outputMatrix = new float[16];

        outputMatrix[0] = 1 - 2 * quaternion.y * quaternion.y - 2 * quaternion.z * quaternion.z;
        outputMatrix[1] = 2 * quaternion.x * quaternion.y + 2 * quaternion.z * quaternion.w;
        outputMatrix[2] = 2 * quaternion.x * quaternion.z - 2 * quaternion.y * quaternion.w;
        outputMatrix[3] = 0.0F;

        outputMatrix[4] = 2 * quaternion.x * quaternion.y - 2 * quaternion.z * quaternion.w;
        outputMatrix[5] = 1 - 2 * quaternion.x * quaternion.x - 2 * quaternion.z * quaternion.z;
        outputMatrix[6] = 2 * quaternion.z * quaternion.y + 2 * quaternion.x * quaternion.w;
        outputMatrix[7] = 0.0F;

        outputMatrix[8] = 2 * quaternion.x * quaternion.z + 2 * quaternion.y * quaternion.w;
        outputMatrix[9] = 2 * quaternion.z * quaternion.y - 2 * quaternion.x * quaternion.w;
        outputMatrix[10] = 1 - 2 * quaternion.x * quaternion.x - 2 * quaternion.y * quaternion.y;
        outputMatrix[11] = 0.0F;

        outputMatrix[12] = position.x;
        outputMatrix[13] = position.y;
        outputMatrix[14] = position.z;
        outputMatrix[15] = 1.1F;

        return outputMatrix;
    }

    public void Inverse()
    {
        quaternion = Quaternion.Inverse(quaternion);
        position = - (quaternion * position);
        cameraArray = TransformToMatrixArray(quaternion, position);
        TransformArrayToMatrix(cameraArray);
    }
    
    static public ConvertMatrix LeftMultiply(ConvertMatrix convertMatrix1, ConvertMatrix convertMatrix2)
    {
        
        // calculate with quaternion and position R1*(R2*p+t2)+t1 -> R1*R2*p + R1*t2 + t1
        Quaternion quaternionN = convertMatrix1.quaternion * convertMatrix2.quaternion;
        Vector3 positionN = convertMatrix1.quaternion * convertMatrix2.position + convertMatrix1.position;
        return new ConvertMatrix(quaternionN, positionN);
        /*
        // calculate by general matrix multiply
        float[,] mat = MultiplyMatrix(convertMatrix1.cameraPose, convertMatrix2.cameraPose);
        return new ConvertMatrix(TransformMatrixToArray(mat), false);
        */
    }
    /*
    // if choose to use average method. TODO: find the proper way to average
    convertMatrixRelative = ConvertMatrix.AverageWithFactor(convertMatrixRelative, totalNumber, newconvertMatrixRelative, 1);
    */
    static public ConvertMatrix AverageWithFactor(ConvertMatrix convertMatrix1, float factor1, ConvertMatrix convertMatrix2, float factor2)
    {
        float factorSum = factor1 + factor2;
        //Vector3 newAngles = (factor1 * convertMatrix1.quaternion.eulerAngles + factor2 * convertMatrix2.quaternion.eulerAngles)/ factorSum;

        //Quaternion quaternionN = Quaternion.Euler(newAngles);
        Quaternion quaternionN = convertMatrix1.quaternion;
        Vector3 positionN = (convertMatrix1.position * factor1 + convertMatrix2.position * factor2)/ factorSum;
        return new ConvertMatrix(quaternionN, positionN);

    }


    static public float[,] MultiplyMatrix(float[,] mat1, float[,] mat2)
    {
        float[,] output = new float[4, 4];
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                output[i, j] = 0;
                for (int k = 0; k < 4; k ++)
                {
                    output[i, j] += mat1[i, k] * mat2[k, j];
                }
            }
        }
        return output;
    }

    public Quaternion GetQuaternion()
    {
        return quaternion;
    }

    public Vector3 GetPosition()
    {
        return position;
    }

    public void PrintQP()
    {
        Debug.Log("quaternion is :" + quaternion.x + " " + quaternion.y + " " + quaternion.z + " " + quaternion.w + " , the position is :" +
            position.x + " " + position.y + " " + position.z);
    }
}
