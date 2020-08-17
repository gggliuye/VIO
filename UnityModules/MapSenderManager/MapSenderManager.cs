using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;

public class MapSenderManager : MonoBehaviour
{

    [DllImport("SaveDataManager")]
    private static extern bool MapAnalysis_Init(int frameWidth, int frameHeight, IntPtr previewTexture);

    [DllImport("SaveDataManager")]
    private static extern void MapAnalysis_UnInit();

    [DllImport("SaveDataManager")]
    private static extern IntPtr GetMakeDataRenderEventFunc();

    public RawImage raw_image;

    IEnumerator Start()
    {
        CreateTextureAndPassToPlugin();
        yield return StartCoroutine("CallPluginAtEndOfFrames");
    }

    private void CreateTextureAndPassToPlugin()
    {
        Texture2D tex = new Texture2D(400, 400, TextureFormat.RGB565, false, false);
        //Debug.Log("camera texture: " + tex.GetNativeTexturePtr().ToInt32());

        // Set texture onto our material
        raw_image.material.mainTexture = tex;
        //GetComponent<Renderer>().material.mainTexture = tex;

        // Pass texture pointer to the plugin
        MapAnalysis_Init(tex.width, tex.height, tex.GetNativeTexturePtr());
    }

    // Update is called once per frame
    void Update()
    {
    }


    private IEnumerator CallPluginAtEndOfFrames()
    {
        while (true)
        {
            // Wait until all frame rendering is done
            yield return new WaitForEndOfFrame();

            // Issue a plugin event with arbitrary integer identifier.
            // The plugin can distinguish between different
            // things it needs to do based on this ID.
            // For our simple plugin, it does not matter which ID we pass here.
            GL.IssuePluginEvent(GetMakeDataRenderEventFunc(), 1);
        }
    }

}
