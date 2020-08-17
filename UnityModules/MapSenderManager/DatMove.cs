using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Networking;

namespace Locus.ImageRecognition
{
    // 将dat包从StreamingAssets路径下转移到Application.persistentDataPath持久化路径
    public class DatMove
    {
        // dat包转移协程
        public IEnumerator IEMoveDat(string streamingAssetsPath, System.Action action)
        {
            // StreamingAssets路径
            string path = streamingAssetsPath;
            // Application.streamingAssetsPath + "/" + path 为完整dat包路径

            string strMD5 = null;

            // 是否在沙盒路径下写文件
            bool isGenerated = true;

            // PlayerPrefs可以判断数据的有无以及删除清空数据
            if (PlayerPrefs.HasKey(path))
            {
                Debug.Log("SDK: 存在path路径的key");
                strMD5 = PlayerPrefs.GetString(path);
                isGenerated = false;
            }

            // 将dat包从StreamingAssets路径下转移到Application.persistentDataPath持久化路径相应目录
            string[] folder = path.Split('/');
            string persistentFolderPath = Application.persistentDataPath;

            if (folder.Length > 1)
            {
                for (int i = 0; i < folder.Length - 1; i++)
                    persistentFolderPath += "/" + folder[i];

                // 创建相应文件夹，还未转移dat包
                if (!Directory.Exists(persistentFolderPath))
                    Directory.CreateDirectory(persistentFolderPath);
            }

            // 检查dat包是否存在
            if (!File.Exists(Application.persistentDataPath + "/" + path))
            {
                Debug.Log("SDK: 识别数据不存在,读取streamingAssetsPath");
                isGenerated = true;
            }

#if UNITY_IOS
            string url = "file://" + Application.streamingAssetsPath + "/" + path;
#elif UNITY_ANDROID
            string url = Application.streamingAssetsPath + "/" + path;
#endif
            Debug.Log("SDK: unity URL:: " + url);

            UnityWebRequest uwr = UnityWebRequest.Get(url);

            yield return uwr.SendWebRequest();

            Debug.Log(" SDK: ########################## 读取streamingAssets中的识别数据 ################################# ");
            if (uwr.isNetworkError)
            {
                Debug.Log("SDK: 读取失败 Error:: " + uwr.error);

                // if (action != null) action();
                yield break;
            }

            byte[] bytes = uwr.downloadHandler.data;
            string persistentFilePath = Application.persistentDataPath + "/" + path;

            // md5加密，即使文件名一致，若内容不一致 也需修改文件内容
            string md5 = FileMd5Info.GetFileBytesMD5(bytes);
            if (!md5.Equals(strMD5))
            {
                isGenerated = true;
                Debug.Log("SDK: 文件不存在或被修改");
            }
            else
            {
                isGenerated = false;
                Debug.Log("SDK: 文件已存在，且未被修改");
            }

            if (!File.Exists(persistentFilePath))
            {
                FileProcess.SaveFile(persistentFilePath, bytes);
                PlayerPrefs.SetString(path, md5);
            }
            else
            {
                if (isGenerated)
                {
                    FileProcess.SaveFile(persistentFilePath, bytes);
                    PlayerPrefs.SetString(path, md5);
                }
            }

            if (action != null) action();
        }
    }

    public class FileProcess
    {
        public static void SaveFile(string path, byte[] bytes)
        {
            try
            {
                FileStream fs = File.Open(path, FileMode.OpenOrCreate);
                fs.Write(bytes, 0, bytes.Length);
                fs.Close();

                //PlayerPrefs.SetString(_p, strMD5);
                Debug.Log("SDK: 文件 " + path + " 保存到沙盒路径");
            }
            catch (System.Exception e)
            {
                Debug.Log("SDK: 写入失败 Error:: " + e.Message);
            }
        }
    }

    // 测试考虑
    // 1、 文件不存在
    // 2、 文件已存在
    // 3、 文件已存在但内容被修改
}
