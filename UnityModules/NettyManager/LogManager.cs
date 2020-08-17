using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UI;

public class LogManager : MonoBehaviour
{
    private static LogManager _singleton;
    public static LogManager Singleton
    {
        get
        {
            if (_singleton == null)
            {
                _singleton = FindObjectOfType<LogManager>();
            }
            return _singleton;
        }
    }

    public Text logText;
    private string logFilePath;

    private void Start()
    {
        logFilePath = Application.persistentDataPath + "error.txt";
        Application.logMessageReceived += Application_logMessageReceived;
    }

    private void Application_logMessageReceived(string condition, string stackTrace, LogType type)
    {
        if (type == LogType.Error)
        {
            PrintLog(condition + '\n' + stackTrace, true);

            if (!File.Exists(logFilePath))
            {
                File.Create(logFilePath).Dispose();
            }

            File.WriteAllText(logFilePath, condition + '\n' + stackTrace);
        }
    }

    public void WriteLog(string message)
    {
        if (!File.Exists(logFilePath))
        {
            File.Create(logFilePath).Dispose();
        }

        File.WriteAllText(logFilePath, message);
    }

    public void PrintLog(string log, bool isClearLog = false)
    {
        if (isClearLog)
        {
            logText.text = "";
        }
        logText.text += log + '\n';
    }
}
