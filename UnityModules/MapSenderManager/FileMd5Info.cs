using System;
using System.IO;
using System.Text;
namespace Locus.ImageRecognition
{
    public class FileMd5Info
    {

        public static string GetFileMD5(string strFileName)
        {
            try
            {
                FileStream file = new FileStream(strFileName, FileMode.Open);
                System.Security.Cryptography.MD5 md5 = new System.Security.Cryptography.MD5CryptoServiceProvider();
                byte[] retVal = md5.ComputeHash(file);
                file.Close();

                StringBuilder sb = new StringBuilder();
                int nlen = retVal.Length;
                for (int ii = 0; ii < nlen; ii++)
                {
                    sb.Append(retVal[ii].ToString("x2"));
                }
                return sb.ToString();
            }
            catch (Exception e)
            {
                throw new Exception("GetMD5HashFromFile() fail, error:" + e.Message);
            }
        }

        public static string GetFileBytesMD5(byte[] bytes)
        {
            System.Security.Cryptography.MD5 md5 = new System.Security.Cryptography.MD5CryptoServiceProvider();
            byte[] retVal = md5.ComputeHash(bytes);

            StringBuilder sb = new StringBuilder();
            int nlen = retVal.Length;
            for (int ii = 0; ii < nlen; ii++)
            {
                sb.Append(retVal[ii].ToString("x2"));
            }
            return sb.ToString();
        }

    }
}
