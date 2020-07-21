#include "Java_com_utopa_arserver_localization_jni_LocalizationSDK.h"
#include "UlocalizationPluginsBeta.h"


jfloatArray changeFloatArray(JNIEnv * env,float* data ,int len) 
{
    jfloatArray result;
    result = env->NewFloatArray(len);

    env->SetFloatArrayRegion( result, 0, len, data);
    free(data);
    return result;
}


jstring charTojstring(JNIEnv* env, const char* pat) 
{
    jclass strClass = (env)->FindClass("Ljava/lang/String;");
    jmethodID ctorID = (env)->GetMethodID(strClass, "<init>", "([BLjava/lang/String;)V");
    jbyteArray bytes = (env)->NewByteArray(strlen(pat));
    (env)->SetByteArrayRegion(bytes, 0, strlen(pat), (jbyte*) pat);
    jstring encoding = (env)->NewStringUTF("GB2312");
    return (jstring) (env)->NewObject(strClass, ctorID, bytes, encoding);
}

char* jstringToChar(JNIEnv* env, jstring jstr) 
{
    char* rtn = NULL;
    jclass clsstring = env->FindClass("java/lang/String");
    jstring strencode = env->NewStringUTF("GB2312");
    jmethodID mid = env->GetMethodID(clsstring, "getBytes", "(Ljava/lang/String;)[B");
    jbyteArray barr = (jbyteArray) env->CallObjectMethod(jstr, mid, strencode);
    jsize alen = env->GetArrayLength(barr);
    jbyte* ba = env->GetByteArrayElements(barr, JNI_FALSE);
    if (alen > 0) {
        rtn = (char*) malloc(alen + 1);
        memcpy(rtn, ba, alen);
        rtn[alen] = 0;
    }
    env->ReleaseByteArrayElements(barr, ba, 0);
    return rtn;
}


JNIEXPORT jint JNICALL Java_com_utopa_arserver_localization_jni_LocalizationSDK_init
  (JNIEnv * env, jclass, jstring  database_path , jstring reconstruction_path, jstring vocIndex_path,jint key)
{
    const char* c_database_path = env->GetStringUTFChars(database_path, JNI_FALSE);
    const char* c_reconstruction_path = env->GetStringUTFChars(reconstruction_path, JNI_FALSE);
    const char* c_vocIndex_path = env->GetStringUTFChars(vocIndex_path, JNI_FALSE);
       
    int result = Internal_Init_Ulocalization_Map_Beta(c_database_path,  c_reconstruction_path,c_vocIndex_path, (int)key);

    env->ReleaseStringUTFChars(database_path, c_database_path);
    env->ReleaseStringUTFChars(reconstruction_path, c_reconstruction_path);
    env->ReleaseStringUTFChars(vocIndex_path, c_vocIndex_path);

    return result;
}

/*
 * Class:     com_utopa_arserver_location_jni_LocationSDK
 * Method:    destroy
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_com_utopa_arserver_localization_jni_LocalizationSDK_destroy
  (JNIEnv * env, jclass,jint key)
{
    return Internal_Destroy_Ulocalization_Map_Beta((int)key);
}

/*
 * Class:     com_utopa_arserver_location_jni_LocationSDK
 * Method:    trackMonocular
 * Signature: ([BID)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL Java_com_utopa_arserver_localization_jni_LocalizationSDK_trackMonocular
  (JNIEnv * env, jclass, jbyteArray inputImage, jint bufferLength, jdouble focus_length, jint devType, 
                jfloatArray poseArray, jint bUseInitGuessJint, jint key)
{
    jbyte* imageData = env->GetByteArrayElements(inputImage, 0);
    unsigned char* char_imageData = (unsigned char*)imageData;
    bool bUseInitGuess = (int)bUseInitGuessJint;
   
   jfloat * init_pose =  NULL;
   if(poseArray != NULL){
          init_pose = env->GetFloatArrayElements(poseArray,0);
  }

  
    //float* data = Internal_Track_Ulocalization_Map(char_imageData, (int) bufferLength, (double) focus_length, (int)devType,(int)key);
    float* data = Internal_Track_Ulocalization_Map_Beta(char_imageData, (int) bufferLength, 
                       (double) focus_length, (int)devType,
                       init_pose, bUseInitGuessJint	, 
                       (int)key);

    std::string dataString("");
    int i = 0;
    for(i = 0; i < 16;i++){
        dataString = dataString + std::to_string(data[i]) + "|";
    }

    char * str = new char[dataString.length()+1];
    strcpy(str, dataString.c_str());

    //const char* str = env->GetStringUTFChars(dataString,0);
    jstring result = env->NewStringUTF(str);

    env->ReleaseByteArrayElements(inputImage, imageData, 0); 
 if(poseArray != NULL){
         env->ReleaseFloatArrayElements(poseArray,init_pose,0);
   }
   

    return result;

}
