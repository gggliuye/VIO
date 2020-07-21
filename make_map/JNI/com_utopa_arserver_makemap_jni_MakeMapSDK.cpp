#include "com_utopa_arserver_makemap_jni_MakeMapSDK.h"
#include "UMappingPlugins.h"
#include <stdio.h>

jfloatArray changeFloatArray(JNIEnv * env,float* data ,int len) {
	jfloatArray result;
	result = env->NewFloatArray(len);

	env->SetFloatArrayRegion( result, 0, len, data);
	free(data);
	return result;
}


jstring charTojstring(JNIEnv* env, const char* pat) {
    jclass strClass = (env)->FindClass("Ljava/lang/String;");
    jmethodID ctorID = (env)->GetMethodID(strClass, "<init>", "([BLjava/lang/String;)V");
    jbyteArray bytes = (env)->NewByteArray(strlen(pat));
    (env)->SetByteArrayRegion(bytes, 0, strlen(pat), (jbyte*) pat);
    jstring encoding = (env)->NewStringUTF("GB2312");
    return (jstring) (env)->NewObject(strClass, ctorID, bytes, encoding);
}

char* jstringToChar(JNIEnv* env, jstring jstr) {
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


static JavaVM *jvm = NULL;
static jobject makedMapCallback = NULL;
static pthread_mutex_t metext;

/***
 * build map data callback
 */
static void buildMapCallBack(const char *key,int flag) {
    std::cout  << " buildMapCallBack key" << key << " flag:" << flag << std::endl;

		/* 锁定互斥锁*/
  //  int result = pthread_mutex_lock(&metext);
 JNIEnv *env = NULL;
		//Attach主线程
  if (makedMapCallback == NULL ||jvm->AttachCurrentThread((void**)&env, NULL) != JNI_OK) {
      return ;
  }

  jclass cls_Obj = env->GetObjectClass(makedMapCallback);
	    std::cout  << " cls_Obj key" << cls_Obj << std::endl;
  jmethodID mid = env->GetMethodID(cls_Obj, "callback",  "(Ljava/lang/String;I)V");
    std::cout  << " jmethodID key" << mid << std::endl;

  jstring keyName = charTojstring(env, key);
	env->CallVoidMethod(makedMapCallback, mid,keyName,flag);

  // env->DeleteLocalRef(makedMapCallback);//使用完一定要释放，不然会造成内存泄漏
  jvm->DetachCurrentThread();//一定要报JNIENV从线程解绑定，不然会造成野指针

	/* 打开互斥锁*/
	// pthread_mutex_unlock(&metext);
}



JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *t_jvm, void *reserved) {
    JNIEnv *env = NULL;
    jint result = -1;

    if (t_jvm->GetEnv((void **) &env, JNI_VERSION_1_8) != JNI_OK) {
        return -1;
    }

    result = JNI_VERSION_1_8;
		jvm  = t_jvm;
    return result;
}


JNIEXPORT jint JNICALL Java_com_utopa_arserver_makemap_jni_MakeMapSDK_initSDK
  (JNIEnv * env, jobject, jstring voc_path1, jstring voc_path2, jstring voc_path3 ){
    //  if (jvm == NULL) {
    //     //通过jni接口 获取java jvm
    //     env->GetJavaVM(&jvm);
    // }

    const char* c_voc_path1 = env->GetStringUTFChars(voc_path1, JNI_FALSE);
    const char* c_voc_path2 = env->GetStringUTFChars(voc_path2, JNI_FALSE);
    const char* c_voc_path3 = env->GetStringUTFChars(voc_path3, JNI_FALSE);


    std::cout << std::endl << " jni_MakeMapSDK_initSDK " << c_voc_path1 << std::endl;


    int result = Internal_Set_Vocabulary_Paths(c_voc_path1,c_voc_path2,c_voc_path3);

    env->ReleaseStringUTFChars(voc_path1, c_voc_path1);
    env->ReleaseStringUTFChars(voc_path2, c_voc_path2);
    env->ReleaseStringUTFChars(voc_path3, c_voc_path3);

   return result;
}

JNIEXPORT jint JNICALL Java_com_utopa_arserver_makemap_jni_MakeMapSDK_buildMap
  (JNIEnv * env, jobject obj, jstring work_space, jstring resource_path, jboolean build_dense,jint quality, jstring key, jobject buildMapCallback){

    const char* c_work_space = env->GetStringUTFChars(work_space, JNI_FALSE);
    const char* c_resource_path = env->GetStringUTFChars(resource_path, JNI_FALSE);
    const char* c_key = env->GetStringUTFChars(key, JNI_FALSE);

    //set callback
		/* 用默认属性初始化一个互斥锁对象*/
  	// pthread_mutex_init(&metext, NULL);
    makedMapCallback = env->NewGlobalRef(buildMapCallback);

    int result = Internal_Start_Map(c_work_space, c_resource_path, (bool)build_dense, (int)quality,c_key,(UMapping::REGISTER_CALLBACK)buildMapCallBack);

    env->ReleaseStringUTFChars(work_space, c_work_space);
    env->ReleaseStringUTFChars(resource_path, c_resource_path);
    env->ReleaseStringUTFChars(key, c_key);


    return result;
}


JNIEXPORT jint JNICALL Java_com_utopa_arserver_makemap_jni_MakeMapSDK_getTaskState
  (JNIEnv *)
{
    int res = Internal_Get_Mapping_State();
    return res;
}
