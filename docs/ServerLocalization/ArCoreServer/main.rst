ArCore SLAM and Server Localization
===================================

* C++负责调用Java，开启Netty（可以自定义IP），并且负责触发信息发送。
* 使用ArCore的Unity接口（实际还是调用的Arcore的Android SDK，如果可以直接从Android调用会更好），获取当前帧的图像数据。
* 将图像数据（byte[]）传递给C++端，并且在C++实现数据的格式转换，得到Java的Array数据。
* C++调用Java的Netty服务将图片传递给服务器。
* Java的Netty服务收到信息之后直接调用Unity的回调函数。


Netty
~~~~~~~~~~~~~~~~~~~~~~~

Unity
--------------------------

    [DllImport("TextureDecoder")]
    
    private static extern void AR_SetNettyParameters(int a, int b, int c, int d, int port, double focus, int mapid, int devicetype);

    [DllImport("TextureDecoder")]
    
    private static extern void AR_InitNetty();

Netty初始化

            SetTextureFromUnity(Frame.CameraImage.Texture.GetNativeTexturePtr());
            
            txt.text = "Init Netty " + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3] + ":" + ip[4];
            
            AR_InitNetty();
            
            AR_SetNettyParameters(ip[0], ip[1], ip[2], ip[3], ip[4], focus, map_id, device_type);
            
            txt.text = "Init Done";
            
            nettyinited = true;


ArCore
~~~~~~~~~~~~~~~~

ArCore的Unity项目中提供了从它的SDK中获取图像帧的接口，其中包含了很多图像的数据，都在**Frame**类中。

Frame
----------------

ArCore还提供了许多的功能，但是这里只着重于图像数据的获取。

            // 获取ArCore的相机数据类格式的数据
            
            GoogleARCore.CameraImageBytes image = Frame.CameraImage.AcquireCameraImageBytes();

            // 需要将数据提取到buffer中进行下一步操作
            
            int bufferSize = image.YRowStride * image.Height;
            
            byte[] s_ImageBuffer = new byte[bufferSize];

            // Move raw data into managed buffer.
            
            System.Runtime.InteropServices.Marshal.Copy(image.Y, s_ImageBuffer, 0, bufferSize);

然后就可以调用unity的C++ plugin传递图像数据并发送。

    [DllImport("TextureDecoder")]
    
    private static extern void AR_SendImageNetty(byte[] data, int size);
    
    AR_SendImageNetty(s_ImageBuffer, bufferSize);
    

Unity场景布置
~~~~~~~~~~~~~~~~~~~~~

场景布置的逻辑和Nova Game的一致，**不过ArCore摆放锚点的坐标变换还需要调整**。



