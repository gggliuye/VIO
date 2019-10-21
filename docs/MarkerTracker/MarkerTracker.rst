Image(Marker) Localization
===================================

The point here, is to make a image(marker) localization algorithm, with the similar effect as the one Vuforia did. With high precise and fast processing.

Pipeline
------------------------

**Marker Maker**
* extract infomation we need
    - several features with descriptors 
    - more features but without descriptors
    - infomation of marker's real size 
* save the infomation to a data file



PreTreatment
--------------------
Before we start calcuation, we need to preprocess the input image. And also prepare the marker data.

Marker Prepare
~~~~~~~~~~~~~~~~~~~~~~~
We should choose marker image with enough features and "good" layout, more details can be seen in Vuforia home page. As most marker localization algorithms share the same criteria for choosing marker.

If we need connection with other localization applization, we have to obtain the real scale of the image (the best for unit all localization measurement is to use the standard meter). So we need to measure the exact size of the printed marker.

What's next is the camera calibration process. All obtain the camera intrinsic parameters from camera devices.

Image Preprocess
~~~~~~~~~~~~~~~~~~~~~~~~

To realize fast processing, we need to resize both the marker image and the input image to have suitable size. (Remember to record ratio we used so we can calculate its real scale, to keep its connection with the real world)

    imageResizeRatio = std::max(
                MARKER_STANDARD_HEIGHT * 1.0 / std::min(src.cols, src.rows),
                MARKER_STANDARD_WIDTH * 1.0 / std::max(src.cols, src.rows));

    cv::resize(src, dst, Size(), imageResizeRatio, imageResizeRatio);

And we didn't do any more preprocessing here to avoid redundant calculation.

