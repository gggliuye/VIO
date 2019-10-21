Image(Marker) Localization
===================================

The point here, is to make a image(marker) localization algorithm, with the similar effect as the one Vuforia did. With high precise and fast processing.

Pipeline
------------------------

**Marker Maker**:

* Extract infomation we need
    - several features(TYPE a) with descriptors 
    - more features(TYPE b) but without descriptors
    - infomation of marker's real size 
* Save the infomation to a data file

**Process Loop**:

* Load the marker data
* for each new input image :
    1. Extract a certain amount of features(TYPE a) and calculate its descriptors.
    2. Match the features with the features of marker by their descriptors.
    3. Use the matched features to calculate homography matrix between marker and the input image.
    4. Project the features (TYPE b) of the marker by the homography matrix to the input image. And also project a rectangle patch surrounding the features to the image.
    5. Use NCC (or other methods) to find the best match of the patch in the image, and calculate its position.
    6. If have done enough iteration or the error is small enough, go to the final step
    7. Go back to step 4.
* Out put the homography matrix, and calculate the 4 times 4 transformation matrix with PnP method (using the marker's real size)


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

