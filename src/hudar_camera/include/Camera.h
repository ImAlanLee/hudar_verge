/*
 * @Author: AlanLee
 * @Date: 2024-07-01 13:25:35
 * @LastEditors: Sian Li lisian_magic@163.com
 * @LastEditTime: 2024-07-01 13:53:46
 * @FilePath: /MinimalCamera/hikSDK/Camera.h
 * @Description:
 */
#ifndef HK_CAMERA_H__
#define HK_CAMERA_H__
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

class HKCam {

  public:
    HKCam();
    ~HKCam();
    cv::Mat getFrame();

  private:
    int nRet = MV_OK;
    void *handle = NULL;
    MVCC_INTVALUE stParam;
    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    unsigned char *pData;
    bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
};

#endif