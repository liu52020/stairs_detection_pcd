//
// Created by liuwei on 2021/4/21.
//

#ifndef STAIRS_DETECTION_CURRENT_SCENE_STAIR_H
#define STAIRS_DETECTION_CURRENT_SCENE_STAIR_H

#include "RGBD/current_scene.h"
#include "stair/stair_classes.h"

class CurrentSceneStair:public CurrentScene {
public:

    CurrentSceneStair(){
        upstair.type = "up";
        downstair.type = "down";
    }



    ~CurrentSceneStair(){}

    bool detectStairs();

    bool getLevelsFromCandidates(Stair & stair, Eigen::Affine3d c2f);


    // 阶梯分类
    Stair downstair;
    Stair upstair;

};









#endif //STAIRS_DETECTION_CURRENT_SCENE_STAIR_H
