# Porting Plan: OA-SLAM Object Visualization from ORB-SLAM2 to ORB-SLAM3

## Overview

This plan details every change needed to port the OA-SLAM object-aware visualization code from the ORB-SLAM2 adapter's viewer components to the ORB-SLAM3 adapter's viewer components.

**Source adapter:** `src/adapters/orbslam2/internal/`
**Target adapter:** `src/adapters/orbslam3/internal/`

## Key API Differences

| Concept | ORB-SLAM2 | ORB-SLAM3 |
|---------|-----------|-----------|
| Map container | `Map* mpMap` | `Atlas* mpAtlas` then `mpAtlas->GetCurrentMap()` |
| Camera pose type | `cv::Mat mCameraPose` | `Sophus::SE3f mCameraPose` |
| MapPoint position | `cv::Mat GetWorldPos()` | `Eigen::Vector3f GetWorldPos()` |
| Frame pose | `cv::Mat mTcw` | `Sophus::SE3f mTcw` via `GetPose()` |
| Frame calibration | `cv::Mat mK` | `Eigen::Matrix3f mK_` |
| Namespace | `ORB_SLAM2` | `ORB_SLAM3` |
| MapDrawer ctor | `MapDrawer(Map*, string&)` | `MapDrawer(Atlas*, string&, Settings*)` |
| FrameDrawer ctor | `FrameDrawer(Map*)` | `FrameDrawer(Atlas*)` |
| Viewer ctor | 5 params | 6 params with Settings* |
| DrawMapPoints | `DrawMapPoints(double, bool)` | `DrawMapPoints()` no params |
| DrawKeyFrames | 2 bool params | 4 bool params |
| GetCurrentOpenGLCameraMatrix | 1 param | 2 params with MOw |
| Image source in Update | `pTracker->im_rgb_` | `pTracker->mImGray` |

## Shared Types Already Present in ORB-SLAM3

These types already exist in the ORB-SLAM3 adapter with the correct `ORB_SLAM3` namespace:

- `MapObject` with `GetEllipsoid()`, `GetTrack()`
- `ObjectTrack` with `GetColor()`, `GetId()`, `GetCategoryId()`, `GetLastBbox()`, `GetLastObsFrameId()`, `GetLastObsScore()`, `GetStatus()`, `GetMapObject()`, `GetFilteredAssociatedMapPoints()`, `unc_`
- `Ellipse` with `GetCenter()`, `GetAxes()`, `GetAngle()`
- `Ellipsoid` with `GetCenter()`, `GetAxes()`, `GetOrientation()`, `GeneratePointCloud()`, `project()`
- `CategoryColorsManager::GetInstance()`
- `Map::GetAllMapObjects()`
- `Tracking::GetObjectTracks()`, `GetCurrentFrameDetections()`, `GetCurrentFrameIdx()`, `GetCurrentMeanDepth()`, `im_rgb_`

---

## Recommended Implementation Order

1. **Change 1** — `MapDrawer.h` header additions
2. **Change 2** — `MapDrawer.cc` implementation
3. **Change 3** — `FrameDrawer.h` header additions
4. **Change 4** — `FrameDrawer.cc` implementation
5. **Change 5** — `Viewer.h` header additions
6. **Change 6** — `Viewer.cc` menu items and draw calls

This order ensures that each file compiles before the next depends on it.

---

## Change 1: MapDrawer.h — Add Object Drawing Declarations

**File:** `src/adapters/orbslam3/internal/include/MapDrawer.h`

### 1a. Add Eigen include

**After line 28** (`#include<mutex>`), add:
```cpp
#include <Eigen/Dense>
```

### 1b. Change DrawMapPoints signature

**Replace** the current declaration on line 46:
```cpp
void DrawMapPoints();
```
**With:**
```cpp
void DrawMapPoints(double size, bool ignore_objects_points);
void DrawMapObjectsPoints(double size);
```

### 1c. Add object drawing method declarations

**After line 51** (`void GetCurrentOpenGLCameraMatrix(...);`), add:
```cpp
    void DrawMapObjects();
    void SetUseCategoryColors(bool use_cat_cols) {
        use_category_cols_ = use_cat_cols;
    }
    void SetDisplay3DBbox(bool disp_bbox) {
        display_3d_bbox_ = disp_bbox;
    }

    void DrawDistanceEstimation(double depth, const Sophus::SE3f &Tcw);
```

> **API adaptation note:** ORB-SLAM2 takes `cv::Mat &Tcw`. ORB-SLAM3 takes `const Sophus::SE3f &Tcw`.

### 1d. Add private member variables

**After line 73** (the `mfFrameColors` array closing brace), add:
```cpp
    bool use_category_cols_ = false;
    bool display_3d_bbox_ = false;
```

---

## Change 2: MapDrawer.cc — Implement Object Drawing

**File:** `src/adapters/orbslam3/internal/src/MapDrawer.cc`

### 2a. Add includes

**After line 23** (`#include <mutex>`), add:
```cpp
#include "Ellipsoid.h"
#include "MapObject.h"
#include "ObjectTrack.h"
#include "ColorManager.h"
#include <unordered_map>
#include <unordered_set>
```

### 2b. Initialize new members in constructor

**In** `MapDrawer::newParameterLoader(Settings*)` (line 53), after `mCameraLineWidth = settings->cameraLineWidth();` add:
```cpp
    use_category_cols_ = false;
    display_3d_bbox_ = false;
```

**In** the `else` branch of the constructor (around line 34), after the `ParseViewerParamFile` block completes, add:
```cpp
    use_category_cols_ = false;
    display_3d_bbox_ = false;
```

### 2c. Replace DrawMapPoints with parameterized version

**Replace** the entire `DrawMapPoints()` method (lines 135-176) with:

```cpp
void MapDrawer::DrawMapPoints(double size, bool ignore_objects_points)
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    std::unordered_set<MapPoint*> associated;
    if (ignore_objects_points) {
        const std::vector<MapObject*> objects = pActiveMap->GetAllMapObjects();
        for (auto* obj : objects) {
            auto assoc_points = obj->GetTrack()->GetFilteredAssociatedMapPoints(10);
            for (auto pt_cnt : assoc_points) {
                associated.insert(pt_cnt.first);
            }
        }
    }

    glPointSize(size);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        if (associated.count(vpMPs[i]) > 0)
            continue;
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    glPointSize(size);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        if (associated.count(*sit) > 0)
            continue;
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }

    glEnd();
}
```

**Adaptations from ORB-SLAM2:**
- `mpAtlas->GetCurrentMap()` instead of `mpMap`
- `Eigen::Matrix<float,3,1>` for `GetWorldPos()` instead of `cv::Mat`
- `pos(0)` instead of `pos.at<float>(0)`

### 2d. Add DrawMapObjectsPoints

**After** `DrawMapPoints`, add:

```cpp
void MapDrawer::DrawMapObjectsPoints(double size)
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    const auto& color_manager = CategoryColorsManager::GetInstance();
    const std::vector<MapObject*> objects = pActiveMap->GetAllMapObjects();
    for (auto* obj : objects) {
        cv::Scalar c;
        if (use_category_cols_) {
            c = color_manager[obj->GetTrack()->GetCategoryId()];
        } else {
            c = obj->GetTrack()->GetColor();
        }
        glColor3f(static_cast<double>(c(2)) / 255,
                static_cast<double>(c(1)) / 255,
                static_cast<double>(c(0)) / 255);
        auto assoc_points = obj->GetTrack()->GetFilteredAssociatedMapPoints(10);
        glPointSize(size);
        glBegin(GL_POINTS);
        for (auto pt_cnt : assoc_points) {
            MapPoint* pt = pt_cnt.first;
            if (pt->isBad())
                continue;
            Eigen::Matrix<float,3,1> pos = pt->GetWorldPos();
            glVertex3f(pos(0),pos(1),pos(2));
        }
        glEnd();
    }
}
```

### 2e. Add DrawDistanceEstimation

**After** `DrawMapObjectsPoints`, add:

```cpp
void MapDrawer::DrawDistanceEstimation(double depth, const Sophus::SE3f &Tcw)
{
    Eigen::Matrix4f T = Tcw.matrix();
    Eigen::Matrix3d R = T.block<3, 3>(0, 0).cast<double>();
    Eigen::Vector3d t = T.block<3, 1>(0, 3).cast<double>();
    Eigen::Matrix3d o = R.transpose();
    Eigen::Vector3d p = -o * t;
    Eigen::Vector3d e = p + depth * o.col(2);

    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(p[0], p[1], p[2]);
    glVertex3f(e[0], e[1], e[2]);
    glEnd();
}
```

**Adaptations:** Takes `Sophus::SE3f` instead of `cv::Mat`. Uses `.matrix()` and `.cast<double>()` instead of `cvToEigenMatrix`.

### 2f. Add DrawMapObjects

**After** `DrawDistanceEstimation`, add:

```cpp
void MapDrawer::DrawMapObjects()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const std::vector<MapObject*> objects = pActiveMap->GetAllMapObjects();

    glPointSize(mPointSize);
    const auto& color_manager = CategoryColorsManager::GetInstance();
    glLineWidth(2);
    for (auto *obj : objects) {
        cv::Scalar c;
        if (use_category_cols_) {
            c = color_manager[obj->GetTrack()->GetCategoryId()];
        } else {
            c = obj->GetTrack()->GetColor();
        }
        glColor3f(static_cast<double>(c(2)) / 255,
                  static_cast<double>(c(1)) / 255,
                  static_cast<double>(c(0)) / 255);
        const Ellipsoid& ell = obj->GetEllipsoid();
        if (!display_3d_bbox_) {
            auto pts = ell.GeneratePointCloud();
            int i = 0;
            while (i < pts.rows()) {
                glBegin(GL_LINE_STRIP);
                for (int k = 0; k < 50; ++k, ++i){
                    glVertex3f(pts(i, 0), pts(i, 1), pts(i, 2));
                }
                glEnd();
            }
        } else {
            Eigen::Vector3d center = ell.GetCenter();
            Eigen::Vector3d axes = ell.GetAxes();
            Eigen::Matrix3d R = ell.GetOrientation();
            Eigen::Matrix<double, 8, 3> pts;
            pts << -axes[0], -axes[1], -axes[2],
                    axes[0], -axes[1], -axes[2],
                    axes[0],  axes[1], -axes[2],
                   -axes[0],  axes[1], -axes[2],
                   -axes[0], -axes[1],  axes[2],
                    axes[0], -axes[1],  axes[2],
                    axes[0],  axes[1],  axes[2],
                   -axes[0],  axes[1],  axes[2];
            Eigen::Matrix<double, 8, 3> obb = (R * pts.transpose()).transpose();
            obb.rowwise() += center.transpose();

            glBegin(GL_LINE_STRIP);
            glVertex3f(obb(0, 0), obb(0, 1), obb(0, 2));
            glVertex3f(obb(1, 0), obb(1, 1), obb(1, 2));
            glVertex3f(obb(2, 0), obb(2, 1), obb(2, 2));
            glVertex3f(obb(3, 0), obb(3, 1), obb(3, 2));
            glVertex3f(obb(0, 0), obb(0, 1), obb(0, 2));
            glVertex3f(obb(4, 0), obb(4, 1), obb(4, 2));
            glVertex3f(obb(5, 0), obb(5, 1), obb(5, 2));
            glVertex3f(obb(1, 0), obb(1, 1), obb(1, 2));
            glVertex3f(obb(5, 0), obb(5, 1), obb(5, 2));
            glVertex3f(obb(6, 0), obb(6, 1), obb(6, 2));
            glVertex3f(obb(2, 0), obb(2, 1), obb(2, 2));
            glVertex3f(obb(6, 0), obb(6, 1), obb(6, 2));
            glVertex3f(obb(7, 0), obb(7, 1), obb(7, 2));
            glVertex3f(obb(3, 0), obb(3, 1), obb(3, 2));
            glVertex3f(obb(7, 0), obb(7, 1), obb(7, 2));
            glVertex3f(obb(4, 0), obb(4, 1), obb(4, 2));
            glEnd();
        }
    }
}
```

**Adaptation:** Uses `mpAtlas->GetCurrentMap()` instead of `mpMap`. Ellipsoid API is identical since it uses Eigen types in both adapters.

---

## Change 3: FrameDrawer.h — Add Object Widget Types and Methods

**File:** `src/adapters/orbslam3/internal/include/FrameDrawer.h`

### 3a. Add includes

**After line 25** (`#include "Atlas.h"`), add:
```cpp
#include "ImageDetections.h"
#include "Ellipse.h"

#include <Eigen/Dense>
```

### 3b. Add DetectionWidget and ObjectProjectionWidget structs

**After line 38** (the `class Viewer;` forward declaration), add:

```cpp
struct DetectionWidget
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    DetectionWidget(BBox2 bb, unsigned int idx, unsigned int cat, double s, cv::Scalar col, double thick, bool disp_info)
        : bbox(bb), id(idx), category_id(cat), score(s), color(col), thickness(thick), display_info(disp_info) {}

    BBox2 bbox;
    unsigned int id;
    unsigned int category_id;
    double score;
    cv::Scalar color;
    double thickness;
    bool display_info;
};

struct ObjectProjectionWidget
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ObjectProjectionWidget(const Ellipse& ell, unsigned int idx, unsigned int cat, cv::Scalar col, bool is_in_map, double unc)
        : ellipse(ell), id(idx), category_id(cat), color(col), in_map(is_in_map), uncertainty(unc) {}
    Ellipse ellipse;
    unsigned int id;
    unsigned int category_id;
    cv::Scalar color;
    bool in_map;
    double uncertainty;
};
```

### 3c. Add drawing method declarations to FrameDrawer class

**After line 51** (`cv::Mat DrawRightFrame(float imageScale=1.f);`), add:
```cpp
    cv::Mat DrawDetections(cv::Mat img);
    cv::Mat DrawProjections(cv::Mat img);

    void SetUseCategoryColors(bool use_cat_cols) {
        use_category_cols_ = use_cat_cols;
    }
```

### 3d. Add member variables to protected section

**After line 74** (`std::mutex mMutex;`), add:
```cpp
    std::vector<DetectionWidget, Eigen::aligned_allocator<DetectionWidget>> detections_widgets_;
    std::vector<ObjectProjectionWidget, Eigen::aligned_allocator<ObjectProjectionWidget>> object_projections_widgets_;
    bool use_category_cols_ = false;
    int frame_id_ = 0;
```

---

## Change 4: FrameDrawer.cc — Implement Object Drawing and Update Logic

**File:** `src/adapters/orbslam3/internal/src/FrameDrawer.cc`

### 4a. Add includes

**After line 20** (`#include "Tracking.h"`), add:
```cpp
#include "ImageDetections.h"
#include "MapObject.h"
#include "ColorManager.h"
```

### 4b. Modify Update method — change image source

**In** `FrameDrawer::Update(Tracking *pTracker)` (line 373), **replace** line 373:
```cpp
    pTracker->mImGray.copyTo(mIm);
```
**With:**
```cpp
    pTracker->im_rgb_.copyTo(mIm);
```

This ensures the frame viewer shows the color image with object overlays, not grayscale.

### 4c. Modify Update method — add object track data collection

**After** line 436 (`mState=static_cast<int>(pTracker->mLastProcessedState);`), and before the closing brace of `Update`, add:

```cpp
    // OA-SLAM: collect object track data for visualization
    auto tracks = pTracker->GetObjectTracks();
    detections_widgets_.clear();
    frame_id_ = pTracker->GetCurrentFrameIdx();

    const auto& detections = pTracker->GetCurrentFrameDetections();
    for (auto det : detections) {
        detections_widgets_.push_back(DetectionWidget(det->bbox, 0, det->category_id,
                                        det->score, cv::Scalar(0, 0, 0),
                                        2, false));
    }
    auto current_frame_id = pTracker->GetCurrentFrameIdx();
    for (auto tr : tracks) {
        if (current_frame_id == tr->GetLastObsFrameId()) {
            auto bb = tr->GetLastBbox();
            detections_widgets_.push_back(DetectionWidget(bb, tr->GetId(), tr->GetCategoryId(),
                                                            tr->GetLastObsScore(), tr->GetColor(),
                                                            3, true));
        }
    }

    object_projections_widgets_.clear();

    // ORB-SLAM3: use Sophus SE3f for pose, Eigen for calibration
    if (pTracker->mCurrentFrame.HasPose()) {
        Eigen::Matrix4f Tcw_f = pTracker->mCurrentFrame.GetPose().matrix();
        Eigen::Matrix<double, 3, 4> Rt;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 4; ++j)
                Rt(i, j) = static_cast<double>(Tcw_f(i, j));

        Eigen::Matrix3f K_f = pTracker->mCurrentFrame.mK_;
        Eigen::Matrix3d K = K_f.cast<double>();
        Eigen::Matrix<double, 3, 4> P = K * Rt;

        for (auto& tr : tracks) {
            const auto* obj = tr->GetMapObject();
            if (obj) {
                auto proj = obj->GetEllipsoid().project(P);
                object_projections_widgets_.push_back(ObjectProjectionWidget(proj, tr->GetId(),
                                                                             tr->GetCategoryId(), tr->GetColor(),
                                                                             tr->GetStatus() == ObjectTrackStatus::IN_MAP,
                                                                             tr->unc_));
            }
        }
    }
```

**Key adaptations from ORB-SLAM2:**
- Uses `pTracker->mCurrentFrame.HasPose()` guard instead of checking `cv_Rt.rows == 4`
- Uses `pTracker->mCurrentFrame.GetPose().matrix()` to get `Eigen::Matrix4f` instead of `cv::Mat mTcw`
- Uses `pTracker->mCurrentFrame.mK_` which is `Eigen::Matrix3f` instead of `cv::Mat mK`
- Casts float to double for the projection matrix computation

### 4d. Modify DrawTextInfo — add frame_id display

**In** `DrawTextInfo` (line 331), after the tracking state text is built and before `int baseline=0;` (line 360), add:
```cpp
    s << " | FRAME "  << frame_id_;
```

### 4e. Add DrawDetections method

**After** the `Update` method closing brace, add:

```cpp
cv::Mat FrameDrawer::DrawDetections(cv::Mat img)
{
    std::vector<DetectionWidget, Eigen::aligned_allocator<DetectionWidget>> detections;
    {
        unique_lock<mutex> lock(mMutex);
        detections = detections_widgets_;
    }
    const auto& manager = CategoryColorsManager::GetInstance();
    cv::Scalar color;
    for (auto d : detections) {
        const auto& bb = d.bbox;
        if (use_category_cols_) {
            color = manager[d.category_id];
        } else {
            color = d.color;
        }
        cv::rectangle(img, cv::Point2i(bb[0], bb[1]),
                           cv::Point2i(bb[2], bb[3]),
                           color,
                           d.thickness);
        if (d.display_info) {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << d.score;
            cv::putText(img, std::to_string(d.id) + "|" + ss.str() + "|" + std::to_string(d.category_id),
                        cv::Point2i(bb[0]-10, bb[1]-5), cv::FONT_HERSHEY_DUPLEX,
                        0.55, cv::Scalar(255, 255, 255), 1, false);
        }
    }
    return img;
}
```

### 4f. Add draw_ellipse_dashed helper and DrawProjections method

**After** `DrawDetections`, add:

```cpp
static void draw_ellipse_dashed(cv::Mat img, const Ellipse& ell, const cv::Scalar& color, int thickness)
{
    int size = 8;
    int space = 16;
    const auto& c = ell.GetCenter();
    const auto& axes = ell.GetAxes();
    double angle = ell.GetAngle();
    for (int i = 0; i < 360; i += space) {
        cv::ellipse(img, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]),
                    TO_DEG(angle), i, i+size, color, thickness);
    }
}

cv::Mat FrameDrawer::DrawProjections(cv::Mat img)
{
    std::vector<ObjectProjectionWidget, Eigen::aligned_allocator<ObjectProjectionWidget>> projections;
    {
        unique_lock<mutex> lock(mMutex);
        projections = object_projections_widgets_;
    }

    const auto& manager = CategoryColorsManager::GetInstance();
    cv::Scalar color;
    for (auto w : projections)
    {
        const auto& ell = w.ellipse;
        const auto& c = ell.GetCenter();
        const auto& axes = ell.GetAxes();
        double angle = ell.GetAngle();
        if (use_category_cols_) {
            color = manager[w.category_id];
        } else {
            color = w.color;
        }
        if (w.in_map) {
            cv::ellipse(img, cv::Point2f(c[0], c[1]), cv::Size2f(axes[0], axes[1]), TO_DEG(angle), 0, 360, color, 2);
        }
        else
            draw_ellipse_dashed(img, ell, color, 2);
    }

    return img;
}
```

**Note:** `TO_DEG` macro is defined in `Utils.h`. Verify it is included transitively via `Ellipse.h` -> `Utils.h`. If not, add `#include "Utils.h"` to the includes.

---

## Change 5: Viewer.h — Add Object-Related Members

**File:** `src/adapters/orbslam3/internal/include/Viewer.h`

### 5a. Add isPaused to public section

**After line 60** (`bool isStepByStep();`), add:
```cpp
    bool isPaused();
```

### 5b. Add private member variables

**After line 95** (`bool mbStopTrack;`), add:
```cpp
    bool mbPaused = false;
    bool use_class_col_ = false;
```

---

## Change 6: Viewer.cc — Add Object Menu Items and Draw Calls

**File:** `src/adapters/orbslam3/internal/src/Viewer.cc`

### 6a. Change window title

**Replace line 167:**
```cpp
    pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer",1024,768);
```
**With:**
```cpp
    pangolin::CreateWindowAndBind("OA-SLAM: Map Viewer",1024,768);
```

### 6b. Add Pangolin menu items for objects

**After line 191** (`pangolin::Var<bool> menuShowOptLba(...);`), add:
```cpp
    pangolin::Var<bool> menuShowCamera("menu.Show Camera",true,true);
    pangolin::Var<bool> menuShowObjectsPoints("menu.Show Obj-Points",true,true);
    pangolin::Var<float> menuPointsSize("menu.Points Size", 1.61, 1e-1, 1e1, true);
    pangolin::Var<float> menuObjectsPointsSize("menu.Obj-Points Size", 1.0, 1e-1, 1e1, true);
    pangolin::Var<bool> menuShowObjects("menu.Show Objects",true,true);
    pangolin::Var<bool> menuPause("menu.Pause",false,true);
    pangolin::Var<bool> menuCatCol("menu.Color by Cat", false, true);
    pangolin::Var<bool> menu3DBbox("menu.Disp 3D Bboxes", false, true);
    pangolin::Var<bool> menuDistEstim("menu.Disp Distance Est.", false, true);
    pangolin::Var<bool> menuQuit("menu.Quit",false,false);
```

### 6c. Change cv::namedWindow title

**Replace line 207:**
```cpp
    cv::namedWindow("ORB-SLAM3: Current Frame");
```
**With:**
```cpp
    cv::namedWindow("OA-SLAM: Current Frame");
```

### 6d. Add Show Camera guard around DrawCurrentCamera

**Replace line 312:**
```cpp
        mpMapDrawer->DrawCurrentCamera(Twc);
```
**With:**
```cpp
        if (menuShowCamera)
            mpMapDrawer->DrawCurrentCamera(Twc);
```

### 6e. Modify DrawMapPoints call and add object drawing calls

**Replace lines 315-316:**
```cpp
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();
```
**With:**
```cpp
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints(menuPointsSize, menuShowObjectsPoints);
        if(menuShowObjectsPoints)
            mpMapDrawer->DrawMapObjectsPoints(menuObjectsPointsSize);
        if (menuShowObjects)
            mpMapDrawer->DrawMapObjects();
```

### 6f. Add distance estimation drawing

**After** the object drawing calls added in 6e, and **before** `pangolin::FinishFrame();` (line 318), add:
```cpp
        if (menuDistEstim)
            mpMapDrawer->DrawDistanceEstimation(mpTracker->GetCurrentMeanDepth(), mpTracker->mCurrentFrame.GetPose());
```

**Adaptation:** ORB-SLAM2 passes `mpTracker->mCurrentFrame.mTcw` which is `cv::Mat`. ORB-SLAM3 uses `mpTracker->mCurrentFrame.GetPose()` which returns `Sophus::SE3f`.

### 6g. Add object drawing on the 2D frame image

**After** the line that creates the frame image (around line 321):
```cpp
        cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);
```

And **after** the `if(both)` block that concatenates left/right frames, **before** the `mImageViewerScale` resize block, add:
```cpp
        if (menuShowObjects) {
            mpFrameDrawer->DrawDetections(toShow);
            mpFrameDrawer->DrawProjections(toShow);
        }
```

This should be placed after `toShow` is assigned (either from `im` or from `hconcat`), but before the `mImageViewerScale` resize. The exact insertion point is after line 329:
```cpp
        else{
            toShow = im;
        }
```

### 6h. Change cv::imshow title

**Replace line 338:**
```cpp
        cv::imshow("ORB-SLAM3: Current Frame",toShow);
```
**With:**
```cpp
        cv::imshow("OA-SLAM: Current Frame",toShow);
```

### 6i. Add category color and 3D bbox toggle handling

**After** the `menuStop` block (around line 369), and **before** `if(Stop())`, add:
```cpp
        if (menuPause) {
            mbPaused = true;
        } else {
            mbPaused = false;
        }

        if (menuCatCol) {
            mpMapDrawer->SetUseCategoryColors(true);
            mpFrameDrawer->SetUseCategoryColors(true);
        } else {
            mpMapDrawer->SetUseCategoryColors(false);
            mpFrameDrawer->SetUseCategoryColors(false);
        }
        mpMapDrawer->SetDisplay3DBbox(menu3DBbox);

        if (menuQuit) {
            this->RequestFinish();
        }
```

### 6j. Add isPaused implementation

**After** the `Release()` method, add:
```cpp
bool Viewer::isPaused()
{
    return mbPaused;
}
```

---

## Summary of All API Adaptations

| ORB-SLAM2 Pattern | ORB-SLAM3 Equivalent |
|---|---|
| `mpMap->GetAllMapObjects()` | `mpAtlas->GetCurrentMap()->GetAllMapObjects()` |
| `mpMap->GetAllMapPoints()` | `mpAtlas->GetCurrentMap()->GetAllMapPoints()` |
| `mpMap->GetReferenceMapPoints()` | `mpAtlas->GetCurrentMap()->GetReferenceMapPoints()` |
| `cv::Mat pos = pt->GetWorldPos()` | `Eigen::Vector3f pos = pt->GetWorldPos()` |
| `pos.at<float>(0)` | `pos(0)` |
| `cv::Mat mTcw` on Frame | `Frame::GetPose()` returns `Sophus::SE3f` |
| `cv::Mat mK` on Frame | `Eigen::Matrix3f mK_` on Frame |
| `Tcw.rows == 4 && Tcw.cols == 4` check | `Frame::HasPose()` check |
| `cvToEigenMatrix<double,float,4,4>(Tcw)` | `Tcw.matrix()` then `.cast<double>()` |
| `mpMap->KeyFramesInMap()` | `mpAtlas->KeyFramesInMap()` |
| `mpMap->MapPointsInMap()` | `mpAtlas->MapPointsInMap()` |

## Files Modified Summary

| File | Type of Change |
|---|---|
| `src/adapters/orbslam3/internal/include/MapDrawer.h` | Add method declarations and member variables |
| `src/adapters/orbslam3/internal/src/MapDrawer.cc` | Add includes, modify DrawMapPoints, add 4 new methods |
| `src/adapters/orbslam3/internal/include/FrameDrawer.h` | Add structs, method declarations, member variables |
| `src/adapters/orbslam3/internal/src/FrameDrawer.cc` | Add includes, modify Update, add 3 new methods |
| `src/adapters/orbslam3/internal/include/Viewer.h` | Add isPaused, member variables |
| `src/adapters/orbslam3/internal/src/Viewer.cc` | Add menu items, draw calls, toggle handling |

## Compilation Notes

- All shared OA-SLAM types (`MapObject`, `ObjectTrack`, `Ellipse`, `Ellipsoid`, `ColorManager`) already exist in the ORB-SLAM3 adapter with the `ORB_SLAM3` namespace. No changes needed to these files.
- The `Map::GetAllMapObjects()` method already exists in the ORB-SLAM3 `Map.h`.
- The `Tracking` class in ORB-SLAM3 already has `GetObjectTracks()`, `GetCurrentFrameDetections()`, `GetCurrentFrameIdx()`, `GetCurrentMeanDepth()`, and `im_rgb_`.
- Ensure `#include <iomanip>` is present in `FrameDrawer.cc` for `std::setprecision` used in `DrawDetections`.
