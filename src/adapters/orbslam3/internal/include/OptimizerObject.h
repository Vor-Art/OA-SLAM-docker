/**
* This file is part of OA-SLAM.
*
* Copyright (C) 2022 Matthieu Zins <matthieu.zins@inria.fr>
* (Inria, LORIA, Université de Lorraine)
* OA-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OA-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OA-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef OPTIMIZER_OBJECT_H
#define OPTIMIZER_OBJECT_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "Distance.h"

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.hpp"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

Ellipsoid reconstruct_ellipsoid(const std::vector<Ellipse, Eigen::aligned_allocator<Ellipse>>& ellipses,
                                const std::vector<Matrix34d, Eigen::aligned_allocator<Matrix34d>>& projections);

struct EllipsoidQuat {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EllipsoidQuat() {}
    explicit EllipsoidQuat(double *data) {
        // storage [ax ay az tx ty tz rx ry rz]
        // qw is computed such that q(qx, qy, qz, qw) is normalized
        axes = Eigen::Vector3d(data);
        se3 = g2o::SE3Quat(Eigen::Matrix<double, 6, 1>(data+3));
    }

    void SetTo(double *data) const {
        for (int i = 0; i < 3; ++i)
            data[i] = axes[i];
        auto rt = se3.log();
        for (int i = 0; i < 6; ++i)
            data[i+3] = rt[i];
    }

    Ellipsoid ToEllipsoid() const {
        return Ellipsoid(axes.cwiseAbs(), se3.rotation().toRotationMatrix(), se3.translation());
    }
    static EllipsoidQuat FromEllipsoid(const Ellipsoid& ell) {
        EllipsoidQuat eq;
        eq.axes = ell.GetAxes();
        eq.se3 = g2o::SE3Quat(Eigen::Quaterniond(ell.GetOrientation()), ell.GetCenter());
        return eq;
    }

    Eigen::Vector3d axes = Eigen::Vector3d::Zero();
    g2o::SE3Quat se3 = g2o::SE3Quat();
};


class VertexEllipsoidNoRot: public g2o::BaseVertex<6, Eigen::Matrix<double, 6, 1>>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        _estimate.head(3) = Eigen::Vector3d::Zero();
        _estimate.tail(3) = Eigen::Vector3d::Zero();
    }

    virtual void oplusImpl(const double *update) override {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> u(update);
        _estimate += u;
    }
    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}


};


class VertexEllipsoidQuat: public g2o::BaseVertex<9, EllipsoidQuat>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        _estimate = EllipsoidQuat();
    }

    virtual void oplusImpl(const double *update) override {
        _estimate.axes += Eigen::Map<const Eigen::Vector3d>(update);
        _estimate.se3 = g2o::SE3Quat::exp(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(update + 3)) * _estimate.se3;
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}
};

class EdgeEllipsoidProjection: public g2o::BaseUnaryEdge<1, double, VertexEllipsoidNoRot>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeEllipsoidProjection(const Eigen::Matrix<double, 3, 4>& P, const Ellipse& det_ell, const Eigen::Matrix3d& ellipsoid_R)
        : P_(P), det_ell_(det_ell), ellipsoid_R_(ellipsoid_R) {}

    virtual void computeError() override
    {
        const VertexEllipsoidNoRot* v = static_cast<VertexEllipsoidNoRot*>(_vertices[0]);
        Eigen::Matrix<double, 6, 1> ell = v->estimate();
        Ellipsoid ellipsoid(ell.head(3), Eigen::Matrix<double, 3, 3>::Identity(), ell.tail(3));

        Ellipse proj = ellipsoid.project(P_);

        _error[0] = gaussian_wasserstein_2d(det_ell_, proj);
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    private:
    Eigen::Matrix<double, 3, 4> P_;
    Ellipse det_ell_;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R_;
};

class EdgeEllipsoidProjectionQuat: public g2o::BaseUnaryEdge<1, double, VertexEllipsoidQuat>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeEllipsoidProjectionQuat(const Eigen::Matrix<double, 3, 4>& P, const Ellipse& det_ell, const Eigen::Matrix3d& ellipsoid_R)
        : P_(P), det_ell_(det_ell), ellipsoid_R_(ellipsoid_R) {}

    virtual void computeError() override
    {
        const VertexEllipsoidQuat* v = static_cast<VertexEllipsoidQuat*>(_vertices[0]);
        Ellipsoid ellipsoid = v->estimate().ToEllipsoid();

        Ellipse proj = ellipsoid.project(P_);
        _error[0] = gaussian_bhattacharrya_2d(det_ell_, proj);
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    private:
    Eigen::Matrix<double, 3, 4> P_;
    Ellipse det_ell_;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R_;
};


class EdgeEllipsoidMapPoint: public g2o::BaseUnaryEdge<1, double, VertexEllipsoidQuat>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeEllipsoidMapPoint(const Eigen::Vector3d& pt)
        : pt_(pt) {}

    virtual void computeError() override
    {
        const VertexEllipsoidQuat* v = static_cast<VertexEllipsoidQuat*>(_vertices[0]);
        Ellipsoid ellipsoid = v->estimate().ToEllipsoid();
        Eigen::Matrix4d C = ellipsoid.AsPrimal();
        double s = pt_.homogeneous().transpose() * C * pt_.homogeneous();
        _error[0] = s * 0.01;
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    private:
    Eigen::Vector3d pt_;
};



class EdgeEllipsoidProjectionQuatLevelSets: public g2o::BaseUnaryEdge<24, Eigen::Matrix<double, 24, 1>, VertexEllipsoidQuat>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeEllipsoidProjectionQuatLevelSets(const Eigen::Matrix<double, 3, 4>& P, const Ellipse& det_ell, const Eigen::Matrix3d& ellipsoid_R)
        : P_(P), det_ell_(det_ell), ellipsoid_R_(ellipsoid_R) {}

    virtual void computeError() override
    {
        const VertexEllipsoidQuat* v = static_cast<VertexEllipsoidQuat*>(_vertices[0]);
        Ellipsoid ellipsoid = v->estimate().ToEllipsoid();

        Ellipse proj = ellipsoid.project(P_);
        auto [total_error, errors, _, __] = ellipses_sampling_metric_ell(det_ell_, proj, 6, 4, 1.0);
        for (int i = 0; i < 24; ++i)
            _error[i] = errors[i];
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    private:
    Eigen::Matrix<double, 3, 4> P_;
    Ellipse det_ell_;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R_;
};

class EdgeEllipsoidProjectionQuatQBBox: public g2o::BaseUnaryEdge<4, Eigen::Vector4d, VertexEllipsoidQuat>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeEllipsoidProjectionQuatQBBox(const Eigen::Matrix<double, 3, 4>& P, const Ellipse& det_ell, const Eigen::Matrix3d& ellipsoid_R,
        unsigned int width, unsigned int height)
        : P_(P), det_ell_(det_ell), ellipsoid_R_(ellipsoid_R), w_(width), h_(height) {}

    virtual void computeError() override
    {
        const VertexEllipsoidQuat* v = static_cast<VertexEllipsoidQuat*>(_vertices[0]);
        Ellipsoid ellipsoid = v->estimate().ToEllipsoid();

        Ellipse proj = ellipsoid.project(P_);
        auto [status_det, bb_det] = find_on_image_bbox(det_ell_, w_, h_);
        auto [status_proj, bb_proj] = find_on_image_bbox(proj, w_, h_);
        _error = bb_det - bb_proj;
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    private:
    Eigen::Matrix<double, 3, 4> P_;
    Ellipse det_ell_;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R_;
    unsigned int w_, h_;
};



class EdgeEllipsoidProjectionQuatAlg: public g2o::BaseUnaryEdge<1, double, VertexEllipsoidQuat>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeEllipsoidProjectionQuatAlg(const Eigen::Matrix<double, 3, 4>& P, const Ellipse& det_ell, const Eigen::Matrix3d& ellipsoid_R)
        : P_(P), det_ell_(det_ell), ellipsoid_R_(ellipsoid_R) {}

    virtual void computeError() override
    {
        const VertexEllipsoidQuat* v = static_cast<VertexEllipsoidQuat*>(_vertices[0]);
        Ellipsoid ellipsoid = v->estimate().ToEllipsoid();

        Ellipse proj = ellipsoid.project(P_);
        _error[0] = algebraic_distance(det_ell_, proj);
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    private:
    Eigen::Matrix<double, 3, 4> P_;
    Ellipse det_ell_;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R_;
};



class EdgeEllipsoidProjectionQuatTangency: public g2o::BaseUnaryEdge<4, Eigen::Vector4d, VertexEllipsoidQuat>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeEllipsoidProjectionQuatTangency(const Eigen::Matrix<double, 3, 4>& P, const Ellipse& det_ell, const Eigen::Matrix3d& ellipsoid_R,
        unsigned int width, unsigned int height)
        : P_(P), det_ell_(det_ell), ellipsoid_R_(ellipsoid_R), w_(width), h_(height) {}

    virtual void computeError() override
    {
        const VertexEllipsoidQuat* v = static_cast<VertexEllipsoidQuat*>(_vertices[0]);
        Ellipsoid ellipsoid = v->estimate().ToEllipsoid();
        BBox2 bb = det_ell_.ComputeBbox();

        Eigen::Vector3d x0(bb[0], bb[1], 1);
        Eigen::Vector3d x1(bb[2], bb[1], 1);
        Eigen::Vector3d x2(bb[0], bb[3], 1);
        Eigen::Vector3d x3(bb[2], bb[3], 1);
        Eigen::Vector3d l01 = x0.cross(x1);
        l01 /= l01[2];
        Eigen::Vector3d l02 = x0.cross(x2);
        l02 /= l02[2];
        Eigen::Vector3d l13 = x1.cross(x3);
        l13 /= l13[2];
        Eigen::Vector3d l23 = x2.cross(x3);
        l23 /= l23[2];
        auto d01 = tangency_segment_ellipsoid(l01, ellipsoid, P_);
        auto d02 = tangency_segment_ellipsoid(l02, ellipsoid, P_);
        auto d13 = tangency_segment_ellipsoid(l13, ellipsoid, P_);
        auto d23 = tangency_segment_ellipsoid(l23, ellipsoid, P_);
        std::cout <<"===========================\n";

        std::cout << d01 << "\n";
        std::cout << d02 << "\n";
        std::cout << d13 << "\n";
        std::cout << d23 << "\n";
        std::cout <<"===========================\n";
        _error[0] = d01;
        _error[1] = d02;
        _error[2] = d13;
        _error[3] = d23;
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    private:
    Eigen::Matrix<double, 3, 4> P_;
    Ellipse det_ell_;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R_;
    unsigned int w_, h_;
};



class EdgeEllipsoidProjectionBbox: public g2o::BaseUnaryEdge<4, Eigen::Vector4d, VertexEllipsoidNoRot>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeEllipsoidProjectionBbox(const Eigen::Matrix<double, 3, 4>& P, const Ellipse& det_ell, const Eigen::Matrix3d& ellipsoid_R)
        : P_(P), det_ell_(det_ell), ellipsoid_R_(ellipsoid_R) {}

    virtual void computeError() override
    {
        const VertexEllipsoidNoRot* v = static_cast<VertexEllipsoidNoRot*>(_vertices[0]);
        Eigen::Matrix<double, 6, 1> ell = v->estimate();
        Ellipsoid ellipsoid(ell.head(3), Eigen::Matrix<double, 3, 3>::Identity(), ell.tail(3));

        Ellipse proj = ellipsoid.project(P_);
        auto bb_proj = proj.ComputeBbox();
        auto bb_det = det_ell_.ComputeBbox();
        _error = bb_det - bb_proj;
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    private:
    Eigen::Matrix<double, 3, 4> P_;
    Ellipse det_ell_;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R_;
};




class  EdgeSE3ProjectEllipsoidOnlyPose: public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectEllipsoidOnlyPose(const Ellipse& ell2d, const Ellipsoid& ell3d, const Eigen::Matrix3d& K) : ellipse_(ell2d), ellipsoid_(ell3d), K_(K) {}

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        g2o::SE3Quat se3_quat = v1->estimate();
        Eigen::Vector3d t = se3_quat.translation();
        Eigen::Matrix3d R = se3_quat.rotation().toRotationMatrix();
        Matrix34d P;
        P << R, t;
        P = K_ * P;
        
        auto proj = ellipsoid_.project(P);
        _error[0] = gaussian_wasserstein_2d(ellipse_, proj);
    }


    Ellipse ellipse_;
    Ellipsoid ellipsoid_;
    Eigen::Matrix3d K_;
};



class  EdgeSE3ProjectEllipsoid: public g2o::BaseBinaryEdge<1, double, VertexEllipsoidQuat, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectEllipsoid(const Ellipse& ell2d, const Eigen::Matrix3d& K) : ellipse_(ell2d), K_(K) {}

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    void computeError()  {
        const VertexEllipsoidQuat* v1 = static_cast<const VertexEllipsoidQuat*>(_vertices[0]);
        const g2o::VertexSE3Expmap* v2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);

        Ellipsoid ellipsoid = v1->estimate().ToEllipsoid();

        g2o::SE3Quat se3_quat = v2->estimate();
        Eigen::Vector3d t = se3_quat.translation();
        Eigen::Matrix3d R = se3_quat.rotation().toRotationMatrix();
        Matrix34d P;
        P << R, t;
        P = K_ * P;
        
        Ellipse proj = ellipsoid.project(P);
        _error[0] = gaussian_bhattacharrya_2d(ellipse_, proj) * 10;
    }

    Ellipse ellipse_;
    Eigen::Matrix3d K_;
};



class  EdgeSE3ProjectEllipsoidCenter: public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectEllipsoidCenter(const Ellipse& ell2d, const Eigen::Matrix3d& K) : ellipse_(ell2d), K_(K) {}

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    void computeError()  {
        const g2o::VertexSBAPointXYZ* v1 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
        const g2o::VertexSE3Expmap* v2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);

        Eigen::Vector3d p = v1->estimate();
        g2o::SE3Quat se3_quat = v2->estimate();
        Eigen::Vector3d t = se3_quat.translation();
        Eigen::Matrix3d R = se3_quat.rotation().toRotationMatrix();
        Matrix34d P;
        P << R, t;
        P = K_ * P;

        Eigen::Vector3d p2 = K_ * (R * p + t);
        Eigen::Vector2d c = ellipse_.GetCenter();
        _error[0] = (c[0] - p2[0] / p2[2]) * 1;
        _error[1] = (c[1] - p2[1] / p2[2]) * 1;
        std::cout << "error = " << _error[0] << " " << _error[1] << "\n";
    }

    Ellipse ellipse_;
    Eigen::Matrix3d K_;
};


} //namespace ORB_SLAM3

#endif // OPTIMIZER_OBJECT_H
