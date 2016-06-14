#ifndef CORRESTORGPROJFAST_H
#define CORRESTORGPROJFAST_H

// CorrEstOrgProjFast.h
// Improves on the CorrespondenceEstimationOrganizedProjection class.

#include <pcl/registration/correspondence_estimation.h>

using namespace pcl::registration;
using namespace pcl;

template <typename PointSource, typename PointTarget, typename Scalar = float>
class CorrEstOrgProjFast : public CorrespondenceEstimationBase <PointSource, PointTarget, Scalar>{

    public:
        typedef boost::shared_ptr< CorrEstOrgProjFast<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr< const CorrEstOrgProjFast<PointSource, PointTarget, Scalar> > ConstPtr;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::force_no_recompute_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
        using PCLBase<PointSource>::deinitCompute;

        CorrEstOrgProjFast(): projection_matrix_(Eigen::Matrix3f::Identity()){
            corr_name_  = "CorrEstOrgProjFast";
        }
		boost::shared_ptr< CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > clone() const {
            boost::shared_ptr< CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> >
                    val(new CorrEstOrgProjFast<PointSource, PointTarget, Scalar>);
            *val = *this;
            return val;
        }
        ~CorrEstOrgProjFast(){}

        // Setters for general parameters
        void setProjectionMatrix(Eigen::Matrix3f &projection_matrix){projection_matrix_ = projection_matrix;}

        // Core functions
        bool initCompute();
        void determineCorrespondences(Correspondences &correspondences, double max_distance);
        void determineReciprocalCorrespondences(Correspondences &correspondences, double max_distance){determineCorrespondences(correspondences, max_distance);}

    private:
        Eigen::Matrix3f projection_matrix_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


// Templated class methods have to be implemented in the header...

template <typename PointSource, typename PointTarget, typename Scalar>
bool CorrEstOrgProjFast<PointSource, PointTarget, Scalar>::initCompute(){

    // Make sure kdtree is not computed
    force_no_recompute_ = true;

    if (!CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute())
        return (false);

    // Check if target cloud is organized
    if (!target_->isOrganized()){
        PCL_WARN ("[%s::initCompute] Target cloud is not organized.\n", corr_name_.c_str());
        return (false);
    }

    return (true);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void CorrEstOrgProjFast<PointSource, PointTarget, Scalar>::determineCorrespondences(pcl::Correspondences &correspondences, double max_distance){

//    QTime time;
//    time.start();

    if (!initCompute())
        return;

    correspondences.resize(indices_->size());
    size_t c_index = 0;

    for (std::vector<int>::const_iterator src_it = indices_->begin (); src_it != indices_->end (); ++src_it){

        if (isFinite (input_->points[*src_it])){

            Eigen::Vector4f p_src(input_->points[*src_it].getVector4fMap());
            Eigen::Vector3f p_src3(p_src[0], p_src[1], p_src[2]);
            Eigen::Vector3f uv(projection_matrix_ * p_src3);

            // Check if the point is behind the camera
            if (uv[2] <= 0)
                continue;

            int u = static_cast<int> (uv[0]/uv[2] + 0.5);
            int v = static_cast<int> (uv[1]/uv[2] + 0.5);

            if (u >= 0 && u < static_cast<int>(target_->width) && v >= 0 && v < static_cast<int>(target_->height)){

                const PointTarget &pt_tgt = target_->at(u, v);

                if (!isFinite(pt_tgt))
                    continue;

                float depth_dist = fabs(uv[2] - pt_tgt.z);
                if (depth_dist > max_distance)
                    continue;

                correspondences[c_index++] =  pcl::Correspondence(*src_it, v*target_->width+u, static_cast<float>(depth_dist));

            }
        }

    }

    correspondences.resize(c_index);

    deinitCompute();

//    std::cout << "CorrEstOrgProjFast: " << time.elapsed() << "ms" << std::endl;
}



#endif // CORRESTORGPROJFAST_H
