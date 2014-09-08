#ifndef CORRREJECTORGBOUNDFAST_H
#define CORRREJECTORGBOUNDFAST_H

// CorrRejectOrgBoundFast.h
// Improves on CorrespondenceRejectionOrganizedBoundary.

#include <pcl/registration/correspondence_rejection.h>

using namespace pcl::registration;
using namespace pcl;

class CorrRejectOrgBoundFast : public CorrespondenceRejector{

    using CorrespondenceRejector::input_correspondences_;
    using CorrespondenceRejector::rejection_name_;
    using CorrespondenceRejector::getClassName;

    public:

        typedef boost::shared_ptr<CorrRejectOrgBoundFast> Ptr;
        typedef boost::shared_ptr<const CorrRejectOrgBoundFast> ConstPtr;

        CorrRejectOrgBoundFast(){
            rejection_name_ = "CorrRejectOrgBoundFast";
        }

        virtual ~CorrRejectOrgBoundFast(){}

        // Add setters for specific parameters
        inline void setNumberOfBoundaryNaNs(int val){boundary_nans_threshold_ = val;}
        inline void setWindowSize(int val){window_size_ = val;}
        inline void setDepthStepThreshhold(float val){depth_step_threshold_ = val;}

        void getRemainingCorrespondences(const pcl::Correspondences& original_correspondences, pcl::Correspondences& remaining_correspondences);

        template <typename PointT> inline void
        setInputSource (const typename pcl::PointCloud<PointT>::ConstPtr &cloud){
            if (!data_container_)
                data_container_.reset (new DataContainer<PointT>);
            boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputSource(cloud);
        }

        template <typename PointT> inline void
        setInputTarget (const typename pcl::PointCloud<PointT>::ConstPtr &target){
            if (!data_container_)
                data_container_.reset (new DataContainer<PointT>);
            boost::static_pointer_cast<DataContainer<PointT> > (data_container_)->setInputTarget(target);

            recomputeTargetBoundary();
        }

        void applyRejection (pcl::Correspondences &correspondences){
            getRemainingCorrespondences(*input_correspondences_, correspondences);
        }

    private:
        void recomputeTargetBoundary();

        int boundary_nans_threshold_;
        int window_size_;
        float depth_step_threshold_;

        typedef boost::shared_ptr<pcl::registration::DataContainerInterface> DataContainerPtr;
        DataContainerPtr data_container_;

        pcl::PointCloud<pcl::Label>::Ptr boundary;
};

#endif // CORRREJECTORGBOUNDFAST_H
