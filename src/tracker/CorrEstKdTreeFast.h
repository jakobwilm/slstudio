#ifndef CORRESTKDTREEFAST_H
#define CORRESTKDTREEFAST_H

// CorrEstKdTreeFast.h
// Improves on the standard CorrespondenceEstimation class.

#include <pcl/registration/correspondence_estimation.h>

using namespace pcl::registration;
using namespace pcl;

template <typename PointSource, typename PointTarget, typename Scalar = float>
class CorrEstKdTreeFast : public CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>{

    public:
        typedef boost::shared_ptr< CorrEstKdTreeFast<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr< const CorrEstKdTreeFast<PointSource, PointTarget, Scalar> > ConstPtr;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_reciprocal_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initComputeReciprocal;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;
        using PCLBase<PointSource>::deinitCompute;

        CorrEstKdTreeFast(){
            corr_name_  = "CorrEstKdTreeFast";
        }
        ~CorrEstKdTreeFast(){}

        // Setters for general parameters
        //void setDistThreshold (const float depth_threshold){dist_threshold_ = depth_threshold;}

        // Core functions
        //bool initCompute();
        void determineCorrespondences(Correspondences &correspondences, double max_distance = std::numeric_limits<double>::max());
        void determineReciprocalCorrespondences(Correspondences &correspondences, double max_distance = std::numeric_limits<double>::max());

};


// Templated class methods have to be implemented in the header...

template <typename PointSource, typename PointTarget, typename Scalar>
void CorrEstKdTreeFast<PointSource, PointTarget, Scalar>::determineCorrespondences(pcl::Correspondences &correspondences, double max_distance){

    if (!initCompute())
        return;

    tree_->setEpsilon(1.0);

    double max_dist_sqr = max_distance * max_distance;

    typedef typename pcl::traits::fieldList<PointTarget>::type FieldListTarget;
    correspondences.resize(indices_->size());

    std::vector<int> index (1);
    std::vector<float> dist_sqr(1);
    pcl::Correspondence corr;
    unsigned int nr_valid_correspondences = 0;

    // Check if the template types are the same. If true, avoid a copy.
    // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
    if (isSamePointType<PointSource, PointTarget>()){

        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx){

            if(isFinite(input_->points[*idx])){

                tree_->nearestKSearch(input_->points[*idx], 1, index, dist_sqr);
                if (dist_sqr[0] > max_dist_sqr)
                    continue;

                corr.index_query = *idx;
                corr.index_match = index[0];
                corr.distance = dist_sqr[0];
                correspondences[nr_valid_correspondences++] = corr;

            }
        }
    }else{
        PointTarget pt;

        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin(); idx != indices_->end(); ++idx){

            if(isFinite (input_->points[*idx])){
                // Copy the source data to a target PointTarget format so we can search in the tree
                pcl::for_each_type<FieldListTarget>(pcl::NdConcatenateFunctor<PointSource, PointTarget> (input_->points[*idx],pt));

                tree_->nearestKSearch(pt, 1, index, dist_sqr);
                if (dist_sqr[0] > max_dist_sqr)
                    continue;

                corr.index_query = *idx;
                corr.index_match = index[0];
                corr.distance = dist_sqr[0];
                correspondences[nr_valid_correspondences++] = corr;
            }
        }
    }
    correspondences.resize (nr_valid_correspondences);
    deinitCompute ();
}

template <typename PointSource, typename PointTarget, typename Scalar>
void CorrEstKdTreeFast<PointSource, PointTarget, Scalar>::determineReciprocalCorrespondences(pcl::Correspondences &correspondences, double max_distance){

    if (!initCompute ())
        return;

    typedef typename pcl::traits::fieldList<PointSource>::type FieldListSource;
    typedef typename pcl::traits::fieldList<PointTarget>::type FieldListTarget;
    typedef typename pcl::intersect<FieldListSource, FieldListTarget>::type FieldList;

    // setup tree for reciprocal search
    // Set the internal point representation of choice
    if (!initComputeReciprocal())
        return;
    double max_dist_sqr = max_distance * max_distance;

    correspondences.resize (indices_->size());
    std::vector<int> index (1);
    std::vector<float> distance (1);
    std::vector<int> index_reciprocal (1);
    std::vector<float> distance_reciprocal (1);
    pcl::Correspondence corr;
    unsigned int nr_valid_correspondences = 0;
    int target_idx = 0;

    // Check if the template types are the same. If true, avoid a copy.
    // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
    if (isSamePointType<PointSource, PointTarget> ())
    {
        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
        {
            tree_->nearestKSearch (input_->points[*idx], 1, index, distance);
            if (distance[0] > max_dist_sqr)
                continue;

            target_idx = index[0];

            tree_reciprocal_->nearestKSearch (target_->points[target_idx], 1, index_reciprocal, distance_reciprocal);
            if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0])
                continue;

            corr.index_query = *idx;
            corr.index_match = index[0];
            corr.distance = distance[0];
            correspondences[nr_valid_correspondences++] = corr;
        }
    }
    else
    {
        PointTarget pt_src;
        PointSource pt_tgt;

        // Iterate over the input set of source indices
        for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
        {
            // Copy the source data to a target PointTarget format so we can search in the tree
            pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointSource, PointTarget> (
                                                input_->points[*idx],
                                            pt_src));

            tree_->nearestKSearch (pt_src, 1, index, distance);
            if (distance[0] > max_dist_sqr)
                continue;

            target_idx = index[0];

            // Copy the target data to a target PointSource format so we can search in the tree_reciprocal
            pcl::for_each_type<FieldList> (pcl::NdConcatenateFunctor <PointTarget, PointSource> (
                                               target_->points[target_idx],
                                               pt_tgt));

            tree_reciprocal_->nearestKSearch (pt_tgt, 1, index_reciprocal, distance_reciprocal);
            if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0])
                continue;

            corr.index_query = *idx;
            corr.index_match = index[0];
            corr.distance = distance[0];
            correspondences[nr_valid_correspondences++] = corr;
        }
    }
    correspondences.resize (nr_valid_correspondences);
    deinitCompute ();
}

#endif
