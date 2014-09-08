#include "CorrRejectOrgBoundFast.h"
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>


#include <QTime>

void CorrRejectOrgBoundFast::recomputeTargetBoundary(){

//    QTime time;
//    time.start();

    pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr target = boost::static_pointer_cast<pcl::registration::DataContainer<pcl::PointXYZRGBNormal, pcl::PointNormal> >(data_container_)->getInputTarget();

    if (!target->isOrganized ()){
      PCL_ERROR ("[CorrRejectOrgBoundFast::getRemainingCorrespondences] The target cloud is not organized.\n");
      return;
    }

    // Make boundary label cloud same size as target
    boundary.reset(new pcl::PointCloud<pcl::Label>);
    boundary->width = target->width;
    boundary->height = target->height;
    boundary->points.resize(target->size());

    // Assign boundary label to boundary points
    for(int y = 0; y < (int)boundary->height; y++){
        for(int x = 0; x < (int)boundary->width; x++){

            int nan_count_tgt = 0;

            for (int x_d = -window_size_/2; x_d <= window_size_/2; ++x_d){
                for (int y_d = -window_size_/2; y_d <= window_size_/2; ++y_d){
                    if (x + x_d >= 0 && x + x_d < target->width && y + y_d >= 0 && y + y_d < target->height){
                        if (!pcl_isfinite(target->at(x + x_d, y + y_d).z) || fabs(target->at(x, y).z - target->at(x + x_d, y + y_d).z) > depth_step_threshold_)
                            nan_count_tgt ++;
                    }
                }
            }

            if (nan_count_tgt >= boundary_nans_threshold_)
                boundary->at(x,y).label = 1;
            else
                boundary->at(x,y).label = 0;

        }
    }

//    std::cout << "CorrRejectOrgBoundFast recomputeTargetBoundary(): " << time.elapsed() << "ms" << std::endl;

    //pcl::io::savePCDFileASCII("boundary.pcd", *boundary);

}

void CorrRejectOrgBoundFast::getRemainingCorrespondences(const pcl::Correspondences& original_correspondences, pcl::Correspondences& remaining_correspondences){

//    QTime time;
//    time.start();

    remaining_correspondences.resize(original_correspondences.size());

    unsigned int num_remaining_correspondences = 0;

    for (size_t c_i = 0; c_i < original_correspondences.size (); ++c_i){

        size_t target_idx = original_correspondences[c_i].index_match;

        if(boundary->points[target_idx].label == 0)
            remaining_correspondences[num_remaining_correspondences++] = original_correspondences[c_i];
    }

    remaining_correspondences.resize(num_remaining_correspondences);

//    std::cout << "CorrRejectOrgBoundFast: " << time.elapsed() << "ms" << std::endl;
}
