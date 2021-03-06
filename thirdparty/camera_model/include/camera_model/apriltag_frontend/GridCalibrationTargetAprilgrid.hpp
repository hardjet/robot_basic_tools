#ifndef ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HPP
#define ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HPP

#include <Eigen/Core>
#include <boost/serialization/export.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

// April tags detector and various tag families
#include "camera_model/apriltag/TagDetector.h"
//#include "apriltags/Tag16h5.h"
//#include "apriltags/Tag25h7.h"
//#include "apriltags/Tag25h9.h"
//#include "apriltags/Tag36h9.h"
#include "camera_model/apriltag/Tag36h11.h"
#include "GridCalibrationTargetBase.hpp"

namespace aslam {
namespace cameras {

class GridCalibrationTargetAprilgrid : public GridCalibrationTargetBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef boost::shared_ptr<GridCalibrationTargetAprilgrid> Ptr;
  typedef boost::shared_ptr<const GridCalibrationTargetAprilgrid> ConstPtr;

  // target extraction options
  struct AprilgridOptions {
    AprilgridOptions()
        : doSubpixRefinement(true),
          maxSubpixDisplacement2(1.5),
          showExtractionVideo(false),
          minTagsForValidObs(2),
          minBorderDistance(4.0),
          blackTagBorder(2){};

    // options
    /// \brief subpixel refinement of extracted corners
    bool doSubpixRefinement;

    /// \brief max. displacement squarred in subpixel refinement  [px^2]
    double maxSubpixDisplacement2;

    /// \brief show video during extraction
    bool showExtractionVideo;

    /// \brief min. number of tags for a valid observation
    unsigned int minTagsForValidObs;

    /// \brief min. distance form image border for valid points [px]
    double minBorderDistance;

    /// \brief size of black border around the tag code bits (in pixels)
    unsigned int blackTagBorder;

    /// \brief Serialization support
    enum { CLASS_SERIALIZATION_VERSION = 1 };
    BOOST_SERIALIZATION_SPLIT_MEMBER();
    template <class Archive>
    void save(Archive& ar, const unsigned int /*version*/) const {
      ar << BOOST_SERIALIZATION_NVP(doSubpixRefinement);
      ar << BOOST_SERIALIZATION_NVP(maxSubpixDisplacement2);
      ar << BOOST_SERIALIZATION_NVP(showExtractionVideo);
      ar << BOOST_SERIALIZATION_NVP(minTagsForValidObs);
      ar << BOOST_SERIALIZATION_NVP(minBorderDistance);
      ar << BOOST_SERIALIZATION_NVP(blackTagBorder);
    }
    template <class Archive>
    void load(Archive& ar, const unsigned int /*version*/) {
      ar >> BOOST_SERIALIZATION_NVP(doSubpixRefinement);
      ar >> BOOST_SERIALIZATION_NVP(maxSubpixDisplacement2);
      ar >> BOOST_SERIALIZATION_NVP(showExtractionVideo);
      ar >> BOOST_SERIALIZATION_NVP(minTagsForValidObs);
      ar >> BOOST_SERIALIZATION_NVP(minBorderDistance);
      ar >> BOOST_SERIALIZATION_NVP(blackTagBorder);
    }
  };

  /// \brief initialize based on checkerboard geometry
  GridCalibrationTargetAprilgrid(size_t tagRows, size_t tagCols, double tagSize, double tagSpacing,
                                 const AprilgridOptions& options = AprilgridOptions());

  ~GridCalibrationTargetAprilgrid() override {}

  /// \brief extract the calibration target points from an image and write to an observation
  bool computeObservation(const cv::Mat& image, std::vector<cv::Point2f>& points2ds,
                          std::vector<bool>& outCornerObserved) const;

 public:
  // 设置尺寸
  void set_params(double tagSize, double tagSpacing) {
    _tagSize = tagSize;
    _tagSpacing = tagSpacing;
  }

  // 获取tagSize
  double get_tagsize() const { return _tagSize; }

  // 更新参数
  void updata_params() { createGridPoints(); }

  /**
   *
   * @param src_image 待检测的图像
   * @param dst_image 有检测结果的图像
   * @param objectPoints 检测到的角点空间坐标
   * @param imagePoints 检测到的角点图像坐标
   * @return
   */
  bool computeObservation(const cv::Mat& src_image, cv::Mat& dst_image, std::vector<cv::Point3f>& objectPoints,
                          std::vector<cv::Point2f>& imagePoints) const;

 private:
  /// \brief initialize the object
  void initialize();

  /// \brief initialize the grid with the points
  void createGridPoints();

  /// \brief size of a tag [m]
  double _tagSize;

  /// \brief space between tags (tagSpacing [m] = tagSize * tagSpacing)
  double _tagSpacing;

  /// \brief target extraction options
  AprilgridOptions _options;

  // create a detector instance
  AprilTags::TagCodes _tagCodes;
  boost::shared_ptr<AprilTags::TagDetector> _tagDetector;

  ///////////////////////////////////////////////////
  // Serialization support
  ///////////////////////////////////////////////////
 public:
  enum { CLASS_SERIALIZATION_VERSION = 1 };
  BOOST_SERIALIZATION_SPLIT_MEMBER()

  // serialization ctor
  GridCalibrationTargetAprilgrid();

 protected:
  friend class boost::serialization::access;

  template <class Archive>
  void save(Archive& ar, const unsigned int /* version */) const {
    boost::serialization::void_cast_register<GridCalibrationTargetAprilgrid, GridCalibrationTargetBase>(
        static_cast<GridCalibrationTargetAprilgrid*>(NULL), static_cast<GridCalibrationTargetBase*>(NULL));
    ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar << BOOST_SERIALIZATION_NVP(_tagSize);
    ar << BOOST_SERIALIZATION_NVP(_tagSpacing);
    ar << BOOST_SERIALIZATION_NVP(_options);
  }
  template <class Archive>
  void load(Archive& ar, const unsigned int /* version */) {
    boost::serialization::void_cast_register<GridCalibrationTargetAprilgrid, GridCalibrationTargetBase>(
        static_cast<GridCalibrationTargetAprilgrid*>(NULL), static_cast<GridCalibrationTargetBase*>(NULL));
    ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(GridCalibrationTargetBase);
    ar >> BOOST_SERIALIZATION_NVP(_tagSize);
    ar >> BOOST_SERIALIZATION_NVP(_tagSpacing);
    ar >> BOOST_SERIALIZATION_NVP(_options);
    initialize();
  }
};

}  // namespace cameras
}  // namespace aslam

BOOST_CLASS_EXPORT_KEY(aslam::cameras::GridCalibrationTargetAprilgrid)

#endif /* ASLAM_GRID_CALIBRATION_TARGET_APRILGRID_HPP */
