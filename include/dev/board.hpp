#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
//  前置声明
namespace glk {
class GLSLShader;
}

namespace dev {
class SensorManager;
}  // namespace dev
namespace dev {

 class Board{
  public:
   explicit Board(std::string& data_path)
   {
   }
   virtual ~Board() = default;
   /**
     * @brief 画ui
     */
   virtual void draw_ui() = 0;
   /**
    * 画3d显示部分
    * @param shader
    */
   virtual void draw_gl(glk::GLSLShader& shader) = 0;
   /**
    * 计算图像的角点和对应的3D点
    * @param
    */
   virtual void computeObservation();
   /**
    * @brief 打开显示ui开关
    */
   void show() { b_show_window_ = true; };
   /**
    * @brief 打开显示3d开关
    */
   void show_3d() { b_show_3d_ = true; };
   /**
    * 获取sensor位姿
    * @param pose
    */
   const Eigen::Matrix4f& get_pose() const { return T_; }
   /**
    * 设置sensor位姿
    * @param new_pose
    */
   void set_pose(const Eigen::Matrix4f& new_pose) { T_ = new_pose; }

  private:
   /**
    * 更新棋盘格在3d场景中的显示
    */
   virtual void update_board_edges();

  private:
   // 图片texture id
   unsigned int texture_id_{0};
   // 图像宽度
   int img_width_{0};
   // 图像高度
   int img_height_{0};
   // tagSize
   double tag_size_{0.03}; //单位mm
   // tagSpacing
   double tag_spacing_{0.3}; //标准棋盘格标定板无此成员
   // 板子长度
   double board_lenght_{};
   // 板子高度
   double board_height_{};
   // 是否显示ui窗口
   bool b_show_window_{false};
   // 是否显示3d内容
   bool b_show_3d_{false};
   // 设备在世界坐标系下的位姿
   Eigen::Matrix4f T_{};
 };

}