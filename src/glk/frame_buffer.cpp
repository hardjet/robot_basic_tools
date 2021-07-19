#include "glk/texture.hpp"
#include "glk/frame_buffer.hpp"

# include <iostream>
namespace glk {

FrameBuffer::FrameBuffer(const Eigen::Vector2i& size) : width(size[0]), height(size[1]) {                 // 接收从GLCanvas传入的窗口宽和高
  color_buffers.push_back(std::make_shared<Texture>(size, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE));           // 按照RGBA成分存储纹理单元，GL_UNSIGNED_BYTE是GL_RGBA的单元/单位？
  depth_buffer = std::make_shared<Texture>(size, GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT);    // GL_DEPTH_COMPONENT32F = 深度格式（浮点型）

  glGenFramebuffers(1, &frame_buffer);                        // void glGenFramebuffers(GLsizei n, GLuint * framebuffers) - n(int): 指定要生成的帧缓冲区对象名称的数量 ; framebuffers 指定存储生成的帧缓冲区对象名称的数组
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);            // void glBindFramebuffer（GLenum target, GLuint framebuffer）- 绑定一个命名的帧缓冲区对象

  // void glFramebufferTexture2D（GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level）;
  // target = 指定帧缓冲目标。 符号常量必须是GL_FRAMEBUFFER
  // attachment = 指定应附加纹理图像的附着点。 必须是以下符号常量之一：GL_COLOR_ATTACHMENT0，GL_DEPTH_ATTACHMENT或GL_STENCIL_ATTACHMENT
  // textarget = 指定纹理目标。 必须是以下符号常量之一：GL_TEXTURE_2D，GL_TEXTURE_CUBE_MAP_POSITIVE_X，GL_TEXTURE_CUBE_MAP_NEGATIVE_X
  //                                               GL_TEXTURE_CUBE_MAP_POSITIVE_Y，GL_TEXTURE_CUBE_MAP_NEGATIVE_Y，GL_TEXTURE_CUBE_MAP_POSITIVE_Z或GL_TEXTURE_CUBE_MAP_NEGATIVE_Z
  // texture = 指定要附加图像的纹理对象
  // level = 指定要附加的纹理图像的mipmap级别，该级别必须为0
  // glFramebufferTexture2D将texture和level指定的纹理图像附加为当前绑定的帧缓冲区对象的逻辑缓冲区之一。 attachment指定是否应将纹理图像附加到帧缓冲对象的颜色，深度或模板缓冲区。 纹理图像不可以附加到默认（名称为0）的帧缓冲对象
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_buffers[0]->id(), 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_buffer->id(), 0);

  GLenum color_attachments[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
  glDrawBuffers(static_cast<int>(color_buffers.size()), color_attachments);       // void glDrawBuffers( GLsizei n, const GLenum *bufs) 指定要绘制到的颜色缓冲区的列表 - n = 指定缓冲区中的缓冲区数, bufs = 指向符号常量数组，这些符号常量指定将片段颜色或数据值写入其中的缓冲区

  glBindFramebuffer(GL_FRAMEBUFFER, 0);        // 保留值零以表示由窗口系统提供的默认帧缓冲区
}

FrameBuffer::~FrameBuffer() { glDeleteFramebuffers(1, &frame_buffer); }

void FrameBuffer::bind() {
  glGetIntegerv(GL_VIEWPORT, viewport);       // 第一个参数，表示你要得到什么状态的值，第二个参数即输出这个值 ； 打印出来后x和y始终是0，后两位会根据窗口拖动大小改变
//  std::cout << "----- FrameBuffer::bind() ..... viewport = " << viewport[0] << ", " << viewport[1] << ", " << viewport[2] << ", " << viewport[3] << std::endl;

  glViewport(0, 0, width, height);            // 调用glViewPort函数来决定视见区域，告诉OpenGL应把渲染之后的图形绘制在窗体的哪个部位；x,y 以像素为单位，指定了窗口的左下角位置，width,height表示视口矩形的宽度和高度，根据窗口的实时变化重绘窗口
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);    // 绑定一个命名的帧缓冲区对象
}

void FrameBuffer::unbind() const {
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
}

void FrameBuffer::add_color_buffer(GLuint internal_format, GLuint format, GLuint type) {      // 添加到color_buffers这个vector，里面装的是指向texture的指针
  glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

  color_buffers.push_back(std::make_shared<Texture>(color_buffers.front()->size(), internal_format, format, type));
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + color_buffers.size() - 1, GL_TEXTURE_2D,
                         color_buffers.back()->id(), 0);

  std::vector<GLuint> color_attachments(color_buffers.size());
  for (int i = 0; i < color_buffers.size(); i++) {
    color_attachments[i] = GL_COLOR_ATTACHMENT0 + i;
  }
  glDrawBuffers(static_cast<int>(color_buffers.size()), color_attachments.data());

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

}  // namespace glk
