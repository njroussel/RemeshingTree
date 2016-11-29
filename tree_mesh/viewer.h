//=============================================================================
//
//   Code framework for the lecture
//
//   "Digital 3D Geometry Processing"
//
//   Gaspard Zoss
//
//   Copyright (C) 2016 by Computer Graphics and Geometry Laboratory,
//         EPF Lausanne
//
//-----------------------------------------------------------------------------
#include <nanogui/opengl.h>
#include <nanogui/glutil.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/popupbutton.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/textbox.h>
#include <nanogui/tabwidget.h>
#include <nanogui/combobox.h>
#include "mesh_processing.h"

#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::pair;
using std::to_string;
using std::min;
using std::max;
using namespace surface_mesh;
using namespace nanogui;

class Viewer : public nanogui::Screen {
public:

    void refresh_mesh();
    void refresh_trackball_center();
    Viewer();
    ~Viewer();

    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers);
    virtual void draw(NVGcontext *ctx);
    Vector2f getScreenCoord();
    virtual void drawContents();

    bool scrollEvent(const Vector2i &p, const Vector2f &rel);
    bool mouseMotionEvent(const Vector2i &p, const Vector2i &rel, int button, int modifiers);
    bool mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers);

private:
    void initShaders();
    void computeCameraMatrices(Eigen::Matrix4f &model,
                               Eigen::Matrix4f &view,
                               Eigen::Matrix4f &proj);

    struct CameraParameters {
        nanogui::Arcball arcball;
        float zoom = 1.0f, viewAngle = 45.0f;
        float dnear = 0.05f, dfar = 100.0f;
        Eigen::Vector3f eye = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
        Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        Eigen::Vector3f modelTranslation = Eigen::Vector3f::Zero();
        Eigen::Vector3f modelTranslation_start = Eigen::Vector3f::Zero();
        float modelZoom = 1.0f;
    };

    CameraParameters camera_;
    bool translate_ = false;
    Vector2i translateStart_ = Vector2i(0, 0);

    // Variables for the viewer
    nanogui::GLShader shader_;
    nanogui::GLShader shaderNormals_;
    nanogui::Window *window_;

    mesh_processing::MeshProcessing* mesh_;

    enum COLOR_MODE : int { NORMAL = 0, VALENCE = 1, CURVATURE = 2 };
    enum CURVATURE_TYPE : int { UNIMEAN = 2, LAPLACEBELTRAMI = 3, GAUSS = 4 };

    // Boolean for the viewer
    bool wireframe_ = false;
    bool normals_ = false;

    CURVATURE_TYPE curvature_type = UNIMEAN;
    COLOR_MODE color_mode = NORMAL;
    mesh_processing::REMESHING_TYPE remeshing_type = mesh_processing::AVERAGE;

    PopupButton *popupCurvature;
    FloatBox<float>* coefTextBox;
    IntBox<int>* iterationTextBox;

};
