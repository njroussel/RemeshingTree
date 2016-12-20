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
#include "viewer/viewer.h"

bool Viewer::keyboardEvent(int key, int scancode, int action, int modifiers) {
    if (Screen::keyboardEvent(key, scancode, action, modifiers)) {
        return true;
    }
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        setVisible(false);
        return true;
    }
    return false;
}

void Viewer::draw(NVGcontext *ctx) {
    /* Draw the user interface */
    Screen::draw(ctx);
}

Vector2f Viewer::getScreenCoord() {
    Vector2i pos = mousePos();
    return Vector2f(2.0f * (float)pos.x() / width() - 1.0f,
            1.0f - 2.0f * (float)pos.y() / height());
}

void Viewer::drawContents() {
    using namespace nanogui;

    /* Draw the window contents using OpenGL */
    shader_.bind();

    Eigen::Matrix4f model, view, proj;
    computeCameraMatrices(model, view, proj);

    Matrix4f mv = view*model;
    Matrix4f p = proj;

    /* MVP uniforms */
    shader_.setUniform("MV", mv);
    shader_.setUniform("P", p);

    // Setup OpenGL (making sure the GUI doesn't disable these
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    // Render everything
    if (wireframe_) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    Vector3f colors(0.98, 0.59, 0.04);
    shader_.setUniform("intensity", colors);
    if (color_mode == CURVATURE) {
        shader_.setUniform("color_mode", int(curvature_type));
    } else {
        shader_.setUniform("color_mode", int(color_mode));
    }
    shader_.drawIndexed(GL_TRIANGLES, 0, mesh_->get_number_of_face());

    if (wireframe_) {
        glDisable(GL_POLYGON_OFFSET_FILL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        colors << 0.0, 0.0, 0.0;
        shader_.setUniform("intensity", colors);
        shader_.drawIndexed(GL_TRIANGLES, 0, mesh_->get_number_of_face());
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if (normals_) {
        shaderNormals_.bind();
        shaderNormals_.setUniform("MV", mv);
        shaderNormals_.setUniform("P", p);
        shaderNormals_.drawIndexed(GL_TRIANGLES, 0, mesh_->get_number_of_face());
    }
}

bool Viewer::scrollEvent(const Vector2i &p, const Vector2f &rel) {
    if (!Screen::scrollEvent(p, rel)) {
        camera_.zoom = max(0.1, camera_.zoom * (rel.y() > 0 ? 1.1 : 0.9));
    }
    return true;
}

bool Viewer::mouseMotionEvent(const Vector2i &p, const Vector2i &rel,
        int button, int modifiers) {
    if (!Screen::mouseMotionEvent(p, rel, button, modifiers)) {
        if (camera_.arcball.motion(p)) {
            //
        } else if (translate_) {
            Eigen::Matrix4f model, view, proj;
            computeCameraMatrices(model, view, proj);
            Point mesh_center = mesh_->get_mesh_center();
            float zval = nanogui::project(Vector3f(mesh_center.x,
                        mesh_center.y,
                        mesh_center.z),
                    view * model, proj, mSize).z();
            Eigen::Vector3f pos1 = nanogui::unproject(
                    Eigen::Vector3f(p.x(), mSize.y() - p.y(), zval),
                    view * model, proj, mSize);
            Eigen::Vector3f pos0 = nanogui::unproject(
                    Eigen::Vector3f(translateStart_.x(), mSize.y() -
                        translateStart_.y(), zval), view * model, proj, mSize);
            camera_.modelTranslation = camera_.modelTranslation_start + (pos1-pos0);
        }
    }
    return true;
}

bool Viewer::mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) {
    if (!Screen::mouseButtonEvent(p, button, down, modifiers)) {
        if (button == GLFW_MOUSE_BUTTON_1 && modifiers == 0) {
            camera_.arcball.button(p, down);
        } else if (button == GLFW_MOUSE_BUTTON_2 ||
                (button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_SHIFT)) {
            camera_.modelTranslation_start = camera_.modelTranslation;
            translate_ = true;
            translateStart_ = p;
        }
    }
    if (button == GLFW_MOUSE_BUTTON_1 && !down) {
        camera_.arcball.button(p, false);
    }
    if (!down) {
        translate_ = false;
    }
    return true;
}

void Viewer::initShaders() {
    // Shaders
    shader_.init(
            "a_simple_shader",

            /* Vertex shader */
            "#version 330\n"
            "uniform mat4 MV;\n"
            "uniform mat4 P;\n"
            "uniform int color_mode;\n"
            "uniform vec3 intensity;\n"

            "in vec3 position;\n"
            "in vec3 valence_color;\n"
            "in vec3 unicruvature_color;\n"
            "in vec3 curvature_color;\n"
            "in vec3 gaussian_curv_color;\n"
            "in vec3 normal;\n"

            "out vec3 fcolor;\n"
            "out vec3 fnormal;\n"
            "out vec3 view_dir;\n"
            "out vec3 light_dir;\n"

            "void main() {\n"
            "    vec4 vpoint_mv = MV * vec4(position, 1.0);\n"
            "    gl_Position = P * vpoint_mv;\n"
            "    if (color_mode == 1) {\n"
            "        fcolor = valence_color;\n"
            "    } else if (color_mode == 2) {\n"
            "        fcolor = unicruvature_color;\n"
            "    } else if (color_mode == 3) {\n"
            "        fcolor = curvature_color;\n"
            "    } else if (color_mode == 4) {\n"
            "        fcolor = gaussian_curv_color;\n"
            "    } else {\n"
            "        fcolor = intensity;\n"
            "    }\n"
            "    fnormal = mat3(transpose(inverse(MV))) * normal;\n"
            "    light_dir = vec3(0.0, 3.0, 3.0) - vpoint_mv.xyz;\n"
            "    view_dir = -vpoint_mv.xyz;\n"
            "}",

        /* Fragment shader */
        "#version 330\n"
            "uniform int color_mode;\n"
            "uniform vec3 intensity;\n"

            "in vec3 fcolor;\n"
            "in vec3 fnormal;\n"
            "in vec3 view_dir;\n"
            "in vec3 light_dir;\n"

            "out vec4 color;\n"

            "void main() {\n"
            "    vec3 c = vec3(0.0);\n"
            "    if (color_mode == 0) {\n"
            "        c += vec3(1.0)*vec3(0.18, 0.1, 0.1);\n"
            "        vec3 n = normalize(fnormal);\n"
            "        vec3 v = normalize(view_dir);\n"
            "        vec3 l = normalize(light_dir);\n"
            "        float lambert = dot(n,l);\n"
            "        if(lambert > 0.0) {\n"
            "            c += vec3(1.0)*vec3(0.9, 0.5, 0.5)*lambert;\n"
            "            vec3 v = normalize(view_dir);\n"
            "            vec3 r = reflect(-l,n);\n"
            "            c += vec3(1.0)*vec3(0.8, 0.8, 0.8)*pow(max(dot(r,v), 0.0), 90.0);\n"
            "        }\n"
            "        c *= fcolor;\n"
            "    } else {\n"
            "       c = fcolor;\n"
            "    }\n"
            "    if (intensity == vec3(0.0)) {\n"
            "        c = intensity;\n"
            "    }\n"
            "    color = vec4(c, 1.0);\n"
            "}"
            );

    shaderNormals_.init(
            "normal_shader",
            /* Vertex shader */
            "#version 330\n\n"
            "in vec3 position;\n"
            "in vec3 normal;\n"
            "uniform mat4 MV;\n"
            "uniform mat4 P;\n"
            "uniform int normal_selector;\n"
            "out VS_OUT {\n"
            "    mat3 normal_mat;\n"
            "    vec3 normal;\n"
            "} vs_out;\n"
            "void main() {\n"
            "  gl_Position = vec4(position, 1.0);\n"
            "    vs_out.normal = normal;\n"
            "    vs_out.normal_mat = mat3(transpose(inverse(MV)));\n"
            "}",
            /* Fragment shader */
            "#version 330\n\n"
            "out vec4 frag_color;\n"
            "void main() {\n"
            "   frag_color = vec4(0.0, 1.0, 0.0, 1.0);\n"
            "}",
            /* Geometry shader */
            "#version 330\n\n"
                "layout (triangles) in;\n"
                "layout (line_strip, max_vertices = 6) out;\n"
                "uniform mat4 MV;\n"
                "uniform mat4 P;\n"
                "in VS_OUT {\n"
                "    mat3 normal_mat;\n"
                "    vec3 normal;\n"
                "} gs_in[];\n"
                "void createline(int index) {\n"
                "   gl_Position = P * MV * gl_in[index].gl_Position;\n"
                "   EmitVertex();\n"
                "   vec4 normal_mv = vec4(normalize(gs_in[index].normal_mat *\n"
                "                                   gs_in[index].normal), 1.0f);\n"
                "   gl_Position = P * (MV * gl_in[index].gl_Position\n"
                "                      + normal_mv * 0.035f);\n"
                "   EmitVertex();\n"
                "   EndPrimitive();\n"
                "}\n"
                "void main() {\n"
                "   createline(0);\n"
                "   createline(1);\n"
                "   createline(2);\n"
                "}"
                );
}

Viewer::Viewer() : nanogui::Screen(Eigen::Vector2i(1024, 768), "DGP Viewer") {

    window_ = new Window(this, "Controls");
    window_->setPosition(Vector2i(15, 15));
    window_->setLayout(new GroupLayout());

    new Label(window_, "Input/Output", "sans-bold");
    PopupButton *popupBtn = new PopupButton(window_, "Open a mesh", ENTYPO_ICON_EXPORT);
    Popup *popup = popupBtn->popup();
    popup->setLayout(new GroupLayout());

    Button *b = new Button(popup, "Open mesh ...");
    b->setCallback([this]() {
            string filename = nanogui::file_dialog({{"obj", "Wavefront OBJ"},
                    {"ply", "Stanford PLY"},
                    {"aln", "Aligned point cloud"},
                    {"off", "Object File Format"}
                    }, false);
            if (filename != "") {
            mesh_->load_mesh(filename);
            this->refresh_mesh();
            this->refresh_trackball_center();
            }
            });

    b = new Button(window_, "Export Mesh");
    b->setCallback([this]() {
            this->mesh_->export_mesh("export.obj");
            });

    new Label(window_, "Display Control", "sans-bold");

    /* ## Display option bloat. */

    popupBtn = new PopupButton(window_, "Infos Display");
    popup = popupBtn->popup();
    popup->setLayout(new GroupLayout());

    Widget* panel = new Widget(popup);
    //panel->setLayout(new GroupLayout());

    GridLayout *layout = new GridLayout(Orientation::Vertical, 4,
                                        Alignment::Middle, 15, 5);
    layout->setColAlignment({ Alignment::Maximum, Alignment::Fill });
    layout->setSpacing(0, 10);
    panel->setLayout(new GridLayout());
   
    b = new Button(popup, "Wireframe");
    b->setFlags(Button::ToggleButton);
    b->setChangeCallback([this](bool wireframe) {
            this->wireframe_ =! this->wireframe_;
            });
    b = new Button(popup, "Normals");
    b->setFlags(Button::ToggleButton);
    b->setChangeCallback([this](bool normals) {
            this->normals_ =! this->normals_;
            });

    b = new Button(popup, "Valence");
    b->setFlags(Button::ToggleButton);
    b->setChangeCallback([this](bool valence) {
            if (valence) {
            this->color_mode = VALENCE;
            } else {
            this->color_mode = NORMAL;
            }
            this->popupCurvature->setPushed(false);
            });

    popupCurvature = new PopupButton(popup, "Curvature");
    popup = popupCurvature->popup();
    popupCurvature->setCallback([this] () {
            this->color_mode = CURVATURE;
            });
    popup->setLayout(new GroupLayout());
    new Label(popup, "Curvature Type", "sans-bold");
    b = new Button(popup, "Uniform Laplacian");
    b->setFlags(Button::RadioButton);
    b->setPushed(true);
    b->setCallback([this]() {
            this->curvature_type = UNIMEAN;
            });
    b = new Button(popup, "Laplace-Beltrami");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
            this->curvature_type = LAPLACEBELTRAMI;
            });
    b = new Button(popup, "Gaussian");
    b->setFlags(Button::RadioButton);
    b->setCallback([this]() {
            this->curvature_type = GAUSS;
            });

    new Label(window_, "Transformations", "sans-bold");
    b = new Button(window_, "3D Wireframe");
    b->setCallback([this]() {
            if (!reset_performed_){
                this->reset_mesh();
            }
            reset_performed_ = false;
            this->mesh_->create_wire_frame(wireframeSphereDiameterTextBox->value(), wireframeCylinderDiameterTextBox->value());
            this->mesh_->compute_mesh_properties();
            this->refresh_mesh();
    });

    popupBtn = new PopupButton(window_, "Wireframe Params.");
    popup = popupBtn->popup();
    popup->setLayout(new GroupLayout());

    panel = new Widget(popup);
    panel->setLayout(new GroupLayout());

    //panel = new Widget(popup);
    layout = new GridLayout(Orientation::Horizontal, 2,
                                        Alignment::Middle, 15, 5);
    layout->setColAlignment({ Alignment::Maximum, Alignment::Fill });
    layout->setSpacing(0, 10);
    panel->setLayout(layout);
    new Label(panel, "Spheres diameter:", "sans-bold");
    wireframeSphereDiameterTextBox = new FloatBox<float>(panel, 0.05f);
    wireframeSphereDiameterTextBox->setEditable(true);
    wireframeSphereDiameterTextBox->setFixedSize(Vector2i(50, 20));
    wireframeSphereDiameterTextBox->setDefaultValue("10");
    wireframeSphereDiameterTextBox->setFontSize(16);
    wireframeSphereDiameterTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");

    new Label(panel, "Cylinders diameter:", "sans-bold");
    wireframeCylinderDiameterTextBox = new FloatBox<float>(panel, 0.05f);
    wireframeCylinderDiameterTextBox->setEditable(true);
    wireframeCylinderDiameterTextBox->setFixedSize(Vector2i(50, 20));
    wireframeCylinderDiameterTextBox->setDefaultValue("2.0");
    wireframeCylinderDiameterTextBox->setFontSize(16);
    wireframeCylinderDiameterTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");

    b = new Button(window_, "Tree");
    b->setCallback([this]() {
        if (!reset_performed_){
            this->reset_mesh();
        }
        reset_performed_ = false;
        mesh_->create_tree_wireframe(sphereDiameterTextBox->value(),
                                     cylinderDiameterTextBox->value(),
                                     branchMaxLengthTextBox->value(),
                                     trunkScaleMultTextBox->value(),
                                     minDotBetweenBranchesTextBox->value(),
                                     minRelLengthBeforeSplitTextBox->value());
        this->mesh_->compute_mesh_properties();
        this->refresh_mesh();
    });

    /* ###### layout for tree  parameters. */
    popupBtn = new PopupButton(window_, "Tree Params.");
    popup = popupBtn->popup();
    popup->setLayout(new GroupLayout());

    panel = new Widget(popup);
    panel->setLayout(new GroupLayout());

    layout = new GridLayout(Orientation::Horizontal, 2,
                                        Alignment::Middle, 15, 5);
    layout->setColAlignment({ Alignment::Maximum, Alignment::Fill });
    layout->setSpacing(0, 10);
    panel->setLayout(layout);
    new Label(panel, "Sphere base diameter:", "sans-bold");
    sphereDiameterTextBox = new FloatBox<float>(panel, 0.5f);
    sphereDiameterTextBox->setEditable(true);
    sphereDiameterTextBox->setFixedSize(Vector2i(50, 20));
    sphereDiameterTextBox->setDefaultValue("10");
    sphereDiameterTextBox->setFontSize(16);
    sphereDiameterTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");

    new Label(panel, "Cylinder base diameter:", "sans-bold");
    cylinderDiameterTextBox = new FloatBox<float>(panel, 0.5f);
    cylinderDiameterTextBox->setEditable(true);
    cylinderDiameterTextBox->setFixedSize(Vector2i(50, 20));
    cylinderDiameterTextBox->setDefaultValue("2.0");
    cylinderDiameterTextBox->setFontSize(16);
    cylinderDiameterTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");

    new Label(panel, "Branch max. length:", "sans-bold");
    branchMaxLengthTextBox = new FloatBox<float>(panel, 15.0);
    branchMaxLengthTextBox->setEditable(true);
    branchMaxLengthTextBox->setFixedSize(Vector2i(50, 20));
    branchMaxLengthTextBox->setDefaultValue("2.0");
    branchMaxLengthTextBox->setFontSize(16);
    branchMaxLengthTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");

    new Label(panel, "Trunk scale multiplier:", "sans-bold");
    trunkScaleMultTextBox = new FloatBox<float>(panel, 1.2);
    trunkScaleMultTextBox->setEditable(true);
    trunkScaleMultTextBox->setFixedSize(Vector2i(50, 20));
    trunkScaleMultTextBox->setDefaultValue("2.0");
    trunkScaleMultTextBox->setFontSize(16);
    trunkScaleMultTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");

    new Label(panel, "Min. dot between branches:", "sans-bold");
    minDotBetweenBranchesTextBox = new FloatBox<float>(panel, 0.0);
    minDotBetweenBranchesTextBox->setEditable(true);
    minDotBetweenBranchesTextBox->setFixedSize(Vector2i(50, 20));
    minDotBetweenBranchesTextBox->setDefaultValue("2.0");
    minDotBetweenBranchesTextBox->setFontSize(16);
    minDotBetweenBranchesTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");

    new Label(panel, "Min. relative length before split:", "sans-bold");
    minRelLengthBeforeSplitTextBox = new FloatBox<float>(panel, 1.0);
    minRelLengthBeforeSplitTextBox->setEditable(true);
    minRelLengthBeforeSplitTextBox->setFixedSize(Vector2i(50, 20));
    minRelLengthBeforeSplitTextBox->setDefaultValue("2.0");
    minRelLengthBeforeSplitTextBox->setFontSize(16);
    minRelLengthBeforeSplitTextBox->setFormat("[-]?[0-9]*\\.?[0-9]+");

    b = new Button(window_, "Reset");
    b->setCallback([this]() {
        this->reset_mesh();
        reset_performed_ = true;
    });

    performLayout();

    initShaders();
    const float diameters = 0.5f;
    mesh_ = new mesh_processing::TreeProcessing("../models/geralt_tex_roots.off", "../models/sphere.obj", "../models/cylinder_medium_poly.obj", diameters, diameters);
    this->refresh_mesh();
    this->refresh_trackball_center();
}

void Viewer::reset_mesh(void) {
    this->mesh_->revert_changes();
    this->mesh_->compute_mesh_properties();
    this->refresh_mesh();
    reset_performed_ = true;
}

void Viewer::refresh_trackball_center() {
    // Re-center the mesh
    Point mesh_center = mesh_->get_mesh_center();
    camera_.arcball = Arcball();
    camera_.arcball.setSize(mSize);
    camera_.modelZoom = 2/mesh_->get_dist_max();
    camera_.modelTranslation = -Vector3f(mesh_center.x, mesh_center.y, mesh_center.z);
}

void Viewer::refresh_mesh() {
    shader_.bind();
    shader_.uploadIndices(*(mesh_->get_indices()));
    shader_.uploadAttrib("position", *(mesh_->get_points()));
    shader_.uploadAttrib("valence_color", *(mesh_->get_colors_valence()));
    shader_.uploadAttrib("unicruvature_color", *(mesh_->get_colors_unicurvature()));
    shader_.uploadAttrib("curvature_color", *(mesh_->get_color_curvature()));
    shader_.uploadAttrib("gaussian_curv_color", *(mesh_->get_colors_gaussian_curv()));
    shader_.uploadAttrib("normal", *(mesh_->get_normals()));
    shader_.setUniform("color_mode", int(color_mode));
    shader_.setUniform("intensity", Vector3f(0.98, 0.59, 0.04));

    shaderNormals_.bind();
    shaderNormals_.shareAttrib(shader_, "indices");
    shaderNormals_.shareAttrib(shader_, "position");
    shaderNormals_.shareAttrib(shader_, "normal");

}

void Viewer::computeCameraMatrices(Eigen::Matrix4f &model,
        Eigen::Matrix4f &view,
        Eigen::Matrix4f &proj) {

    view = nanogui::lookAt(camera_.eye, camera_.center, camera_.up);

    float fH = std::tan(camera_.viewAngle / 360.0f * M_PI) * camera_.dnear;
    float fW = fH * (float) mSize.x() / (float) mSize.y();

    proj = nanogui::frustum(-fW, fW, -fH, fH, camera_.dnear, camera_.dfar);
    model = camera_.arcball.matrix();

    model = nanogui::scale(model, Eigen::Vector3f::Constant(camera_.zoom * camera_.modelZoom));
    model = nanogui::translate(model, camera_.modelTranslation);
}

Viewer::~Viewer() {
    shader_.free();
    shaderNormals_.free();
    delete mesh_;
}
