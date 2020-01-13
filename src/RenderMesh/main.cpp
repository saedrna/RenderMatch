/*
 * @Author: Han
 * @Date: 2019-10-22 15:43:42
 * Render the mesh to ground images with color, depth and normal
 */

#include <cxxopts.hpp>

#include <RenderMatch/block.h>
#include <base/base.h>

#include <opencv2/imgcodecs.hpp>

#include <osg/PagedLOD>
#include <osgDB/DatabasePager>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgUtil/SmoothingVisitor>
#include <osgViewer/Viewer>

using namespace h2o;

cv::Mat pcd_normal(const cv::Mat &mat_xyz, const Vector2i &size);

struct ScreenShot : public osg::Camera::DrawCallback {
    ScreenShot(const h2o::Block &block) : block_(block) {
        iid_ = INVALID_INDEX;
        outdir_ = "";
        save_rgb_ = save_dep_ = save_xyz_ = save_nor_ = true;
    }

    void set_iid(uint32_t iid) { iid_ = iid; }
    void set_outdir(const std::string &dir) { outdir_ = dir; }
    void set_export(bool rgb, bool dep = false, bool xyz = false, bool nor = false) {
        save_rgb_ = rgb;
        save_dep_ = dep;
        save_xyz_ = xyz;
        save_nor_ = nor;
    }

    virtual void operator()(osg::RenderInfo &renderInfo) const {
        if (iid_ == INVALID_INDEX) {
            return;
        }
        if (outdir_.empty()) {
            return;
        }

        osg::Camera *camera = renderInfo.getCurrentCamera();
        osg::Viewport *viewport = camera ? camera->getViewport() : 0;
        if (viewport) {
            osg::ref_ptr<osg::Image> image = new osg::Image;

            std::string name = block_.photos.at(iid_).path;
            name = get_filename_noext(name);

            cv::Mat mat_rgb;
            cv::Mat mat_dep;
            cv::Mat mat_xyz;
            cv::Mat mat_nor;

            // read the rgb value
            if (save_rgb_) {
                glReadBuffer(camera->getDrawBuffer());
                image->readPixels(viewport->x(), viewport->y(), viewport->width(), viewport->height(), GL_RGBA,
                                  GL_UNSIGNED_BYTE, 1);
                image->flipVertical();
                std::string path_rgba = join_paths(outdir_, name + "_rgb.png");
                mat_rgb = cv::Mat(viewport->height(), viewport->width(), CV_8UC4, (void *)image->getDataPointer(),
                                  (size_t)image->getRowStepInBytes())
                              .clone();
                cv::cvtColor(mat_rgb, mat_rgb, cv::COLOR_RGBA2BGRA);
                cv::imwrite(path_rgba, mat_rgb);
            }

            // read the depth value
            if (save_dep_) {
                glReadBuffer(camera->getDrawBuffer());
                image->readPixels(viewport->x(), viewport->y(), viewport->width(), viewport->height(),
                                  GL_DEPTH_COMPONENT, GL_FLOAT);
                image->flipVertical();
                mat_dep = cv::Mat(viewport->height(), viewport->width(), CV_32FC1, (void *)image->getDataPointer(),
                                  (size_t)image->getRowStepInBytes());
                std::string path_dep = join_paths(outdir_, name + "_dep.tif");
                gdal_write_image(path_dep, mat_dep);
                gdal_set_nodata(path_dep, 1.0);
                gdal_compute_statistics(path_dep);
            }

            // calculate the xyz for the depths value
            // this is only for demonstration, in applications, we never directly calculate the xyz and normal images
            if (save_dep_ && save_xyz_) {
                mat_xyz = cv::Mat(mat_rgb.rows, mat_rgb.cols, CV_32FC3);
                osg::Matrixd proj = camera->getProjectionMatrix();
                osg::Matrixd view = camera->getViewMatrix();

                Matrix4f eproj, eview;
                for (int r = 0; r < 4; ++r) {
                    for (int c = 0; c < 4; ++c) {
                        eproj(r, c) = proj(c, r);
                        eview(r, c) = view(c, r);
                    }
                }

                Matrix4f inverse_mvp = (eproj * eview).inverse();
                for (int r = 0; r < mat_rgb.rows; ++r) {
                    float *ptr_dep = mat_dep.ptr<float>(r);
                    cv::Vec3f *ptr_xyz = mat_xyz.ptr<cv::Vec3f>(r);
                    for (int c = 0; c < mat_rgb.cols; ++c) {
                        float depth = ptr_dep[c];
                        if (std::abs(depth - 1.0) < 1e-6f) {
                            ptr_xyz[c][0] = FLT_MAX;
                            ptr_xyz[c][1] = FLT_MAX;
                            ptr_xyz[c][2] = FLT_MAX;
                            continue;
                        }

                        // image use the pixel center, but ogl maps [0,1] to the image boarder
                        Vector4f screen;
                        screen(0) = (float)(c + 0.5f) / mat_rgb.cols;
                        screen(1) = (float)(mat_rgb.rows - r - 0.5) / mat_rgb.rows;
                        screen(2) = depth;
                        screen(3) = 1.0;

                        screen = screen.array() * 2.0f - 1.0f;

                        Vector4f object = inverse_mvp * screen;
                        Vector3f coord = object.hnormalized();
                        coord.z() = coord.z() - 500;
                        coord.normalize();
                       //std::cout << coord << std::endl;

                        ptr_xyz[c][0] = coord.x();
                        ptr_xyz[c][1] = coord.y();
                        ptr_xyz[c][2] = coord.z();
                    }
                }

                std::string path_xyz = join_paths(outdir_, name + "_xyz.tif");
                gdal_write_image(path_xyz, mat_xyz);
                gdal_set_nodata(path_xyz, FLT_MAX);
                gdal_compute_statistics(path_xyz);
            }

            // calculate the normal vector, because it's not possible without writing shader for it
            // we calculate it in CPU from the points here
            if (save_dep_ && save_xyz_ && save_nor_) {
                mat_nor = pcd_normal(mat_xyz, Vector2i(mat_xyz.cols / 2, mat_xyz.rows / 2));
                // change the direction of normal vector
                // normal vector should face the camera
                Vector3f C = block_.photos.at(iid_).C.cast<float>();
                for (int r = 0; r < mat_nor.rows; ++r) {
                    cv::Vec3f *ptr_nor = mat_nor.ptr<cv::Vec3f>(r);
                    cv::Vec3f *ptr_xyz = mat_xyz.ptr<cv::Vec3f>(r);
                    for (int c = 0; c < mat_nor.cols; ++c) {
                        cv::Vec3f nor = ptr_nor[c];
                        cv::Vec3f xyz = ptr_xyz[c];
                        if (nor[0] == 0.0f && nor[1] == 0.0f && nor[2] == 0.0f) {
                            continue;
                        }
                        if (xyz[0] == FLT_MAX || xyz[1] == FLT_MAX || xyz[2] == FLT_MAX) {
                            continue;
                        }
                        Vector3f xyz3(xyz[0], xyz[1], xyz[2]);
                        Vector3f nor3(nor[0], nor[1], nor[2]);

                        Vector3f dir = C - xyz3;
                        if (nor3.dot(dir) < 0) {
                            nor = -nor;
                        }
                        ptr_nor[c] = nor;
                    }
                }

                std::string path_nor = join_paths(outdir_, name + "_nor.tif");
                gdal_write_image(path_nor, mat_nor);
                gdal_compute_statistics(path_nor);
            }
        }
    }

    h2o::Block block_;
    uint32_t iid_;
    std::string outdir_;

    bool save_rgb_;
    bool save_dep_;
    bool save_xyz_;
    bool save_nor_;
};

int main(int argc, char **argv) {
    cxxopts::Options options("RenderMesh", "Render the mesh to ground images with color, depth and normal information");

    std::string path_at;
    std::string path_model;
    std::string path_out;

    bool save_rgb = true;
    bool save_dep = true;
    bool save_xyz = true;
    bool save_nor = true;

    // clang-format off
    options.add_options("RenderMesh")
        ("a,atfile", "Input aerial triangulation file in block exchange format", cxxopts::value(path_at))
        ("m,mesh", "Input mesh file in osgb format", cxxopts::value(path_model))
        ("o,output", "Output directory containing all the rendered images", cxxopts::value(path_out))
        ("no-dep", "Do not save depth data")
        ("no-xyz", "Do not compute the xyz data")
        ("no-nor", "Do not compute the normal vectors")
        ("h,help", "Print this help message");
    // clang-format on

    auto results = options.parse(argc, argv);
    if (results.arguments().empty() || results.count("help")) {
        std::cout << options.help({"RenderMesh"}) << std::endl;
        return 1;
    }

    if (results.count("no-dep")) {
        save_dep = false;
    }
    if (results.count("no-xyz")) {
        save_xyz = false;
    }
    if (results.count("no-nor")) {
        save_nor = false;
    }

    path_at = QFileInfo(QString::fromLocal8Bit(path_at.c_str())).absoluteFilePath().toStdString();
    path_model = QFileInfo(QString::fromLocal8Bit(path_model.c_str())).absoluteFilePath().toStdString();
    path_out = QFileInfo(QString::fromLocal8Bit(path_out.c_str())).absoluteFilePath().toStdString();

    using namespace h2o;
    h2o::Block block = load_block_xml(path_at);
    h2o::Block rectified = undistort_block(block);

    osgViewer::Viewer viewer;

    osg::ref_ptr<ScreenShot> screen_shot = new ScreenShot(rectified);
    screen_shot->set_outdir(path_out);
    screen_shot->set_export(save_rgb, save_dep, save_xyz, save_nor);

    osg::ref_ptr<osg::Node> model = osgDB::readRefNodeFile(path_model);
    viewer.setSceneData(model);

    // for each photo groups, set the viewport for rendering and do the rendering
    for (const auto &pgroups : rectified.groups) {
        uint32_t cid = pgroups.first;
        PhotoGroup pgroup = pgroups.second;

        uint32_t width = pgroup.width;
        uint32_t height = pgroup.height;
        double focal = pgroup.f;

        // setup per camera settings
        {
            osg::ref_ptr<osg::DisplaySettings> ds = new osg::DisplaySettings;

            ds->setScreenWidth((float)width);
            ds->setScreenHeight((float)height);
            ds->setScreenDistance((float)focal);
            ds->setStereo(false);

            osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits(ds.get());
            traits->readDISPLAY();
            if (traits->displayNum < 0)
                traits->displayNum = 0;

            traits->x = 0;
            traits->y = 0;
            traits->width = width;
            traits->height = height;
            traits->alpha = 8;
            traits->windowDecoration = false;
            traits->doubleBuffer = true;
            traits->sharedContext = 0;
            traits->pbuffer = true;
            osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
            if (!gc) {
                LOG(ERROR) << "Failed to created requested graphics context";
                return 1;
            }

            // transparent background
            viewer.getCamera()->setClearColor(osg::Vec4(255.0f, 255.0f, 255.0f, 0.0f));
            viewer.getCamera()->setGraphicsContext(gc.get());
            viewer.getCamera()->setDisplaySettings(ds.get());

            osgViewer::GraphicsWindow *gw = dynamic_cast<osgViewer::GraphicsWindow *>(gc.get());
            if (gw) {
                LOG(INFO) << "GraphicsWindow has been created successfully." << std::endl;
                gw->getEventQueue()->getCurrentEventState()->setWindowRectangle(0, 0, width, height);
            } else {
                LOG(INFO) << "PixelBuffer has been created succseffully " << traits->width << ", " << traits->height
                          << std::endl;
            }

            viewer.getCamera()->setCullMask(0xffffffff);
            viewer.getCamera()->setCullMaskLeft(0x00000001);
            viewer.getCamera()->setCullMaskRight(0x00000002);

            viewer.getCamera()->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));

            GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;

            viewer.getCamera()->setDrawBuffer(buffer);
            viewer.getCamera()->setReadBuffer(buffer);

            viewer.getCamera()->setFinalDrawCallback(screen_shot);

            viewer.getCamera()->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
            viewer.getCamera()->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            viewer.getCamera()->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

            viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
            viewer.realize();
        }

        // set up the viewport
        auto photos = pgroup.photos;
        for (uint32_t iid = 21; iid < 23;iid++/* : photos*/) {
            Photo photo = rectified.photos.at(iid);

            // near and far are used to determine the projection matrix in ogl
            double znear = photo.znear;
            double zfar = photo.zfar;
            double zmed = photo.zmed;

            Matrix3d R = photo.R;
            Vector3d dir = R.transpose() * Vector3d(0, 0, 1);
            dir.normalize();

            // used to compute the view matrix in ogl
            /* The up direction is (0, -1, 0) in object space
             * ------->
             * |
             * |
             * \/
             */
            Vector3d eye = photo.C;
            Vector3d target = eye + dir * zmed;
            Vector3d up = R.transpose() * Vector3d(0, -1, 0);

            // set up matices and do the rendering
            {
                // per photo settings
                double vfov = atan2((double)height / 2.0, focal) * RAD2DEG * 2.0;
                viewer.getCamera()->setProjectionMatrixAsPerspective(vfov, (double)width / (double)height, znear, zfar);

                viewer.getCamera()->setViewMatrixAsLookAt(osg::Vec3(eye.x(), eye.y(), eye.z()),
                                                          osg::Vec3(target.x(), target.y(), target.z()),
                                                          osg::Vec3(up.x(), up.y(), up.z()));
            }

            // initialize the paged lod, waiting for loading
            screen_shot->set_iid(INVALID_INDEX);

            // there are at most 21 levels, and each frame will request all the nodes in the next level
            for (int i = 0; i < 21; ++i) {
                viewer.frame();
                int node_num = viewer.getDatabasePager()->getFileRequestListSize();
                LOG(INFO) << "Waiting " << node_num << " to load";
                while (viewer.getDatabasePager()->getFileRequestListSize()) {
                    // waiting for loading
                }
            }

            screen_shot->set_iid(iid);
            viewer.frame();
        }
    }

    return 0;
}
