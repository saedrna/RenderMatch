/*
 * @Author: Han
 * @Date: 2019-11-15 20:01:59
 * The pipeline for render image and aerial-ground match with rendered images as delegates
 */

#include <cxxopts.hpp>

#include <opencv2/imgcodecs.hpp>
#include <osg/PagedLOD>
#include <osgDB/DatabasePager>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgUtil/SmoothingVisitor>
#include <osgViewer/Viewer>

#include <RenderMatch/block.h>
#include <base/base.h>

#include <gdal.h>

#include "render_matcher.h"

using namespace h2o;

struct ScreenShot : public osg::Camera::DrawCallback {
    ScreenShot(const h2o::Block &block) : block_ground_(block) { iid_ = INVALID_INDEX; }

    void set_iid(uint32_t iid) { iid_ = iid; }

    void set_buffer(const std::shared_ptr<cv::Mat> &rgb, const std::shared_ptr<cv::Mat> &dep) {
        mat_rgb_ = rgb;
        mat_dep_ = dep;
    }

    virtual void operator()(osg::RenderInfo &renderInfo) const {
        if (iid_ == INVALID_INDEX) {
            return;
        }

        osg::Camera *camera = renderInfo.getCurrentCamera();
        osg::Viewport *viewport = camera ? camera->getViewport() : 0;
        if (viewport) {
            osg::ref_ptr<osg::Image> image = new osg::Image;

            std::string name = block_ground_.photos.at(iid_).path;

            cv::Mat mat_rgb;
            cv::Mat mat_dep;

            // read the rgb value
            {
                glReadBuffer(camera->getDrawBuffer());
                image->readPixels(viewport->x(), viewport->y(), viewport->width(), viewport->height(), GL_RGBA,
                                  GL_UNSIGNED_BYTE, 1);
                image->flipVertical();
                mat_rgb = cv::Mat(viewport->height(), viewport->width(), CV_8UC4, (void *)image->getDataPointer(),
                                  (size_t)image->getRowStepInBytes())
                              .clone();
                cv::cvtColor(mat_rgb, mat_rgb, cv::COLOR_RGBA2BGRA);
                *mat_rgb_ = mat_rgb.clone();
            }

            // read the depth value
            {
                glReadBuffer(camera->getDrawBuffer());
                image->readPixels(viewport->x(), viewport->y(), viewport->width(), viewport->height(),
                                  GL_DEPTH_COMPONENT, GL_FLOAT);
                image->flipVertical();
                mat_dep = cv::Mat(viewport->height(), viewport->width(), CV_32FC1, (void *)image->getDataPointer(),
                                  (size_t)image->getRowStepInBytes());

                *mat_dep_ = mat_dep.clone();
            }
        }
    }

    h2o::Block block_ground_;
    uint32_t iid_;

    // render function only support const, we must pass pointer here
    std::shared_ptr<cv::Mat> mat_rgb_;
    std::shared_ptr<cv::Mat> mat_dep_;
};

int main(int argc, char **argv) {
    GDALAllRegister();

    cxxopts::Options options("RenderMeshMatch",
                             "The pipeline for render image and aerial-ground match with rendered images as delegates");

    std::string path_ground_at;
    std::string path_aerial_at;
    std::string path_model;
    std::string path_config;
    std::string origin_coord_at;

    // clang-format off
    options.add_options("RenderMeshMatch")
        ("a,aerial", "Aerial AT file", cxxopts::value(path_aerial_at))
        ("g,ground", "Ground AT file", cxxopts::value(path_ground_at))
        ("m,model", "Path to the mesh", cxxopts::value(path_model))
        ("c,config", "Path to the match parameters", cxxopts::value(path_config))
		("t,transform","original coordinate", cxxopts::value(origin_coord_at))
        ("h,help", "Print this help message");
    // clang-format on

    auto results = options.parse(argc, argv);
    if (results.arguments().empty() || results.count("help")) {
        std::cout << options.help({"UndistortImage"}) << std::endl;
        return 1;
    }

    path_ground_at = QFileInfo(QString::fromLocal8Bit(path_ground_at.c_str())).absoluteFilePath().toStdString();
    path_aerial_at = QFileInfo(QString::fromLocal8Bit(path_aerial_at.c_str())).absoluteFilePath().toStdString();
    path_model = QFileInfo(QString::fromLocal8Bit(path_model.c_str())).absoluteFilePath().toStdString();
    RenderMeshMatchConfig param = load_config(path_config);
    Eigen::Vector3d origin_coord;
    const char *s = origin_coord_at.data();
    double a, b, c;
    std::sscanf(s, "%lf,%lf,%lf", &a, &b, &c);
    origin_coord << a, b, c;

    h2o::Block block_aerial = load_block(path_aerial_at,origin_coord);
    h2o::Block block_ground = load_block(path_ground_at,origin_coord);
    h2o::Block block_ground_rectified = undistort_block(block_ground);

    osgViewer::Viewer viewer;

    osg::ref_ptr<ScreenShot> screen_shot = new ScreenShot(block_ground_rectified);

    osg::ref_ptr<osg::Node> model = osgDB::readRefNodeFile(path_model);
    viewer.setSceneData(model);

    RenderMatcher matcher;
    matcher.set_block(block_aerial, block_ground);
    matcher.set_param(param);
    RenderMatchResults match_results;

    // for each photo groups, set the viewport for rendering and do the rendering
    for (const auto &pgroups : block_ground_rectified.groups) {
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
        for (uint32_t iid : photos) {
            Photo photo = block_ground_rectified.photos.at(iid);

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
                while (viewer.getDatabasePager()->getFileRequestListSize()) {
                    // waiting for loading
                }
            }

            std::shared_ptr<cv::Mat> mat_rgb = std::make_shared<cv::Mat>();
            std::shared_ptr<cv::Mat> mat_dep = std::make_shared<cv::Mat>();
            screen_shot->set_buffer(mat_rgb, mat_dep);
            screen_shot->set_iid(iid);
            viewer.frame();

            // we have finished the rendering, now we have the rgb and depth image
            osg::Matrixd proj = viewer.getCamera()->getProjectionMatrix();
            osg::Matrixd view = viewer.getCamera()->getViewMatrix();

            Matrix4f eproj, eview;
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    eproj(r, c) = proj(c, r);
                    eview(r, c) = view(c, r);
                }
            }

            matcher.set_ogl_matrices(eview, eproj);
            RenderMatchResults results_image = matcher.match(iid, *mat_rgb, *mat_dep);
            match_results.insert(end(match_results), begin(results_image), end(results_image));
        }
    }
    matcher.save_match(match_results);

    return 0;
}
