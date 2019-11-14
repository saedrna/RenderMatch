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
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>

using namespace h2o;
struct ScreenShot : public osg::Camera::DrawCallback {
    ScreenShot(const h2o::Block &block) : block_(block) {
        iid_ = INVALID_INDEX;
        outdir_ = "";
    }

    void set_iid(uint32_t iid) { iid_ = iid; }
    void set_outdir(const std::string &dir) { outdir_ = dir; }

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

            glReadBuffer(camera->getDrawBuffer());
            image->readPixels(viewport->x(), viewport->y(), viewport->width(), viewport->height(), GL_RGBA,
                              GL_UNSIGNED_BYTE, 1);

            {
                image->flipVertical();
                std::string path_rgba = join_paths(outdir_, name + "_rgba.png");
                cv::Mat mat_rgba(viewport->height(), viewport->width(), CV_8UC4, (void *)image->getDataPointer(),
                                 (size_t)image->getRowStepInBytes());
                cv::cvtColor(mat_rgba, mat_rgba, cv::COLOR_RGBA2BGRA);
                cv::imwrite(path_rgba, mat_rgba);
            }

            glReadBuffer(camera->getDrawBuffer());
            image->readPixels(viewport->x(), viewport->y(), viewport->width(), viewport->height(), GL_DEPTH_COMPONENT,
                              GL_FLOAT);

            {
                image->flipVertical();
                std::string path_depth = join_paths(outdir_, name + "_depth.tif");
                cv::Mat mat_depth(viewport->height(), viewport->width(), CV_32FC1, (void *)image->getDataPointer(),
                                  (size_t)image->getRowStepInBytes());
                cv::imwrite(path_depth, mat_depth);
            }
        }
    }

    h2o::Block block_;
    uint32_t iid_;
    std::string outdir_;
};

int main(int argc, char **argv) {
    cxxopts::Options options("RenderMesh", "Render the mesh to ground images with color, depth and normal information");

    std::string path_at;
    std::string path_model;
    std::string path_out;

    // clang-format off
    options.add_options("RenderMesh")
        ("a,atfile", "Input aerial triangulation file in block exchange format", cxxopts::value(path_at))
        ("m,mesh", "Input mesh file in osgb format", cxxopts::value(path_model))
        ("o,output", "Output directory containing all the rendered images", cxxopts::value(path_out))
        ("h,help", "Print this help message");
    // clang-format on

    auto results = options.parse(argc, argv);
    if (results.arguments().empty() || results.count("help")) {
        std::cout << options.help({"RenderMesh"}) << std::endl;
        return 1;
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
        for (uint32_t iid : photos) {
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