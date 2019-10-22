/*
 * @Author: Han Hu
 * @Date: 2018-06-21 20:25:35
 * 从OSG中提取三维模型
 */
#include <MathGeoLib/MathGeoLib.h>

#include <base/base.h>
#include <modelpro/mesh.h>

#include <geogram/basic/command_line_args.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <osg/NodeVisitor>
#include <osg/PagedLOD>
#include <osgDB/ReadFile>

namespace h2o {
///\brief 不停的往下遍历node，知道碰到geometry的节点，裁剪对应的三角形
class ClipVisitor : public osg::NodeVisitor {
public:
    ClipVisitor(const BoundingBox3d &_bb, GEO::Mesh *mesh);
    ClipVisitor(const OBB3d &obb, GEO::Mesh *mesh);
    ~ClipVisitor();

    virtual void apply(osg::Group &group) override;
    virtual void apply(osg::PagedLOD &plod) override;
    virtual void apply(osg::Geometry &geom) override;

    template <typename T> void add_triangles(const std::vector<T> &vertices, const std::vector<uint32_t> &faces);

    OBB to_mathgeolib(const OBB3d &obb);

public:
    ///\brief osg 格式的 mesh 的 bb，用于判断某个物体是否和输入的 mesh 相交，如果相交才遍历他
    osg::BoundingBox osg_bb_;

    ///\brief 同上，mesh 的 bb
    BoundingBox3d bb_;

    // 顾及旋转的包围盒
    OBB3d obb_;

    ///\brief 输入和输出的mesh
    GEO::Mesh *mesh_;

    ///\brief
    std::vector<std::string> node_names_;
};

ClipVisitor::ClipVisitor(const BoundingBox3d &_bb, GEO::Mesh *_mesh)
    : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {

    bb_ = _bb;
    osg_bb_.xMin() = bb_.min()(0);
    osg_bb_.yMin() = bb_.min()(1);
    osg_bb_.zMin() = bb_.min()(2);
    osg_bb_.xMax() = bb_.max()(0);
    osg_bb_.yMax() = bb_.max()(1);
    osg_bb_.zMax() = bb_.max()(2);

    mesh_ = _mesh;
}

ClipVisitor::ClipVisitor(const OBB3d &_obb, GEO::Mesh *_mesh)
    : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {

    OBB obb = to_mathgeolib(_obb);
    AABB aabb = obb.MinimalEnclosingAABB();

    obb_ = _obb;
    bb_.min() = Vector3d(aabb.minPoint.x, aabb.minPoint.y, aabb.minPoint.z);
    bb_.max() = Vector3d(aabb.maxPoint.x, aabb.maxPoint.y, aabb.maxPoint.z);

    osg_bb_.xMin() = bb_.min()(0);
    osg_bb_.yMin() = bb_.min()(1);
    osg_bb_.zMin() = bb_.min()(2);
    osg_bb_.xMax() = bb_.max()(0);
    osg_bb_.yMax() = bb_.max()(1);
    osg_bb_.zMax() = bb_.max()(2);

    mesh_ = _mesh;
}

ClipVisitor::~ClipVisitor() {}

void ClipVisitor::apply(osg::Group &group) {
    // 判断节点和bb是否相交，如果相交才继续处理
    osg::BoundingSphere bs = group.getBound();
    osg::BoundingBox bb;
    bb.expandBy(bs);

    if (osg_bb_.intersects(bb))
        traverse(group);
    return;
}

void ClipVisitor::apply(osg::PagedLOD &plod) {
    osg::BoundingSphere bs = plod.getBound();
    osg::BoundingBox bb;
    bb.expandBy(bs);

    if (osg_bb_.intersects(bb)) {
        int num_files = plod.getNumFileNames();
        // 只有对根节点才会traverse，否则读入子节点
        if (num_files == 0) {
            ///\brief 只有到最底层了，才会希望真正的读入三角网文件
            traverse(plod);
        } else {
            // 读入子节点的文件，并且对子节点应用这个visitor
            for (int i = 0; i < num_files; ++i) {
                std::string basepath = plod.getDatabasePath();
                std::string path = plod.getFileName(i);
                if (!path.empty() && !basepath.empty()) {
                    std::string childpath = join_paths(basepath, path);
                    if (file_exist(childpath)) {
                        auto node = osgDB::readRefNodeFile(childpath);
                        if (node) {
                            if (node->asGeode() != NULL) {
                                node_names_.push_back(childpath);
                            }
                            node->accept(*this);
                        }
                    }
                }
            }
        }
    }
    return;
}

void ClipVisitor::apply(osg::Geometry &geom) {
    // 判断节点和bb是否相交，如果相交才继续处理
    osg::BoundingSphere bs = geom.getBound();
    osg::BoundingBox bb;
    bb.expandBy(bs);

    if (osg_bb_.intersects(bb)) {
        // 获取geometry的顶点和三角形
        osg::Array *vertices = geom.getVertexArray();
        std::vector<osg::Vec3> float_vertices;
        std::vector<osg::Vec3d> double_vertices;
        std::vector<uint32_t> indices;

        int num_primitive = geom.getNumPrimitiveSets();
        if (vertices->getType() == osg::Array::Vec3ArrayType || vertices->getType() == osg::Array::Vec3dArrayType) {
            if (vertices->getType() == osg::Array::Vec3ArrayType)
                float_vertices = ((osg::Vec3Array *)vertices)->asVector();
            else if (vertices->getType() == osg::Array::Vec3dArrayType)
                double_vertices = ((osg::Vec3dArray *)vertices)->asVector();

            indices.reserve(float_vertices.size() * 6);

            for (int i = 0; i < num_primitive; ++i) {
                osg::DrawElements *elements = dynamic_cast<osg::DrawElements *>(geom.getPrimitiveSet(i));
                if (elements->getDataType() == GL_UNSIGNED_INT &&
                    elements->getType() == osg::PrimitiveSet::DrawElementsUIntPrimitiveType) {
                    const std::vector<uint32_t> &sub_indicies = ((osg::DrawElementsUInt *)elements)->asVector();
                    indices.insert(end(indices), begin(sub_indicies), end(sub_indicies));
                }
            }
            // 把当前节点加入到输出结果
            if (vertices->getType() == osg::Array::Vec3ArrayType)
                add_triangles(float_vertices, indices);
            else if (vertices->getType() == osg::Array::Vec3dArrayType)
                add_triangles(double_vertices, indices);
        }
    }
}

template <typename T>
void ClipVisitor::add_triangles(const std::vector<T> &vertices, const std::vector<uint32_t> &faces) {
    CHECK(faces.size() % 3 == 0);
    int num_triangles = faces.size() / 3;

    auto get_vertex = [&](uint32_t f, uint32_t i) {
        Vector3d v;
        v(0) = vertices[faces[f * 3 + i]].x();
        v(1) = vertices[faces[f * 3 + i]].y();
        v(2) = vertices[faces[f * 3 + i]].z();
        return v;
    };

    bool use_obb = false;
    if (obb_.r(0) > 0 && obb_.r(1) > 0 && obb_.r(2) > 0) {
        use_obb = true;
    }
    OBB obb = to_mathgeolib(obb_);

    ///\brief 只有当输入的 bb，包含对应的三角形的重心，才加入他
    std::vector<uint32_t> valid_faces;
    for (int i = 0; i < num_triangles; ++i) {
        Vector3d v1 = get_vertex(i, 0);
        Vector3d v2 = get_vertex(i, 1);
        Vector3d v3 = get_vertex(i, 2);

        ///\brief 三角形的重心
        Vector3d bc = (v1 + v2 + v3) / 3.0;

        if (use_obb && obb.Contains(vec(bc.x(), bc.y(), bc.z()))) {
            valid_faces.push_back(i);
        } else if (!use_obb && bb_.contains(bc)) {
            valid_faces.push_back(i);
        }
    }

    if (vertices.size() > 0 && valid_faces.size() > 0) {
        // other 是新增区域的三角形
        GEO::Mesh other;
        if (vertices.size()) {
            /* 增加点 */
            other.vertices.create_vertices(vertices.size());
            for (int i = 0; i < vertices.size(); ++i) {
                other.vertices.point(i) = {(double)vertices[i].x(), (double)vertices[i].y(), (double)vertices[i].z()};
            }

            /* 增加面 */
            other.facets.create_triangles(valid_faces.size());
            for (int i = 0; i < valid_faces.size(); ++i) {
                other.facets.set_vertex(i, 0, faces[valid_faces[i] * 3]);
                other.facets.set_vertex(i, 1, faces[valid_faces[i] * 3 + 1]);
                other.facets.set_vertex(i, 2, faces[valid_faces[i] * 3 + 2]);
            }

            /* 移除没有用到的点 */
            other.vertices.remove_isolated();

            // 合并mesh
            mesh_merge(*mesh_, other);
        }
    }
}

OBB ClipVisitor::to_mathgeolib(const OBB3d &obb) {
    vec center(obb.center(0), obb.center(1), obb.center(2));
    vec r(obb.r(0), obb.r(1), obb.r(2));
    vec axis1(obb.axis[0](0), obb.axis[0](1), obb.axis[0](2));
    vec axis2(obb.axis[1](0), obb.axis[1](1), obb.axis[1](2));
    vec axis3(obb.axis[2](0), obb.axis[2](1), obb.axis[2](2));

    OBB obb2(center, r, axis1, axis2, axis3);
    return obb2;
}

void mesh_extract_load(osg::Node *node, const BoundingBox3d &bb, GEO::Mesh &mesh) {
    ClipVisitor visitor(bb, &mesh);
    node->accept(visitor);

    /* 会有很多相同的点，需要移除，采用geogram 的 mesh_repair算法 */
    if (mesh.vertices.nb() > 0) {
        mesh_repair(mesh, GEO::MESH_REPAIR_DEFAULT, EPSILON_LENGTH);
    }
}

void mesh_extract_path(const std::string &path, const BoundingBox3d &bb, GEO::Mesh &mesh) {
    osg::ref_ptr<osg::Node> root = osgDB::readRefNodeFile(path);
    mesh_extract_load(root, bb, mesh);
}

/************************************************************************/
/*                     ClipNoLoadVisitor                                */
/************************************************************************/
class ClipNoLoadVisitor : public ClipVisitor {
public:
    ClipNoLoadVisitor(const BoundingBox3d &_bb, GEO::Mesh *mesh);
    ClipNoLoadVisitor(const OBB3d &_bb, GEO::Mesh *mesh);
    virtual void apply(osg::PagedLOD &plod) override;
};

ClipNoLoadVisitor::ClipNoLoadVisitor(const BoundingBox3d &_bb, GEO::Mesh *mesh) : ClipVisitor(_bb, mesh) {
    setTraversalMode(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
}
ClipNoLoadVisitor::ClipNoLoadVisitor(const OBB3d &_bb, GEO::Mesh *mesh) : ClipVisitor(_bb, mesh) {
    setTraversalMode(osg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
}
void ClipNoLoadVisitor::apply(osg::PagedLOD &plod) {
    osg::BoundingSphere bs = plod.getBound();
    osg::BoundingBox bb;
    bb.expandBy(bs);

    if (osg_bb_.intersects(bb)) {
        traverse(plod);
    }
    return;
}

void mesh_extract_no_load(osg::Node *node, const BoundingBox3d &bb, GEO::Mesh &mesh) {
    ClipNoLoadVisitor visitor(bb, &mesh);
    node->accept(visitor);

    /* 会有很多相同的点，需要移除，采用geogram 的 mesh_repair算法 */
    if (mesh.vertices.nb() > 0) {
        mesh_repair(mesh, GEO::MESH_REPAIR_DEFAULT, EPSILON_LENGTH);
    }
}

void mesh_extract_no_load(osg::Node *node, const OBB3d &bb, GEO::Mesh &mesh) {
    ClipNoLoadVisitor visitor(bb, &mesh);
    node->accept(visitor);

    /* 会有很多相同的点，需要移除，采用geogram 的 mesh_repair算法 */
    if (mesh.vertices.nb() > 0) {
        mesh_repair(mesh, GEO::MESH_REPAIR_DEFAULT, EPSILON_LENGTH);
    }
}

/************************************************************************/
/*                       mesh_extract                                   */
/************************************************************************/
MeshPtr mesh_extract(osg::Node *node, const BoundingBox3d &bounds, bool load) {
    MeshPtr mesh = std::make_shared<GEO::Mesh>();
    if (!load) {
        mesh_extract_no_load(node, bounds, *mesh);
    }

    return mesh;
}

MeshPtr mesh_extract(osg::Node *node, const OBB3d bounds, bool load) {
    MeshPtr mesh = std::make_shared<GEO::Mesh>();
    if (!load) {
        mesh_extract_no_load(node, bounds, *mesh);
    }

    return mesh;
}

} // namespace h2o