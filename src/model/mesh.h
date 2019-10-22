/*
 * @Author: Han Hu
 * @Date: 2018-06-21 20:23:30
 * 三角网模型处理
 */
#pragma once

#include <base/common.h>
#include <modelpro/obb.h>
#include <set>

namespace GEO {
class Mesh;
}
namespace osg {
class Node;
}
using MeshPtr = std::shared_ptr<GEO::Mesh>;

namespace h2o {
constexpr static double EPSILON_LENGTH = 0.0005; // Sketchup support 1/1000 precision
constexpr static double EPSILON_ANGLE = 0.0000000000005;
constexpr static double INV_EPSILON_LENGTH = 2000.0;

/**
 * \brief 初始化 geogram
 */
void mesh_initilize();

MeshPtr mesh_load(const std::string &path);

/**
 * \brief 从osg node 中提取 mesh
 * \param node 可以是多种不同形式的node，包含 pagedlod
 */
MeshPtr mesh_extract(osg::Node *node, const BoundingBox3d &bounds, bool load = false);

/**
 * \brief 从 Sketchup enumerable 中提取 mesh
 * \param entity entity 可以是多种不同的形式
 */
MeshPtr mesh_extract(VALUE enumerable, const BoundingBox3d &bounds);

/*
 * \brief 从 osg node 中提取在当前 OBB 中的mesh
 */
MeshPtr mesh_extract(osg::Node *node, const OBB3d, bool load = false);

// V和F都是column major
MeshPtr mesh_extract(const MatrixXd &V, const MatrixXu &F);

/**
 * \brief 将两个 mesh 合并
 * \param target 输入输出的 mesh
 * \param other 需要合并到 target 的 mesh
 */
void mesh_merge(GEO::Mesh &target, const GEO::Mesh &other);
/**
 * \brief 从当前 mesh 中删除 bb内的内容
 * \param mesh 当前 mesh
 * \param bb 待删除区域的bb
 */
void mesh_remove_by_bbox(GEO::Mesh &mesh, BoundingBox3d &bb);

/**
 * \brief 获取 mesh 的 包围盒
 * \param mesh
 */
BoundingBox3d mesh_bbox(const GEO::Mesh &mesh);

/**
 * \brief 获取 mesh 每个 face 的重心
 * \param mesh
 */
std::vector<Vector3d> mesh_barycenter(const GEO::Mesh &mesh);

/**
 * \brief 采用 isometric_remesh 算法进行 remehs
 * \param M 输入 mesh
 * \param M_out 输出 mesh
 * \param target_length 目标的边的长度
 * \param edges_fix 需要保持固定的边的索引
 * \param iter 迭代次数
 */
void mesh_remesh(const GEO::Mesh &M, GEO::Mesh &M_out, double target_length, const std::vector<uint32_t> &edges_fix,
                 int iter = 1);

/**
 * \brief 采用 isometric_remesh 算法进行 remehs
 * \param M 输入 mesh
 * \param M_out 输出 mesh
 * \param target_length 目标的边的长度
 * \param edges_fix 需要固定的边，边采用顶点索引的pair进行表达
 * \param iter 迭代次数
 */
void mesh_remesh(const GEO::Mesh &M, GEO::Mesh &M_out, double target_length,
                 const std::set<std::pair<uint32_t, uint32_t>> &edges_fix, int iter = 1);

/**
 * \brief 采用中点分割的方式分割mesh
 * \param M
 * \param max_length 目标边长
 * \param max_iter 最大迭代次数
 */
void mesh_subdivision(GEO::Mesh &M, double max_length, int max_iter = 5);

// 变换 mesh 的定点
void mesh_transform(GEO::Mesh &M, const Matrix4d &xform);

} // namespace h2o
