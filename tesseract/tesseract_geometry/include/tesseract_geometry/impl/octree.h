/**
 * @file octree.h
 * @brief Tesseract Octree Geometry
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_GEOMETRY_OCTREE_H
#define TESSERACT_GEOMETRY_OCTREE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace tesseract_geometry
{
  class Octree;
  typedef std::shared_ptr<Octree> OctreePtr;
  typedef std::shared_ptr<const Octree> OctreeConstPtr;

  class Octree : public Geometry
  {
  public:
    enum SubType
    {
      BOX,
      SPHERE_INSIDE,
      SPHERE_OUTSIDE
    };

    Octree(const std::shared_ptr<const octomap::OcTree>& octree, SubType sub_type)  : Geometry(GeometryType::OCTREE), octree_(octree), sub_type_(sub_type) {}
    ~Octree() override = default;

    const std::shared_ptr<const octomap::OcTree>& getOctree() const { return octree_; }
    SubType getSubType() const { return sub_type_; }

    GeometryPtr clone() const override { return OctreePtr(new Octree(octree_, sub_type_)); }

    /**
     * @brief Octrees are typically generated from 3D sensor data so this method
     * should be used to efficiently update the collision shape.
     */
    void update() { assert(false); }

    static void prune(octomap::OcTree& octree)
    {
      if (octree.getRoot() == nullptr)
        return;

      for (unsigned int depth = octree.getTreeDepth() - 1; depth > 0; --depth)
      {
        unsigned int num_pruned = 0;
        pruneRecurs(octree, octree.getRoot(), 0, depth, num_pruned);
        if (num_pruned == 0)
          break;
      }
    }

  private:
    std::shared_ptr<const octomap::OcTree> octree_;
    SubType sub_type_;

    static bool isNodeCollapsible(octomap::OcTree& octree, octomap::OcTreeNode* node)
    {
      if (!octree.nodeChildExists(node, 0))
        return false;

      double occupancy_threshold = octree.getOccupancyThres();

      const octomap::OcTreeNode* firstChild = octree.getNodeChild(node, 0);
      if (octree.nodeHasChildren(firstChild) || firstChild->getOccupancy() < occupancy_threshold)
        return false;

      for (unsigned int i = 0; i < 8; i++)
      {
        // comparison via getChild so that casts of derived classes ensure that the right == operator gets called
        if (!octree.nodeChildExists(node, i))
          return false;

        if (octree.nodeHasChildren(octree.getNodeChild(node, i)))
          return false;

        if (octree.getNodeChild(node, i)->getOccupancy() < occupancy_threshold)
          return false;
      }

      return true;
    }

    static bool pruneNode(octomap::OcTree& octree, octomap::OcTreeNode* node)
    {
      if (!isNodeCollapsible(octree, node))
        return false;

      // set value to children's values (all assumed equal)
      node->copyData(*(octree.getNodeChild(node, 0)));

      // delete children (known to be leafs at this point)
      for (unsigned int i = 0; i < 8; i++)
      {
        octree.deleteNodeChild(node, i);
      }

      return true;
    }

    static void pruneRecurs(octomap::OcTree& octree,
                            octomap::OcTreeNode* node,
                            unsigned int depth,
                            unsigned int max_depth,
                            unsigned int& num_pruned)
    {
      assert(node);

      if (depth < max_depth)
      {
        for (unsigned int i = 0; i < 8; i++)
        {
          if (octree.nodeChildExists(node, i))
          {
            pruneRecurs(octree, octree.getNodeChild(node, i), depth + 1, max_depth, num_pruned);
          }
        }
      } // end if depth

      else
      {
        // max level reached
        if (pruneNode(octree, node))
        {
          num_pruned++;
        }
      }

      return;
    } // end pruneRecurs()


  };
}
#endif
