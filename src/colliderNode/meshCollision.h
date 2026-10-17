#pragma once

#include <maya/MMatrix.h>
#include <maya/MMeshIntersector.h>
#include <maya/MObject.h>
#include <maya/MObjectHandle.h>
#include <maya/MString.h>
#include <maya/MVector.h>
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace meshCollision
{
    using Triangle = std::array<unsigned int, 3>;

    struct Bounds
    {
        MVector lo, hi;
        Bounds();
        void include(const MVector& point);
        void expand(double amount);
        bool overlaps(const Bounds& other) const;
    };

    // A small native swept-AABB broad phase supplements MMeshIntersector,
    // which supplies point queries but has no capsule/triangle sweep API.
    class TriangleTree
    {
    public:
        TriangleTree(const std::vector<MVector>& a, const std::vector<MVector>& b,
                     const std::vector<Triangle>& triangles);
        void query(const Bounds& bounds, std::vector<unsigned int>& result) const;
    private:
        struct Node { Bounds bounds; int left = -1, right = -1; unsigned int begin = 0, end = 0; };
        int build(unsigned int begin, unsigned int end);
        std::vector<Node> m_nodes;
        std::vector<unsigned int> m_order;
        std::vector<Bounds> m_bounds;
    };

    struct Snapshot
    {
        std::vector<MVector> points;
        std::vector<Triangle> triangles;
        std::uint64_t hash = 0;
        MObject ownedMesh;
        std::unique_ptr<MMeshIntersector> intersector;
        std::shared_ptr<const TriangleTree> tree;
        bool closest(const MVector& point, MVector& closestPoint, MVector& normal,
                     unsigned int& triangle) const;
    };

    struct Segment
    {
        double begin = 0, end = 0;
        std::shared_ptr<const Snapshot> a, b;
        std::shared_ptr<const TriangleTree> tree;
    };

    struct Timeline
    {
        std::vector<Segment> segments;
        double contactOffset = 0.001; // world centimeters
        bool continuous = true;
        bool topologyChanged = false;
        bool rootOverlapFallback = false;
        MObjectHandle provider;
        std::uint64_t policyRevision = 0;
    };

    // Called only while the mesh provider is serialized, never by query workers.
    MStatus snapshot(const MObject& mesh, const MString& colorSet, double excludeThreshold,
                     std::vector<std::shared_ptr<const Snapshot>>& cache,
                     std::shared_ptr<const Snapshot>& result, bool& built);
    Segment segment(double begin, double end, std::shared_ptr<const Snapshot> a,
                    std::shared_ptr<const Snapshot> b, bool& topologyChanged);

    struct Sweep
    {
        MVector root0, tip0, root1, tip1;
        double rootRadius = 0, tipRadius = 0;
        bool capsule = false;
    };

    // Preserve bone length while resolving a two-sided triangle surface.
    // All arguments are world-space, independently of a bone's simulationScale.
    struct Result { bool collided = false; bool unresolved = false; };
    Result solve(const Timeline& collider, const Sweep& motion, double beginSeconds,
                 double endSeconds, MVector& tip);
    bool penetrating(const Timeline& collider, const Sweep& motion, double seconds,
                     const MVector& tip);
    // Swept root-surface proximity, independent of endpoint CCD policy.
    bool rootOverlaps(const Timeline& collider, const Sweep& motion, double beginSeconds,
                      double endSeconds, double padding);
}
