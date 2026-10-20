#include "meshCollision.h"
#include <maya/MColorArray.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <maya/MStringArray.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <mutex>

namespace meshCollision
{
namespace
{
    MVector mix(const MVector& a, const MVector& b, double t) { return a + (b - a) * t; }
    double clamp(double t) { return std::max(0.0, std::min(1.0, t)); }

    MVector closestOnEdge(const MVector& p, const MVector& a, const MVector& b)
    {
        const MVector ab = b - a;
        const double l2 = ab * ab;
        return a + ab * (l2 > 0 ? clamp(((p - a) * ab) / l2) : 0);
    }

    // Closest point over face, edges and vertices, including degenerate faces.
    MVector closestOnTriangle(const MVector& p, const std::array<MVector, 3>& v)
    {
        const MVector ab = v[1] - v[0], ac = v[2] - v[0], ap = p - v[0];
        const double d1 = ab * ap, d2 = ac * ap;
        if (d1 <= 0 && d2 <= 0) return v[0];
        const MVector bp = p - v[1];
        const double d3 = ab * bp, d4 = ac * bp;
        if (d3 >= 0 && d4 <= d3) return v[1];
        const double vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0 && d1 != d3)
            return v[0] + ab * (d1 / (d1 - d3));
        const MVector cp = p - v[2];
        const double d5 = ab * cp, d6 = ac * cp;
        if (d6 >= 0 && d5 <= d6) return v[2];
        const double vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0 && d2 != d6)
            return v[0] + ac * (d2 / (d2 - d6));
        const double va = d3 * d6 - d5 * d4;
        if (va <= 0 && d4 >= d3 && d5 >= d6 && d4 - d3 + d5 - d6 > 0)
            return v[1] + (v[2] - v[1]) * ((d4 - d3) / (d4 - d3 + d5 - d6));
        const double denominator = va + vb + vc;
        if (denominator > 0) return v[0] + ab * (vb / denominator) + ac * (vc / denominator);
        MVector best = v[0];
        double bestD2 = (p - best) * (p - best);
        for (int i = 0; i < 3; ++i) {
            const MVector q = closestOnEdge(p, v[i], v[(i + 1) % 3]);
            const double d2 = (p - q) * (p - q);
            if (d2 < bestD2) { bestD2 = d2; best = q; }
        }
        return best;
    }

    std::array<double, 3> barycentric(const MVector& p, const std::array<MVector, 3>& tri)
    {
        const MVector a = tri[1] - tri[0], b = tri[2] - tri[0], d = p - tri[0];
        const double aa = a * a, ab = a * b, bb = b * b;
        const double det = aa * bb - ab * ab;
        if (det <= std::numeric_limits<double>::epsilon() * aa * bb) return {{1, 0, 0}};
        const double v = (bb * (d * a) - ab * (d * b)) / det;
        const double w = (aa * (d * b) - ab * (d * a)) / det;
        return {{1 - v - w, v, w}};
    }

    struct Contact
    {
        double clearance = std::numeric_limits<double>::infinity();
        double boneT = 1;
        MVector bonePoint, meshPoint;
    };

    Contact separation(const MVector& root, const MVector& tip, double rootRadius,
                       double tipRadius, bool capsule, const std::array<MVector, 3>& tri)
    {
        auto at = [&](double t) {
            Contact c;
            c.boneT = t;
            c.bonePoint = mix(root, tip, t);
            c.meshPoint = closestOnTriangle(c.bonePoint, tri);
            c.clearance = (c.bonePoint - c.meshPoint).length() -
                          (rootRadius + (tipRadius - rootRadius) * t);
            return c;
        };
        if (!capsule) return at(1);

        // distance(segment(t), triangle) - linearRadius(t) is convex. Minimize
        // over the entire tapered capsule, rather than sampling a few spheres.
        double lo = 0, hi = 1;
        constexpr double ratio = 0.6180339887498948482;
        Contact x = at(hi - ratio * (hi - lo)), y = at(lo + ratio * (hi - lo));
        for (int i = 0; i < 40; ++i) {
            if (x.clearance < y.clearance) {
                hi = y.boneT; y = x; x = at(hi - ratio * (hi - lo));
            } else {
                lo = x.boneT; x = y; y = at(lo + ratio * (hi - lo));
            }
        }
        Contact best = x.clearance < y.clearance ? x : y;
        const Contact first = at(0), last = at(1);
        if (first.clearance < best.clearance) best = first;
        if (last.clearance < best.clearance) best = last;
        // Conservative error bound prevents minimization roundoff from skipping
        // zero-thickness contacts during conservative advancement.
        best.clearance -= ((tip - root).length() + std::abs(tipRadius - rootRadius)) * (hi - lo);
        return best;
    }

    std::array<MVector, 3> triangleAt(const Segment& segment, unsigned int index, double t)
    {
        std::array<MVector, 3> result;
        for (int j = 0; j < 3; ++j) {
            const auto vertex = segment.a->triangles[index][j];
            result[j] = mix(segment.a->points[vertex], segment.b->points[vertex], t);
        }
        return result;
    }

    // The fixed root imposes a sphere constraint on the tip. Project directly
    // onto the sphere/halfspace intersection, so a final length correction does
    // not immediately undo collision separation.
    bool project(const MVector& root, double length, MVector& tip, const MVector& point,
                 const MVector& normal, double boneT, double radius, double tolerance)
    {
        if (boneT <= 1e-8) return ((root - point) * normal) >= radius - tolerance;
        const double required = (radius - (root - point) * normal) / boneT;
        MVector v = tip - root;
        if (v * normal >= required - tolerance) return true;
        if (required > length) { tip = root + normal * length; return false; }
        const double d = std::max(-length, required);
        MVector tangent = v - normal * (v * normal);
        if (tangent.length() <= tolerance) {
            tangent = normal ^ (std::abs(normal.x) < 0.8 ? MVector::xAxis : MVector::yAxis);
        }
        tip = root + normal * d + tangent.normal() * std::sqrt(std::max(0.0, length * length - d * d));
        return true;
    }

    struct Hit { bool hit = false; bool limited = false; double time = 1; Contact contact; MVector normal; };

    Hit sweepTriangle(const Sweep& sweep, const std::array<MVector, 3>& start,
                      const std::array<MVector, 3>& end, double offset, bool continuous)
    {
        Hit hit;
        const MVector rootVelocity = sweep.root1 - sweep.root0, tipVelocity = sweep.tip1 - sweep.tip0;
        double meshSpeed = 0;
        for (int j = 0; j < 3; ++j) meshSpeed = std::max(meshSpeed, (end[j] - start[j]).length());
        const double speed = std::max(rootVelocity.length(), tipVelocity.length()) + meshSpeed;
        const double tolerance = std::max(1e-9, std::max((sweep.tip1 - sweep.root1).length(), speed) * 1e-9);
        double t = continuous ? 0 : 1;
        for (int iteration = 0; iteration < 128; ++iteration) {
            std::array<MVector, 3> tri;
            for (int j = 0; j < 3; ++j) tri[j] = mix(start[j], end[j], t);
            Contact contact = separation(mix(sweep.root0, sweep.root1, t), mix(sweep.tip0, sweep.tip1, t),
                                         sweep.rootRadius, sweep.tipRadius, sweep.capsule, tri);
            if (contact.clearance <= offset + tolerance || iteration == 127) {
                hit.hit = true;
                hit.limited = iteration == 127;
                hit.time = t;
                hit.contact = contact;
                const MVector difference = contact.bonePoint - contact.meshPoint;
                if (difference.length() > tolerance) hit.normal = difference.normal();
                else {
                    hit.normal = ((tri[1] - tri[0]) ^ (tri[2] - tri[0])).normal();
                    const auto bary = barycentric(contact.meshPoint, tri);
                    MVector surfaceVelocity;
                    for (int j = 0; j < 3; ++j) surfaceVelocity += (end[j] - start[j]) * bary[j];
                    const MVector relativeVelocity = mix(rootVelocity, tipVelocity, contact.boneT) - surfaceVelocity;
                    if (hit.normal * relativeVelocity > 0) hit.normal = -hit.normal;
                }
                return hit;
            }
            if (!continuous || t >= 1 || speed <= tolerance) return hit;
            const double advance = 0.9 * (contact.clearance - offset) / speed;
            if (t + advance > 1) { t = 1; continue; }
            t += std::max(advance, std::numeric_limits<double>::epsilon());
        }
        return hit;
    }
}

Bounds::Bounds()
{
    const double inf = std::numeric_limits<double>::infinity();
    lo = MVector(inf, inf, inf); hi = -lo;
}
void Bounds::include(const MVector& p)
{
    for (int i = 0; i < 3; ++i) { lo[i] = std::min(lo[i], p[i]); hi[i] = std::max(hi[i], p[i]); }
}
void Bounds::expand(double r) { lo -= MVector(r, r, r); hi += MVector(r, r, r); }
bool Bounds::overlaps(const Bounds& b) const
{
    for (int i = 0; i < 3; ++i) if (hi[i] < b.lo[i] || lo[i] > b.hi[i]) return false;
    return true;
}

TriangleTree::TriangleTree(const std::vector<MVector>& a, const std::vector<MVector>& b,
                           const std::vector<Triangle>& triangles)
{
    m_order.resize(triangles.size());
    std::iota(m_order.begin(), m_order.end(), 0u);
    m_bounds.resize(triangles.size());
    for (unsigned int i = 0; i < triangles.size(); ++i)
        for (auto vertex : triangles[i]) { m_bounds[i].include(a[vertex]); m_bounds[i].include(b[vertex]); }
    if (!triangles.empty()) build(0, static_cast<unsigned int>(triangles.size()));
}
int TriangleTree::build(unsigned int begin, unsigned int end)
{
    const int index = static_cast<int>(m_nodes.size());
    m_nodes.emplace_back();
    Bounds bounds;
    for (unsigned int i = begin; i < end; ++i) { bounds.include(m_bounds[m_order[i]].lo); bounds.include(m_bounds[m_order[i]].hi); }
    m_nodes[index].bounds = bounds;
    m_nodes[index].begin = begin; m_nodes[index].end = end;
    if (end - begin > 8) {
        const MVector extent = bounds.hi - bounds.lo;
        int axis = extent.y > extent.x ? 1 : 0;
        if (extent.z > extent[axis]) axis = 2;
        const unsigned int middle = begin + (end - begin) / 2;
        std::nth_element(m_order.begin() + begin, m_order.begin() + middle, m_order.begin() + end,
            [&](unsigned int a, unsigned int b) { return m_bounds[a].lo[axis] + m_bounds[a].hi[axis] < m_bounds[b].lo[axis] + m_bounds[b].hi[axis]; });
        const int left = build(begin, middle), right = build(middle, end);
        m_nodes[index].left = left; m_nodes[index].right = right;
    }
    return index;
}
void TriangleTree::query(const Bounds& bounds, std::vector<unsigned int>& result) const
{
    if (m_nodes.empty()) return;
    std::vector<int> stack(1, 0);
    while (!stack.empty()) {
        const Node& n = m_nodes[stack.back()]; stack.pop_back();
        if (!n.bounds.overlaps(bounds)) continue;
        if (n.left >= 0) { stack.push_back(n.left); stack.push_back(n.right); }
        else for (unsigned int i = n.begin; i < n.end; ++i)
            if (m_bounds[m_order[i]].overlaps(bounds)) result.push_back(m_order[i]);
    }
}

bool Snapshot::closest(const MVector& p, MVector& q, MVector& normal, unsigned int& triangle) const
{
    if (!intersector) return false;
    MPointOnMesh result;
    if (!intersector->getClosestPoint(MPoint(p), result)) return false;
    const int face = result.faceIndex(); // cached mesh consists of triangle polygons
    if (face < 0 || static_cast<std::size_t>(face) >= triangles.size()) return false;
    triangle = static_cast<unsigned int>(face);
    const auto& ids = triangles[triangle];
    const std::array<MVector, 3> tri{{points[ids[0]], points[ids[1]], points[ids[2]]}};
    q = closestOnTriangle(p, tri); // refine float API result using double geometry
    normal = ((tri[1] - tri[0]) ^ (tri[2] - tri[0])).normal();
    return true;
}

MStatus snapshot(const MObject& mesh, const MString& colorSet, double threshold,
                 std::vector<std::shared_ptr<const Snapshot>>& cache,
                 std::shared_ptr<const Snapshot>& result, bool& built)
{
    built = false;
    // Legacy direct mesh inputs may be consumed by multiple parallel bones.
    // Serialize Maya extraction/build calls; published queries stay lock-free.
    static std::mutex extractionMutex;
    const std::lock_guard<std::mutex> lock(extractionMutex);
    auto candidate = std::make_shared<Snapshot>();
    MStatus status;
    if (!mesh.isNull()) {
        MFnMesh fn(mesh, &status); if (!status) return status;
        MPointArray points;
        CHECK_MSTATUS_AND_RETURN_IT(fn.getPoints(points, MSpace::kObject));
        candidate->points.reserve(points.length());
        for (unsigned int i = 0; i < points.length(); ++i) {
            const auto& p = points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return MS::kInvalidParameter;
            candidate->points.emplace_back(p.x, p.y, p.z);
        }
        MIntArray faceCounts, faceVertices, triangleCounts, triangleVertices;
        CHECK_MSTATUS_AND_RETURN_IT(fn.getVertices(faceCounts, faceVertices));
        CHECK_MSTATUS_AND_RETURN_IT(fn.getTriangles(triangleCounts, triangleVertices));
        MColorArray colors;
        MStringArray colorSets;
        CHECK_MSTATUS_AND_RETURN_IT(fn.getColorSetNames(colorSets));
        bool hasMask = false;
        for (unsigned int i = 0; i < colorSets.length(); ++i) if (colorSet.length() && colorSets[i] == colorSet) hasMask = true;
        if (hasMask) {
            const MColor defaultColor(-1, -1, -1, -1);
            CHECK_MSTATUS_AND_RETURN_IT(fn.getFaceVertexColors(colors, &colorSet, &defaultColor));
        }
        unsigned int corner = 0, triangle = 0;
        for (unsigned int face = 0; face < faceCounts.length(); ++face) {
            bool excluded = hasMask && faceCounts[face] > 0;
            for (int i = 0; i < faceCounts[face]; ++i, ++corner)
                if (!hasMask || corner >= colors.length() || colors[corner].r < 0 || !std::isfinite(colors[corner].r) ||
                    colors[corner].r < threshold) excluded = false;
            for (int i = 0; i < triangleCounts[face]; ++i, triangle += 3) {
                if (excluded) continue;
                Triangle ids{{static_cast<unsigned int>(triangleVertices[triangle]),
                              static_cast<unsigned int>(triangleVertices[triangle + 1]),
                              static_cast<unsigned int>(triangleVertices[triangle + 2])}};
                for (auto vertex : ids) if (vertex >= candidate->points.size()) return MS::kInvalidParameter;
                const MVector u = candidate->points[ids[1]] - candidate->points[ids[0]];
                const MVector v = candidate->points[ids[2]] - candidate->points[ids[0]];
                if ((u ^ v).length() <= std::numeric_limits<double>::epsilon() * u.length() * v.length()) continue;
                candidate->triangles.push_back(ids);
            }
        }
    }
    std::uint64_t hash = 1469598103934665603ull;
    auto hashWord = [&](std::uint64_t word) { hash ^= word; hash *= 1099511628211ull; };
    for (const auto& p : candidate->points) for (int i = 0; i < 3; ++i) {
        std::uint64_t bits; const double component = p[i]; std::memcpy(&bits, &component, sizeof(bits)); hashWord(bits);
    }
    for (const auto& tri : candidate->triangles) for (auto i : tri) hashWord(i);
    candidate->hash = hash;
    for (const auto& cached : cache) if (cached->hash == hash && cached->points == candidate->points && cached->triangles == candidate->triangles) {
        result = cached; return MS::kSuccess;
    }
    if (!candidate->triangles.empty()) {
        MFnMeshData owner;
        candidate->ownedMesh = owner.create(&status); if (!status) return status;
        MPointArray points;
        for (const auto& p : candidate->points) points.append(MPoint(p));
        MIntArray counts, vertices;
        for (const auto& tri : candidate->triangles) { counts.append(3); for (auto i : tri) vertices.append(static_cast<int>(i)); }
        MFnMesh fn;
        fn.create(static_cast<int>(points.length()), static_cast<int>(counts.length()), points, counts, vertices, candidate->ownedMesh, &status);
        if (!status) return status;
        candidate->intersector = std::make_unique<MMeshIntersector>();
        CHECK_MSTATUS_AND_RETURN_IT(candidate->intersector->create(candidate->ownedMesh));
        built = true;
    }
    candidate->tree = std::make_shared<TriangleTree>(candidate->points, candidate->points, candidate->triangles);
    result = candidate; cache.push_back(candidate);
    return MS::kSuccess;
}

Segment segment(double begin, double end, std::shared_ptr<const Snapshot> a,
                std::shared_ptr<const Snapshot> b, bool& topologyChanged)
{
    Segment result; result.begin = begin; result.end = end;
    if (a->points.size() != b->points.size() || a->triangles != b->triangles) {
        topologyChanged = true;
        a = b; // no vertex correspondence: use current surface, report lost history
    }
    result.a = a; result.b = b;
    result.tree = a == b ? b->tree : std::make_shared<TriangleTree>(a->points, b->points, b->triangles);
    return result;
}

Result solve(const Timeline& collider, const Sweep& motion, double begin, double end, MVector& tip)
{
    Result result;
    const double length = (motion.tip1 - motion.root1).length();
    // Map directed scene times to increasing playback coordinates. Geometry
    // stays in evaluation order, including when the scene frames decrease.
    const double direction = end < begin ? -1.0 : 1.0;
    begin *= direction;
    end *= direction;
    for (const auto& interval : collider.segments) {
        const double intervalBegin = interval.begin * direction;
        const double intervalEnd = interval.end * direction;
        // A newly connected fallback provider has only the current snapshot.
        // Resolve against that surface immediately; no prior mesh motion is known.
        const bool instantFallback = collider.rootOverlapFallback && !motion.capsule && intervalBegin == intervalEnd;
        const double start = instantFallback ? begin : std::max(begin, intervalBegin);
        const double finish = instantFallback ? end : std::min(end, intervalEnd);
        if (finish < start || (finish == start && end > begin)) continue;
        const double intervalDuration = intervalEnd - intervalBegin;
        const double ta = intervalDuration > 0 ? clamp((start - intervalBegin) / intervalDuration) : 1;
        const double tb = intervalDuration > 0 ? clamp((finish - intervalBegin) / intervalDuration) : 1;
        const double fa = end > begin ? clamp((start - begin) / (end - begin)) : 1;
        const double fb = end > begin ? clamp((finish - begin) / (end - begin)) : 1;
        Sweep sweep = motion;
        sweep.root0 = mix(motion.root0, motion.root1, fa);
        sweep.root1 = mix(motion.root0, motion.root1, fb);
        sweep.tip0 = mix(motion.tip0, tip, fa);
        sweep.tip1 = mix(motion.tip0, tip, fb);
        Bounds bounds;
        bounds.include(sweep.tip0); bounds.include(sweep.tip1);
        if (motion.capsule) { bounds.include(sweep.root0); bounds.include(sweep.root1); }
        bounds.expand(std::max(motion.rootRadius, motion.tipRadius) + collider.contactOffset);
        std::vector<unsigned int> candidates;
        interval.tree->query(bounds, candidates);

        // Native point proximity query is especially efficient for resting tips.
        // Include its closest triangle as well as the swept broad-phase results.
        if (interval.a == interval.b && !motion.capsule) {
            MVector q, normal; unsigned int index;
            if (interval.b->closest(sweep.tip1, q, normal, index) &&
                (sweep.tip1 - q).length() <= motion.tipRadius + collider.contactOffset)
                candidates.push_back(index);
        }
        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
        for (auto index : candidates) {
            const auto tri0 = triangleAt(interval, index, ta), tri1 = triangleAt(interval, index, tb);
            const Hit hit = sweepTriangle(sweep, tri0, tri1, collider.contactOffset, collider.continuous);
            if (!hit.hit) continue;
            result.collided = true;
            result.unresolved |= hit.limited;
            std::array<MVector, 3> triHit;
            for (int j = 0; j < 3; ++j) triHit[j] = mix(tri0[j], tri1[j], hit.time);
            const auto bary = barycentric(hit.contact.meshPoint, triHit);
            // Carry the contact to the end of the full bone substep. A moving
            // surface must push the bone, rather than merely stopping it at TOI.
            const double endT = intervalDuration > 0 ? clamp((end - intervalBegin) / intervalDuration) : 1;
            const auto triEnd = triangleAt(interval, index, endT);
            MVector surfacePoint;
            for (int j = 0; j < 3; ++j) surfacePoint += triEnd[j] * bary[j];
            MVector normal = hit.normal;
            if (normal.length() < 0.5) { result.unresolved = true; continue; }
            const double radius = motion.rootRadius + (motion.tipRadius - motion.rootRadius) * hit.contact.boneT;
            const bool feasible = project(motion.root1, length, tip, surfacePoint, normal,
                                          hit.contact.boneT, radius + collider.contactOffset * 1.01, 1e-9);
            result.unresolved |= !feasible;
        }
    }
    return result;
}

bool rootOverlaps(const Timeline& collider, const Sweep& motion, double begin, double end, double padding)
{
    const double radius = std::max(0.0, motion.rootRadius + collider.contactOffset + padding);
    // Map directed scene times to increasing playback coordinates. Geometry
    // stays in evaluation order, including when the scene frames decrease.
    const double direction = end < begin ? -1.0 : 1.0;
    begin *= direction;
    end *= direction;
    for (const auto& interval : collider.segments) {
        const double intervalBegin = interval.begin * direction;
        const double intervalEnd = interval.end * direction;
        const bool instant = intervalBegin == intervalEnd;
        const double start = instant ? begin : std::max(begin, intervalBegin);
        const double finish = instant ? end : std::min(end, intervalEnd);
        if (finish < start || (finish == start && end > begin)) continue;
        const double duration = intervalEnd - intervalBegin;
        const double ta = duration > 0 ? clamp((start - intervalBegin) / duration) : 1;
        const double tb = duration > 0 ? clamp((finish - intervalBegin) / duration) : 1;
        const double fa = end > begin ? clamp((start - begin) / (end - begin)) : 1;
        const double fb = end > begin ? clamp((finish - begin) / (end - begin)) : 1;
        Sweep root;
        root.tip0 = root.root0 = mix(motion.root0, motion.root1, fa);
        root.tip1 = root.root1 = mix(motion.root0, motion.root1, fb);
        root.tipRadius = root.rootRadius = radius;
        if (interval.a == interval.b && root.tip0 == root.tip1) {
            MVector q, normal; unsigned int index;
            if (interval.b->closest(root.tip1, q, normal, index) && (root.tip1 - q).length() < radius)
                return true;
            continue;
        }
        Bounds bounds;
        bounds.include(root.tip0); bounds.include(root.tip1); bounds.expand(radius);
        std::vector<unsigned int> candidates;
        interval.tree->query(bounds, candidates);
        for (auto index : candidates) {
            const Hit hit = sweepTriangle(root, triangleAt(interval, index, ta),
                                           triangleAt(interval, index, tb), 0, true);
            if (hit.hit) return true; // A limited sweep conservatively keeps fallback.
        }
    }
    return false;
}

bool penetrating(const Timeline& collider, const Sweep& motion, double seconds, const MVector& tip)
{
    const double tolerance = std::max(1e-8, (tip - motion.root1).length() * 1e-8);
    for (const auto& interval : collider.segments) {
        if (seconds < std::min(interval.begin, interval.end) - 1e-10 ||
            seconds > std::max(interval.begin, interval.end) + 1e-10) continue;
        const double duration = interval.end - interval.begin;
        const double t = duration != 0 ? clamp((seconds - interval.begin) / duration) : 1;
        if (!motion.capsule && interval.a == interval.b) {
            MVector q, normal; unsigned int index;
            if (interval.b->closest(tip, q, normal, index))
                return (tip - q).length() < motion.tipRadius + collider.contactOffset - tolerance;
            return false;
        }
        Bounds bounds;
        bounds.include(tip);
        if (motion.capsule) bounds.include(motion.root1);
        bounds.expand(std::max(motion.rootRadius, motion.tipRadius) + collider.contactOffset);
        std::vector<unsigned int> candidates;
        interval.tree->query(bounds, candidates);
        for (auto index : candidates) {
            const auto tri = triangleAt(interval, index, t);
            if (separation(motion.root1, tip, motion.rootRadius, motion.tipRadius, motion.capsule, tri).clearance <
                collider.contactOffset - tolerance) return true;
        }
        return false;
    }
    return false;
}
}
