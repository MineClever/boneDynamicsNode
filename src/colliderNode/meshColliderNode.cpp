#include "meshColliderNode.h"
#include "../nodeIds.h"
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MDistance.h>
#include <maya/MDGContext.h>
#include <maya/MDGContextGuard.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnPluginData.h>
#include <maya/MFnStringData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <algorithm>
#include <cmath>

MTypeId MeshColliderData::id(nodeIds::meshColliderData);
void* MeshColliderData::creator() { return new MeshColliderData; }
void MeshColliderData::copy(const MPxData& other)
{
    timeline = static_cast<const MeshColliderData&>(other).timeline;
}

MTypeId meshColliderNode::s_id(nodeIds::meshColliderNode);
MObject meshColliderNode::s_inputMesh, meshColliderNode::s_time;
MObject meshColliderNode::s_colorSet, meshColliderNode::s_excludeThreshold;
MObject meshColliderNode::s_timeSamples, meshColliderNode::s_contactOffset, meshColliderNode::s_continuous;
MObject meshColliderNode::s_rootOverlapFallback;
MObject meshColliderNode::s_output, meshColliderNode::s_geometryBuilds;
MObject meshColliderNode::s_sampleCount, meshColliderNode::s_topologyChanged;

void* meshColliderNode::creator() { return new meshColliderNode; }

MStatus meshColliderNode::initialize()
{
    MFnTypedAttribute typed;
    MFnNumericAttribute numeric;
    MFnUnitAttribute unit;
    MFnStringData string;
    s_inputMesh = typed.create("inputMesh", "im", MFnData::kMesh);
    s_time = unit.create("time", "tm", MFnUnitAttribute::kTime, 0);
    unit.setKeyable(false);
    s_colorSet = typed.create("excludeColorSet", "ecs", MFnData::kString, string.create("bdnCollisionExclude"));
    s_excludeThreshold = numeric.create("excludeThreshold", "et", MFnNumericData::kDouble, 0.5);
    numeric.setMin(0); numeric.setMax(1); numeric.setKeyable(true);
    s_timeSamples = numeric.create("timeSamples", "tsp", MFnNumericData::kInt, 4);
    numeric.setMin(1); numeric.setMax(64); numeric.setKeyable(false); numeric.setChannelBox(true);
    s_contactOffset = unit.create("contactOffset", "co", MFnUnitAttribute::kDistance, 0.001);
    unit.setMin(0); unit.setKeyable(true);
    s_continuous = numeric.create("continuous", "ccd", MFnNumericData::kBoolean, true);
    numeric.setKeyable(true);

    s_rootOverlapFallback = numeric.create("rootOverlapFallback", "rof", MFnNumericData::kBoolean, false);
    numeric.setKeyable(true);

    s_output = typed.create("outputCollider", "oc", MeshColliderData::id);
    typed.setWritable(false); typed.setStorable(false);
    s_geometryBuilds = numeric.create("geometryBuilds", "gb", MFnNumericData::kInt, 0);
    numeric.setWritable(false); numeric.setStorable(false);
    s_sampleCount = numeric.create("sampleCount", "sc", MFnNumericData::kInt, 0);
    numeric.setWritable(false); numeric.setStorable(false);
    s_topologyChanged = numeric.create("topologyChanged", "tc", MFnNumericData::kBoolean, false);
    numeric.setWritable(false); numeric.setStorable(false);
    const MObject inputs[] = {s_inputMesh, s_time, s_colorSet, s_excludeThreshold,
                             s_timeSamples, s_contactOffset, s_continuous, s_rootOverlapFallback};
    const MObject outputs[] = {s_output, s_geometryBuilds, s_sampleCount, s_topologyChanged};
    for (const auto& input : inputs) CHECK_MSTATUS_AND_RETURN_IT(addAttribute(input));
    for (const auto& output : outputs) CHECK_MSTATUS_AND_RETURN_IT(addAttribute(output));
    for (const auto& input : inputs) for (const auto& output : outputs)
        CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(input, output));
    return MS::kSuccess;
}

#if MAYA_API_VERSION >= 20220000
void meshColliderNode::getCacheSetup(const MEvaluationNode&, MNodeCacheDisablingInfo& info,
                                     MNodeCacheSetupInfo&, MObjectArray&) const
{
    info.setCacheDisabled(true);
    info.setReason("Mesh collider carries native query caches and sampled motion history.");
}
#endif

MStatus meshColliderNode::connectionMade(const MPlug& plug, const MPlug& other, bool asSource)
{
    if (plug.attribute() == s_inputMesh) ++m_policyRevision;
    return MPxNode::connectionMade(plug, other, asSource);
}

MStatus meshColliderNode::connectionBroken(const MPlug& plug, const MPlug& other, bool asSource)
{
    if (plug.attribute() == s_inputMesh) ++m_policyRevision;
    return MPxNode::connectionBroken(plug, other, asSource);
}

MStatus meshColliderNode::compute(const MPlug& plug, MDataBlock& data)
{
    if (plug != s_output && plug != s_geometryBuilds && plug != s_sampleCount && plug != s_topologyChanged)
        return MS::kUnknownParameter;
    MStatus status;
    const double now = data.inputValue(s_time).asTime().as(MTime::kSeconds);
    const bool normalContext = data.context().isNormal();
    const int samples = data.inputValue(s_timeSamples).asInt();
    const double threshold = data.inputValue(s_excludeThreshold).asDouble();
    const MString colorSet = data.inputValue(s_colorSet).asString();
    const double contactOffset = data.inputValue(s_contactOffset).asDistance().asCentimeters();
    if (!std::isfinite(now) || !std::isfinite(threshold) || !std::isfinite(contactOffset) ||
        contactOffset < 0 || samples < 1 || samples > 64)
        return MS::kInvalidParameter;
    auto timeline = std::make_shared<meshCollision::Timeline>();
    timeline->continuous = data.inputValue(s_continuous).asBool();
    timeline->contactOffset = contactOffset;
    timeline->rootOverlapFallback = data.inputValue(s_rootOverlapFallback).asBool();
    timeline->provider = MObjectHandle(thisMObject());

    auto capture = [&](const MObject& mesh, std::shared_ptr<const meshCollision::Snapshot>& value) -> MStatus {
        bool built = false;
        const MStatus captured = meshCollision::snapshot(mesh, colorSet, threshold, m_cache, value, built);
        if (built) ++m_builds;
        return captured;
    };
    std::shared_ptr<const meshCollision::Snapshot> current;
    CHECK_MSTATUS_AND_RETURN_IT(capture(data.inputValue(s_inputMesh).asMeshTransformed(), current));
    const MPlug input(thisMObject(), s_inputMesh);
    const MPlug source = input.source(&status);
    const bool policyChanged = !m_published || source != m_previousSource ||
        samples != m_previousSamples || colorSet != m_policyColorSet || threshold != m_policyThreshold ||
        timeline->continuous != m_published->continuous || contactOffset != m_published->contactOffset ||
        timeline->rootOverlapFallback != m_published->rootOverlapFallback;
    timeline->policyRevision = m_policyRevision + (policyChanged ? 1 : 0);
    const bool advancing = normalContext && m_hasPrevious && source == m_previousSource &&
                           now > m_previousSeconds && now - m_previousSeconds <= 1.0 / 15.0;
    int sampleCount = 1;
    const bool repeat = normalContext && m_hasPrevious && now == m_previousSeconds &&
                        source == m_previousSource && current == m_previous && m_published && samples == m_previousSamples;
    if (repeat) {
        // Multiple downstream bones (or forced same-time refreshes) must all see
        // the same motion interval, rather than replacing it with a zero-time
        // sample after the first consumer has advanced this provider's history.
        timeline->segments = m_published->segments;
        timeline->topologyChanged = m_published->topologyChanged;
        sampleCount = static_cast<int>(timeline->segments.size()) + 1;
        if (timeline->segments.size() == 1 && timeline->segments[0].begin == timeline->segments[0].end) sampleCount = 1;
    } else if (advancing) {
        auto previous = m_previous;
        double previousTime = m_previousSeconds;
        const int divisions = timeline->continuous ? samples : 1;
        for (int i = 1; i <= divisions; ++i) {
            const double t = m_previousSeconds + (now - m_previousSeconds) * (static_cast<double>(i) / divisions);
            auto value = current;
            if (i < divisions && !source.isNull()) {
                // Request upstream geometry only, never the collider output or
                // downstream solver, and never change MAnimControl's scene time.
                const MDGContext sampleContext(MTime(t, MTime::kSeconds));
                MDGContextGuard guard(sampleContext);
                MDataHandle sourceHandle = source.asMDataHandle(&status);
                if (!status) {
                    MGlobal::displayError("meshColliderNode: failed to evaluate the input mesh at a CCD time sample.");
                    return status;
                }
                const MStatus captured = capture(sourceHandle.asMeshTransformed(), value);
                source.destructHandle(sourceHandle);
                if (!captured) return captured;
            }
            timeline->segments.push_back(meshCollision::segment(previousTime, t, previous, value, timeline->topologyChanged));
            previous = value; previousTime = t; ++sampleCount;
        }
    } else {
        timeline->segments.push_back(meshCollision::segment(now, now, current, current, timeline->topologyChanged));
    }
    if (normalContext) {
        m_policyRevision = timeline->policyRevision;
        m_policyColorSet = colorSet; m_policyThreshold = threshold;
        m_previous = current; m_previousSeconds = now; m_hasPrevious = true;
        m_previousSource = source;
        m_published = timeline;
        m_previousSamples = samples;
    }
    // Keep a bounded strong cache. Published timelines retain their own immutable
    // references, so trimming never invalidates concurrent bone queries.
    const std::size_t limit = static_cast<std::size_t>(2 * std::max(1, samples) + 4);
    if (m_cache.size() > limit) m_cache.erase(m_cache.begin(), m_cache.end() - limit);

    MFnPluginData pluginData;
    const MObject object = pluginData.create(MeshColliderData::id, &status);
    if (!status) return status;
    auto* output = static_cast<MeshColliderData*>(pluginData.data(&status));
    if (!status) return status;
    output->timeline = timeline;
    data.outputValue(s_output).setMObject(object);
    data.outputValue(s_geometryBuilds).setInt(m_builds);
    data.outputValue(s_sampleCount).setInt(sampleCount);
    data.outputValue(s_topologyChanged).setBool(timeline->topologyChanged);
    CHECK_MSTATUS_AND_RETURN_IT(data.setClean(s_output));
    CHECK_MSTATUS_AND_RETURN_IT(data.setClean(s_geometryBuilds));
    CHECK_MSTATUS_AND_RETURN_IT(data.setClean(s_sampleCount));
    CHECK_MSTATUS_AND_RETURN_IT(data.setClean(s_topologyChanged));
    return MS::kSuccess;
}
