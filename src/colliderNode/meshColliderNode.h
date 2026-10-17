#pragma once

#include "meshCollision.h"
#include <maya/MPxData.h>
#include <maya/MPxNode.h>
#include <maya/MTime.h>
#include <maya/MPlug.h>
#include <maya/MTypeId.h>

class MeshColliderData : public MPxData
{
public:
    static MTypeId id;
    static void* creator();
    MTypeId typeId() const override { return id; }
    MString name() const override { return "boneDynamicsMeshColliderData"; }
    void copy(const MPxData& other) override;
    std::shared_ptr<const meshCollision::Timeline> timeline;
};

class meshColliderNode : public MPxNode
{
public:
    static MTypeId s_id;
    static MObject s_inputMesh, s_time, s_colorSet, s_excludeThreshold;
    static MObject s_timeSamples, s_contactOffset, s_continuous;
    static MObject s_output, s_geometryBuilds, s_sampleCount, s_topologyChanged;
    static void* creator();
    static MStatus initialize();
    MStatus compute(const MPlug& plug, MDataBlock& data) override;
    SchedulingType schedulingType() const override { return kGloballySerial; }
#if MAYA_API_VERSION >= 20220000
    void getCacheSetup(const MEvaluationNode&, MNodeCacheDisablingInfo&,
                      MNodeCacheSetupInfo&, MObjectArray&) const override;
#endif
private:
    std::vector<std::shared_ptr<const meshCollision::Snapshot>> m_cache;
    std::shared_ptr<const meshCollision::Snapshot> m_previous;
    double m_previousSeconds = 0;
    bool m_hasPrevious = false;
    MPlug m_previousSource;
    std::shared_ptr<const meshCollision::Timeline> m_published;
    int m_previousSamples = 0;
    int m_builds = 0;
};
