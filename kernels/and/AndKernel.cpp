/******************************************************************************
* Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include "AndKernel.hpp"

#include "../io/buffer/BufferReader.hpp"
#include <pdal/KDIndex.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo("kernels.and", "AND Kernel",
    "http://pdal.io/apps/and.html" );

CREATE_STATIC_PLUGIN(1, 0, AndKernel, Kernel, s_info)

std::string AndKernel::getName() const
{
    return s_info.name;
}


void AndKernel::addSwitches(ProgramArgs& args)
{
    args.add("full,f", "Full filename", m_fullFile).setPositional();
    args.add("sampled,s", "Sampled filename", m_sampFile).setPositional();
    args.add("output,o", "Output filename", m_outputFile).setPositional();
    args.add("tolerance,t", "Tolerance", m_tolerance, 0.0001);
    args.add("dimension,d", "Dimension name", m_dimName);
}


int AndKernel::execute()
{
    PointTable fullTable;
    Stage& fullStage = makeReader(m_fullFile, m_driverOverride);
    fullStage.prepare(fullTable);
    
    PointTable sampTable;
    Stage& sampStage = makeReader(m_sampFile, m_driverOverride);
    sampStage.prepare(sampTable);
    
    PointLayoutPtr layout(fullTable.layout());
    Dimension::Id dim = layout->registerOrAssignDim(m_dimName, Dimension::Type::Unsigned8);
    PointViewPtr outView(new PointView(fullTable));
        
    PointViewSet fullViewSet = fullStage.execute(fullTable);
    PointViewPtr fullView = *fullViewSet.begin();
    PointViewSet sampViewSet = sampStage.execute(sampTable);
    PointViewPtr sampView = *sampViewSet.begin();
    
    KD3Index sampIndex(*sampView);
    sampIndex.build();
    
    for (PointId i = 0; i < fullView->size(); ++i)
    {
        std::vector<PointId> indices(1);
        std::vector<double> sqr_dists(1);
        PointRef fullPoint = fullView->point(i);
        sampIndex.knnSearch(fullPoint, 1, &indices, &sqr_dists);
        outView->appendPoint(*fullView, i);
        if (sqr_dists[0] < m_tolerance)
            outView->setField(dim, i, 1);
        else
            outView->setField(dim, i, 0);
    }

    BufferReader bufferReader;
    bufferReader.addView(outView);

    Options writerOptions;
    Stage& writer = makeWriter(m_outputFile, bufferReader, "", writerOptions);
    writer.prepare(fullTable);
    writer.execute(fullTable);

    return 0;
}

} // namespace pdal
