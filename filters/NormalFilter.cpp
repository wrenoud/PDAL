/******************************************************************************
 * Copyright (c) 2016-2017, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "NormalFilter.hpp"

#include <pdal/EigenUtils.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.normal", "Normal Filter",
               "http://pdal.io/stages/filters.normal.html");

CREATE_STATIC_PLUGIN(1, 0, NormalFilter, Filter, s_info)

std::string NormalFilter::getName() const
{
    return s_info.name;
}

void NormalFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k-Nearest Neighbors", m_knn, 8);
    m_vxArg = &args.add("vx", "Viewpoint X coordinate", m_vx);
    m_vyArg = &args.add("vy", "Viewpoint Y coordinate", m_vy);
    m_vzArg = &args.add("vz", "Viewpoint Z coordinate", m_vz);
}

void NormalFilter::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;

    layout->registerDims(
        {Id::NormalX, Id::NormalY, Id::NormalZ, Id::Curvature});
}

void NormalFilter::doFilter(PointView& view)
{
    m_knn = 8;
    filter(view);
}

void NormalFilter::filter(PointView& view)
{
    KD3Index& kdi = view.build3dIndex();

    BOX3D bounds;
    view.calculateBounds(bounds);

    double vx =
        m_vxArg->set() ? m_vx : bounds.minx + (bounds.maxx - bounds.minx) * 0.5;
    double vy =
        m_vyArg->set() ? m_vy : bounds.miny + (bounds.maxy - bounds.miny) * 0.5;
    double vz = m_vzArg->set() ? m_vz : bounds.maxz + 1000;

    for (PointId i = 0; i < view.size(); ++i)
    {
        // find the k-nearest neighbors
        auto ids = kdi.neighbors(i, m_knn);

        // compute covariance of the neighborhood
        auto B = eigen::computeCovariance(view, ids);

        // perform the eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(B);
        if (solver.info() != Eigen::Success)
            throwError("Cannot perform eigen decomposition.");
        auto eval = solver.eigenvalues();
        Eigen::Vector3f normal = solver.eigenvectors().col(0);

        // flip normal towards viewpoint
        PointRef p = view.point(i);
        Eigen::Vector3f vp(vx - p.getFieldAs<double>(Dimension::Id::X),
                           vy - p.getFieldAs<double>(Dimension::Id::Y),
                           vz - p.getFieldAs<double>(Dimension::Id::Z));
        if (vp.dot(normal) < 0)
            normal *= -1.0;

        view.setField(Dimension::Id::NormalX, i, normal[0]);
        view.setField(Dimension::Id::NormalY, i, normal[1]);
        view.setField(Dimension::Id::NormalZ, i, normal[2]);

        double sum = eval[0] + eval[1] + eval[2];
        view.setField(Dimension::Id::Curvature, i,
                      sum ? std::fabs(eval[0] / sum) : 0);
    }
}

} // namespace pdal
