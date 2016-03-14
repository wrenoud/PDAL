/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "LowNoiseFilter.hpp"

#include <algorithm>

#include "PCLConversions.hpp"

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_macros.hpp>

#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.lownoise", "Statistical outlier removal",
               "http://pdal.io/stages/filters.lownoise.html");

CREATE_SHARED_PLUGIN(1, 0, LowNoiseFilter, Filter, s_info)

std::string LowNoiseFilter::getName() const
{
    return s_info.name;
}

Options LowNoiseFilter::getDefaultOptions()
{
    Options options;
    options.add("mean_k", 8, "Mean number of neighbors");
    options.add("multiplier", 2, "Standard deviation threshold");
    options.add("classify", true, "Apply classification labels?");
    options.add("extract", false, "Extract ground returns?");
    return options;
}

void LowNoiseFilter::processOptions(const Options& options)
{
    m_meanK = options.getValueOrDefault<int>("mean_k", 8);
    m_multiplier = options.getValueOrDefault<double>("multiplier", 2);
    m_classify = options.getValueOrDefault<bool>("classify", true);
    m_extract = options.getValueOrDefault<bool>("extract", false);
}

void LowNoiseFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Classification);
}

PointViewSet LowNoiseFilter::run(PointViewPtr input)
{
    bool logOutput = log()->getLevel() > LogLevel::Debug1;
    if (logOutput)
        log()->floatPrecision(8);
    log()->get(LogLevel::Debug2) << "Process LowNoiseFilter...\n";

    // convert PointView to PointXYZ
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
    Cloud::Ptr cloud(new Cloud);
    BOX3D bounds;
    input->calculateBounds(bounds);
    pclsupport::PDALtoPCD(input, *cloud, bounds);

    // PCL should provide console output at similar verbosity level as PDAL
    int level = log()->getLevel();
    switch (level)
    {
        case 0:
            pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
            break;
        case 1:
            pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
            break;
        case 2:
            pcl::console::setVerbosityLevel(pcl::console::L_WARN);
            break;
        case 3:
            pcl::console::setVerbosityLevel(pcl::console::L_INFO);
            break;
        case 4:
            pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
            break;
        default:
            pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
            break;
    }

    // Compute global mean, variance, and standard deviation for Z dimension
    double sum = 0.0, sumsqr = 0.0;
    for (auto const& p : cloud->points)
    {
      sum += p.z;
      sumsqr += p.z * p.z;
    }

    double mean = sum / cloud->size();
    double variance = (sumsqr - sum * sum / cloud->size()) / (cloud->size() - 1);
    double stdev = std::sqrt(variance);

    // Set threshold at mean minus one standard deviation - we will filter these
    // more aggressively as possible low noise points
    double thresh = mean - stdev;

    // pcl::PointIndicesPtr low_idx(new pcl::PointIndices);
    // low_idx->indices.reserve(cloud->size());
    std::set<int> inliers;//, outliers;
    // inliers.reserve(cloud->size());

    for (int i = 0; i < cloud->size(); ++i)
    {
      if (cloud->points[i].z >= thresh)
      //     outliers.insert(i);
      // else
          inliers.insert(i);
    }

    log()->get(LogLevel::Debug2) << inliers.size() << " points detected above threshold" << std::endl;

    // // Setup ExtractIndices filter with indices of possible low noise points
    // pcl::ExtractIndices<pcl::PointXYZ> ei;
    // ei.setInputCloud(cloud);
    // ei.setIndices(low_idx);
    //
    // // Extract below ground points from the input cloud
    // Cloud::Ptr cloud_f(new Cloud);
    // ei.filter(*cloud_f);

    // Setup the outlier filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);
    sor.setInputCloud(cloud);
    sor.setMeanK(m_meanK);
    sor.setStddevMulThresh(m_multiplier);
    sor.setNegative(true);

    // Execute statistical outlier removal filter
    pcl::PointCloud<pcl::PointXYZ> output;
    sor.filter(output);

    // Get the indices of the inliers below ground (i.e., noise-free points below ground)
    pcl::PointIndicesPtr inliers_sor(new pcl::PointIndices);
    inliers_sor->indices.reserve(cloud->size());
    sor.getRemovedIndices(*inliers_sor);

    log()->get(LogLevel::Debug2) << inliers_sor->indices.size() << " inliers" << std::endl;

    // // Get the indices of the outliers below ground (i.e., noise-free points below ground)
    // sor.setNegative(false);
    // pcl::PointIndicesPtr outliers_sor(new pcl::PointIndices);
    // outliers_sor->indices.reserve(cloud->size());
    // sor.getRemovedIndices(*outliers_sor);

    // // Setup the inliers PointIndicesPtr
    // pcl::PointIndicesPtr inliers(new pcl::PointIndices);
    // inliers->indices.reserve(cloud->size());
    //
    // std::sort(nonlow_idx.begin(), nonlow_idx.end());
    // std::sort(inliers_sor->indices.begin(), inliers_sor->indices.end());
    // std::set_union(nonlow_idx.begin(), nonlow_idx.end(), inliers_sor->indices.begin(), inliers_sor->indices.end(), std::back_inserter(inliers->indices));

    for (auto const& i : inliers_sor->indices)
    {
      inliers.insert(i);
    }

    log()->get(LogLevel::Debug2) << inliers.size() << " after adding points" << std::endl;

    // // Add the points that were not possible low noise points
    // for (auto const& i : nonlow_idx)
    // {
    //   inliers->indices.push_back(i);
    // }
    //
    // log()->get(LogLevel::Debug2) << inliers->indices.size() << " points added - those above the threshold" << std::endl;
    //
    // // Add the inliers from the possible low noise points
    // for (auto const& i : inliers_below->indices)
    // {
    //   log()->get(LogLevel::Debug) << i << " : " << low_idx->indices[i] << std::endl;
    //   inliers->indices.push_back(low_idx->indices[i]);
    // }
    //
    // log()->get(LogLevel::Debug2) << inliers->indices.size() << " with below ground inliers" << std::endl;

    PointViewSet viewSet;
    if (inliers.empty())
    {
        log()->get(LogLevel::Warning) << "Requested filter would remove all points. Try increasing the multiplier.\n";
        viewSet.insert(input);
        return viewSet;
    }

    std::vector<int> all(cloud->size());
    for (int i = 0; i < cloud->size(); ++i)
        all[i] = i;

    // inverse are the outliers
    // std::vector<int> outliers(input->size()-inliers.size());
    // for (PointId i = 0, j = 0, k = 0; i < input->size(); ++i)
    // {
    //     if (i == (PointId)inliers[j])
    //     {
    //         j++;
    //         continue;
    //     }
    //     outliers[k++] = i;
    // }

    std::vector<int> outliers;
    // std::sort(all.begin(), all.end());
    // std::sort(inliers.begin(), inliers.end());
    std::set_difference(all.begin(), all.end(),
                        inliers.begin(), inliers.end(),
                        std::back_inserter(outliers));

    if (!outliers.empty() && (m_classify || m_extract))
    {

        if (m_classify)
        {
            log()->get(LogLevel::Debug2) << "Labeled " << outliers.size() << " outliers as noise!\n";

            // set the classification label of outlier returns as 7
            // (corresponding to ASPRS LAS specification for low noise)
            for (const auto& i : outliers)
            {
                input->setField(Dimension::Id::Classification, i, 7);
            }

            viewSet.insert(input);
        }

        if (m_extract)
        {
            log()->get(LogLevel::Debug2) << "Extracted " << inliers.size() << " inliers!\n";

            // create new PointView containing only outliers
            PointViewPtr output = input->makeNew();
            log()->get(LogLevel::Debug) << "made\n";
            for (const auto& i : inliers)
            {
                output->appendPoint(*input, i);
            }
            log()->get(LogLevel::Debug) << "appended\n";

            viewSet.erase(input);
            log()->get(LogLevel::Debug) << "erased\n";
            viewSet.insert(output);
            log()->get(LogLevel::Debug) << "inserted\n";
        }
    }
    else
    {
        if (outliers.empty())
            log()->get(LogLevel::Warning) << "Filtered cloud has no outliers!\n";

        if (!(m_classify || m_extract))
            log()->get(LogLevel::Warning) << "Must choose --classify or --extract\n";

        // return the input buffer unchanged
        viewSet.insert(input);
    }
    log()->get(LogLevel::Debug) << "foo\n";

    return viewSet;
}

} // namespace pdal
