/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include <pdal/pdal_test_main.hpp>

#include <FauxReader.hpp>
#include <LasReader.hpp>
#include <PlyReader.hpp>
#include <PlyWriter.hpp>
#include <pdal/StageFactory.hpp>
#include "Support.hpp"


namespace pdal
{

TEST(PlyWriter, constructor)
{
    PlyWriter writer1;

    StageFactory f;
    std::unique_ptr<Stage> writer2(f.createStage("writers.ply"));
    EXPECT_TRUE(writer2.get());
}


TEST(PlyWriter, write)
{
    Options readerOptions;
    readerOptions.add("count", 750);
    readerOptions.add("mode", "random");
    FauxReader reader;
    reader.setOptions(readerOptions);

    Options writerOptions;
    writerOptions.add("filename", Support::temppath("out.ply"));
    PlyWriter writer;
    writer.setOptions(writerOptions);
    writer.setInput(reader);

    PointTable table;
    writer.prepare(table);
    writer.execute(table);
}

void readWrite(const std::string storageMode)
{
    std::string input(Support::datapath("las/mvk-thin.las"));
    std::string output(Support::temppath("out.ply"));

    Options ro;
    ro.add("filename", input);

    LasReader reader;
    reader.setOptions(ro);

    Options wo;
    wo.add("filename", output);
    wo.add("storage_mode", storageMode);
    wo.add("precision", 15);

    PlyWriter writer;
    writer.setOptions(wo);
    writer.setInput(reader);

    FileUtils::deleteFile(output);
    PointTable table;
    writer.prepare(table);
    writer.execute(table);

    Options o1;
    o1.add("filename", input);

    LasReader r1;
    r1.setOptions(o1);

    PointTable t1;
    r1.prepare(t1);
    PointViewSet s1 = r1.execute(t1);

    Options o2;
    o2.add("filename", output);

    PlyReader r2;
    r2.setOptions(o2);

    PointTable t2;
    r2.prepare(t2);
    PointViewSet s2 = r2.execute(t2);

    EXPECT_EQ(s1.size(), 1u);
    EXPECT_EQ(s2.size(), 1u);
    PointViewPtr v1 = *s1.begin();
    PointViewPtr v2 = *s2.begin();
    EXPECT_EQ(v1->size(), 6280u);
    EXPECT_EQ(v1->size(), v2->size());
    for (PointId i = 0; i < v1->size(); i++)
    {
        EXPECT_DOUBLE_EQ(v1->getFieldAs<double>(Dimension::Id::X, i),
            v2->getFieldAs<double>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(v1->getFieldAs<double>(Dimension::Id::Y, i),
            v2->getFieldAs<double>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(v1->getFieldAs<double>(Dimension::Id::Z, i),
            v2->getFieldAs<double>(Dimension::Id::Z, i));
    }
}

TEST(PlyWriter, readWriteLittle)
{
    readWrite("little endian");
}

TEST(PlyWriter, readWriteBig)
{
    readWrite("big endian");
}

TEST(PlyWriter, readWriteAscii)
{
    readWrite("ascii");
}

TEST(PlyWriter, readWriteDefault)
{
    readWrite("default");
}

} // namespace pdal
