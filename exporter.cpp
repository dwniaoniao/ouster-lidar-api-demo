#include <fstream>
#include <iostream>

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>

#include "exporter.h"

extern "C"{

void write_point_cloud(xyz_t *xyz, size_t len)
{
    using namespace pdal;

    PointTable table;
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    PointViewPtr view(new PointView(table));
    for(size_t i = 0; i < len; i++){
        PointId id = view->size();
        view->setField(Dimension::Id::X, id, xyz[i].x);
        view->setField(Dimension::Id::Y, id, xyz[i].y);
        view->setField(Dimension::Id::Z, id, xyz[i].z);
    }
    BufferReader reader;
    reader.addView(view);
    StageFactory factory;
    Stage *writer = factory.createStage("writers.las");
    Options opts;
    opts.add("filename", "output.las");
    writer->setInput(reader);
    writer->setOptions(opts);
    writer->prepare(table);
    writer->execute(table);
}

}

