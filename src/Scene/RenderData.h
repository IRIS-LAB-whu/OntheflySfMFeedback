// RenderData.h
#ifndef RENDERDATA_H
#define RENDERDATA_H

#include <array>
#include <vector>

using PointPainterData = std::array<float, 7>; // x,y,z,r,g,b,a
using LinePainterData = std::array<float, 14>;
using TrianglePainterData = std::array<float, 21>;

struct RenderData {
	std::vector<TrianglePainterData>    mesh_data;
	std::vector<LinePainterData>        mesh_lines;
	std::vector<TrianglePainterData>    quality_data;
	std::vector<PointPainterData>       guide_points;
	std::vector<LinePainterData>        guide_lines;
	std::vector<LinePainterData>        guide_directions;
	std::vector<PointPainterData>       heatmap_points;
	std::vector<TrianglePainterData>    base_faces;
};

#endif // RENDERDATA_H