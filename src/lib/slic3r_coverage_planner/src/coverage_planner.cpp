//
// Created by Clemens Elflein on 27.08.21.
//

#include "ros/ros.h"

#include <boost/range/adaptor/reversed.hpp>
#include <optional>

#include "ExPolygon.hpp"
#include "Polyline.hpp"
#include "Fill/FillRectilinear.hpp"
#include "Fill/FillConcentric.hpp"


#include "slic3r_coverage_planner/PlanPath.h"
#include "visualization_msgs/MarkerArray.h"
#include "Surface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <Fill/FillPlanePath.hpp>
#include <PerimeterGenerator.hpp>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ClipperUtils.hpp"
#include "ExtrusionEntityCollection.hpp"


bool visualize_plan;
// Fill the area cell by cell, each in one zigzag, and mow the cells and the rounds around obstacles in the order with
// the least driving in between, instead of the slic3r fill order
bool cell_fill;
ros::Publisher marker_array_publisher;


void
createMarkers(const slic3r_coverage_planner::PlanPathRequest &planning_request,
              const slic3r_coverage_planner::PlanPathResponse &planning_result,
              visualization_msgs::MarkerArray &markerArray) {

    std::vector<std_msgs::ColorRGBA> colors;

    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }
    {
        std_msgs::ColorRGBA color;
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        color.a = 1.0;
        colors.push_back(color);
    }

    // Create markers for the input polygon
    {
        // Walk through the paths we send to the navigation stack
        auto &path = planning_request.outline.points;
        // Each group gets a single line strip as marker
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.ns = "mower_map_service_lines";
        marker.id = static_cast<int>(markerArray.markers.size());
        marker.frame_locked = true;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.color = colors[0];
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;

        // Add the points to the line strip
        for (auto &point: path) {

            geometry_msgs::Point vpt;
            vpt.x = point.x;
            vpt.y = point.y;
            marker.points.push_back(vpt);
        }
        markerArray.markers.push_back(marker);

        // Create markers for start and end
        if (!path.empty()) {
            visualization_msgs::Marker marker{};

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.color = colors[0];
            marker.pose.position.x = path.front().x;
            marker.pose.position.y = path.front().y;
            marker.scale.x = 0.1;
            marker.scale.y = marker.scale.z = 0.1;
            markerArray.markers.push_back(marker);
        }
    }


    // keep track of the color used last, so that we can use a new one for each path
    uint32_t cidx = 0;

    // Walk through the paths we send to the navigation stack
    for (auto &path: planning_result.paths) {
        // Each group gets a single line strip as marker
        visualization_msgs::Marker marker;

        marker.header.frame_id = "map";
        marker.ns = "mower_map_service_lines";
        marker.id = static_cast<int>(markerArray.markers.size());
        marker.frame_locked = true;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.color = colors[cidx];
        marker.pose.orientation.w = 1;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;

        // Add the points to the line strip
        for (auto &point: path.path.poses) {

            geometry_msgs::Point vpt;
            vpt.x = point.pose.position.x;
            vpt.y = point.pose.position.y;
            marker.points.push_back(vpt);
        }
        markerArray.markers.push_back(marker);

        // Create markers for start
        if (!path.path.poses.empty()) {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = static_cast<int>(markerArray.markers.size());
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.color = colors[cidx];
            marker.pose = path.path.poses.front().pose;
            marker.scale.x = 0.2;
            marker.scale.y = marker.scale.z = 0.05;
            markerArray.markers.push_back(marker);
        }

        // New color for a new path
        cidx = (cidx + 1) % colors.size();
    }
}

void traverse_from_left(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: contours) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse_from_left(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}

void traverse_from_right(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: boost::adaptors::reverse(contours)) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse_from_right(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}

slic3r_coverage_planner::Path determinePathForOutline(std_msgs::Header &header, Slic3r::Polygon &outline_poly, Slic3r::Polygons &group, bool isObstacle, Point *areaLastPoint) {
    slic3r_coverage_planner::Path path;
    path.is_outline = true;
    path.path.header = header;

    Point lastPoint;
    bool is_first_point = true;
    for (int i = 0; i < group.size(); i++) {
        auto points = group[i].equally_spaced_points(scale_(0.1));
        if (points.size() < 2) {
            ROS_INFO("Skipping single dot");
            continue;
        }
        ROS_INFO_STREAM("Got " << points.size() << " points");

        if (!is_first_point) {
            // Find a good transition point between the loops.
            // It should be close to the last split point, so that we don't need to traverse a lot.

            // Find the point in the current poly which is closest to the last point of the last group
            // (which is the next inner poly from this point of view).
            const auto last_x = unscale(lastPoint.x);
            const auto last_y = unscale(lastPoint.y);
            double min_distance = INFINITY;
            int closest_idx = 0;
            for (int idx = 0; idx < points.size(); ++idx) {
                const auto &pt = points[idx];
                const auto pt_x = unscale(pt.x);
                const auto pt_y = unscale(pt.y);
                double distance = sqrt((pt_x - last_x) * (pt_x - last_x) + (pt_y - last_y) * (pt_y - last_y));
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_idx = idx;
                }
            }

            // In order to smooth the transition we skip some points (think spiral movement of the mower).
            // Check, that the skip did not break the path (cross the outer poly during transition).
            // If it's fine, use the smoothed path, otherwise use the shortest point to split.
            int smooth_transition_idx = (closest_idx + 3) % points.size();

            const Polygon *next_outer_poly;
            if (i < group.size() - 1) {
                next_outer_poly = &group[i + 1];
            } else {
                // we are in the outermost line, use outline for collision check
                next_outer_poly = &outline_poly;
            }
            Line connection(points[smooth_transition_idx], lastPoint);
            Point intersection_pt{};
            if (next_outer_poly->intersection(connection, &intersection_pt)) {
                // intersection, we need to transition at closest point
                smooth_transition_idx = closest_idx;
            }

            if (smooth_transition_idx > 0) {
                std::rotate(points.begin(), points.begin() + smooth_transition_idx, points.end());
            }
        }

        for (auto &pt: points) {
            if (is_first_point) {
                lastPoint = pt;
                is_first_point = false;
                continue;
            }

            // calculate pose for "lastPoint" pointing to current point

            // Direction for obstacle needs to be inversed compared to area outline, because we will reverse the point order later.
            auto dir = isObstacle ? lastPoint - pt : pt - lastPoint;

            double orientation = atan2(dir.y, dir.x);
            tf2::Quaternion q(0.0, 0.0, orientation);

            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = tf2::toMsg(q);
            pose.pose.position.x = unscale(lastPoint.x);
            pose.pose.position.y = unscale(lastPoint.y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);
            lastPoint = pt;
        }
    }

    if (is_first_point) {
        // there wasn't any usable point, so return the empty path
        return path;
    }

    // finally, we add the final pose for "lastPoint" with the same orientation as the last pose
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose.orientation = path.path.poses.back().pose.orientation;
    pose.pose.position.x = unscale(lastPoint.x);
    pose.pose.position.y = unscale(lastPoint.y);
    pose.pose.position.z = 0;
    path.path.poses.push_back(pose);

    if (areaLastPoint != nullptr) {
        *areaLastPoint = lastPoint;
    }

    return path;
}

// A lane of a cell, in the frame rotated so lanes run along x
struct CellLane {
    coord_t x0, x1, y;
    int row;
};

// A part of the area the lanes cross in one piece, mowed in one zigzag. Or a part too narrow for lanes, mowed
// along its middle (path)
struct Cell {
    std::vector<CellLane> lanes;
    Polyline path;
};

// Lanes along the angle, d apart, in cells. A cell goes on as long as each lane leads to exactly one in the next row
// and both ends connect inside the area. Where the area splits (an obstacle, a bay) the part that overlaps most goes
// on and the rest starts a cell of its own. Where parts join again all of them end, so the parts beside an obstacle
// can be mowed one after the other before going on below it.
std::vector<Cell> buildCells(const Polygons &inner, double angle, coord_t d) {
    std::vector<Cell> cells;
    for (const auto &region: union_ex(inner)) {
        ExPolygon r = region;
        r.rotate(-angle);
        // connectors between two lanes may use the half lane next to the innermost round
        const ExPolygons connector_area = offset_ex(ExPolygons{r}, float(d) / 4);
        const auto inside = [&](const Point &p, const Point &q) {
            for (const auto &ca: connector_area) {
                if (ca.contains(Line(p, q))) return true;
            }
            return false;
        };
        const auto overlaps = [](const CellLane &p, const CellLane &q) { return p.x0 < q.x1 && q.x0 < p.x1; };
        // the lane centres keep half a lane from the innermost round, like the slic3r fill
        const ExPolygons centres = offset_ex(ExPolygons{r}, -float(d) / 2);
        // lanes only where it's at least a lane wide, a narrower part (between rounds, a corridor) is mowed along its
        // middle. Bits under 30 cm are corners the lanes reach anyway
        const ExPolygons wide = offset_ex(offset_ex(centres, -float(d) / 2), float(d) / 2);
        for (const auto &narrow: offset_ex(diff_ex(centres, wide), -float(scale_(0.01)))) {
            Polylines middle;
            offset_ex(ExPolygons{narrow}, float(scale_(0.01))).front().medial_axis(2.0 * d, 0, &middle);
            for (auto &line: middle) {
                line.simplify(scale_(0.02));
                if (line.length() >= scale_(0.3)) cells.push_back({{}, line});
            }
        }
        for (const auto &lanes_area: wide) {
            // A long connector is fine along the edge (a slanted edge), not through the middle: there it would cut
            // across the lanes, that's where the edge has a step and a new cell starts
            const ExPolygons edge = diff_ex(offset_ex(ExPolygons{lanes_area}, float(d) / 4),
                                            offset_ex(ExPolygons{lanes_area}, -float(d)));
            const auto connects = [&](const Point &p, const Point &q) {
                if (!inside(p, q)) return false;
                if (p.distance_to(q) <= 2 * d) return true;
                for (const auto &e: edge) {
                    if (e.contains(Line(p, q))) return true;
                }
                return false;
            };
            const BoundingBox bb = lanes_area.bounding_box();
            const coord_t h = bb.max.y - bb.min.y;
            const int rows = h / d + 1;
            // the room that's left over goes to both sides equally
            const coord_t y0 = bb.min.y + (h - coord_t(rows - 1) * d) / 2;
            std::vector<int> open;
            std::vector<std::vector<CellLane>> all_rows;
            const size_t first_cell = cells.size();
            for (int k = 0; k < rows; k++) {
                const coord_t y = y0 + coord_t(k) * d;
                Polyline line;
                line.points = {Point(bb.min.x - d, y), Point(bb.max.x + d, y)};
                std::vector<CellLane> row;
                for (const auto &c: intersection_pl(Polylines{line}, (Polygons) lanes_area)) {
                    coord_t x0 = c.first_point().x, x1 = c.last_point().x;
                    if (x0 > x1) std::swap(x0, x1);
                    if (x1 - x0 > scale_(0.02)) row.push_back({x0, x1, y, k});
                }
                std::sort(row.begin(), row.end(), [](const CellLane &p, const CellLane &q) { return p.x0 < q.x0; });

                std::vector<std::tuple<coord_t, int, int>> pairs;  // overlap, cell, lane
                for (int c: open) {
                    const CellLane &last = cells[c].lanes.back();
                    for (size_t j = 0; j < row.size(); j++) {
                        if (!overlaps(last, row[j])) continue;
                        int joining = 0;
                        for (int other: open) joining += overlaps(cells[other].lanes.back(), row[j]);
                        if (joining > 1) continue;
                        if (!connects(Point(last.x0, last.y), Point(row[j].x0, row[j].y)) ||
                            !connects(Point(last.x1, last.y), Point(row[j].x1, row[j].y))) continue;
                        pairs.emplace_back(std::min(last.x1, row[j].x1) - std::max(last.x0, row[j].x0), c, int(j));
                    }
                }
                std::sort(pairs.begin(), pairs.end(),
                          [](const auto &p, const auto &q) { return std::get<0>(p) > std::get<0>(q); });
                std::vector<int> cell_of(row.size(), -1);
                std::vector<int> taken;
                for (const auto &[overlap, c, j]: pairs) {
                    if (cell_of[j] >= 0 || std::find(taken.begin(), taken.end(), c) != taken.end()) continue;
                    cell_of[j] = c;
                    taken.push_back(c);
                }
                std::vector<int> next;
                for (size_t j = 0; j < row.size(); j++) {
                    if (cell_of[j] >= 0) {
                        cells[cell_of[j]].lanes.push_back(row[j]);
                    } else {
                        cells.push_back({{row[j]}});
                        cell_of[j] = int(cells.size()) - 1;
                    }
                    next.push_back(cell_of[j]);
                }
                open = next;
                all_rows.push_back(row);
            }

            // The rows are a fixed grid, an edge along the lanes can leave up to half a lane between the round and the
            // first or last lane of a cell. Where such a strip is long (at a slanted edge the lane ends and
            // connectors cover it), an extra lane half a row further out mows just that stretch
            const auto extra = [&](const CellLane &lane, int dir) -> std::optional<CellLane> {
                std::vector<std::pair<coord_t, coord_t>> strip;
                Polyline probe;
                probe.points = {Point(lane.x0 - d, lane.y + dir * d * 3 / 4), Point(lane.x1 + d, lane.y + dir * d * 3 / 4)};
                for (const auto &c: intersection_pl(Polylines{probe}, (Polygons) lanes_area)) {
                    coord_t x0 = c.first_point().x, x1 = c.last_point().x;
                    if (x0 > x1) std::swap(x0, x1);
                    strip.push_back({x0, x1});
                }
                // the next row covers its part anyway
                const int next_row = lane.row + dir;
                if (lane.row >= 0 && next_row >= 0 && next_row < int(all_rows.size())) {
                    for (const auto &other: all_rows[next_row]) {
                        std::vector<std::pair<coord_t, coord_t>> rest;
                        for (const auto &[x0, x1]: strip) {
                            if (other.x1 <= x0 || other.x0 >= x1) {
                                rest.push_back({x0, x1});
                                continue;
                            }
                            if (other.x0 > x0) rest.push_back({x0, other.x0});
                            if (other.x1 < x1) rest.push_back({other.x1, x1});
                        }
                        strip = rest;
                    }
                }
                coord_t length = 0, from = 0, to = 0;
                for (const auto &[x0, x1]: strip) {
                    if (length == 0 || x0 < from) from = x0;
                    if (length == 0 || x1 > to) to = x1;
                    length += x1 - x0;
                }
                if (length < std::max<coord_t>(scale_(0.5), (lane.x1 - lane.x0) * 3 / 10)) return std::nullopt;
                const coord_t y = lane.y + dir * d / 2;
                Polyline line;
                line.points = {Point(from - d / 2, y), Point(to + d / 2, y)};
                std::optional<CellLane> best;
                for (const auto &c: intersection_pl(Polylines{line}, (Polygons) lanes_area)) {
                    coord_t x0 = c.first_point().x, x1 = c.last_point().x;
                    if (x0 > x1) std::swap(x0, x1);
                    if (x1 - x0 > scale_(0.02) && (!best || x1 - x0 > best->x1 - best->x0)) best = CellLane{x0, x1, y, -1};
                }
                return best;
            };
            // cells of their own are added after the loop, adding to cells here would move the lanes being changed
            std::vector<Cell> single;
            const size_t last_cell = cells.size();
            for (size_t c = first_cell; c < last_cell; c++) {
                auto &lanes = cells[c].lanes;
                if (auto lane = extra(lanes.front(), -1)) {
                    if (inside(Point(lane->x0, lane->y), Point(lanes.front().x0, lanes.front().y)) &&
                        inside(Point(lane->x1, lane->y), Point(lanes.front().x1, lanes.front().y))) {
                        lanes.insert(lanes.begin(), *lane);
                    } else {
                        single.push_back({{*lane}});
                    }
                }
                if (auto lane = extra(lanes.back(), 1)) {
                    if (inside(Point(lanes.back().x0, lanes.back().y), Point(lane->x0, lane->y)) &&
                        inside(Point(lanes.back().x1, lanes.back().y), Point(lane->x1, lane->y))) {
                        lanes.push_back(*lane);
                    } else {
                        single.push_back({{*lane}});
                    }
                }
            }
            cells.insert(cells.end(), single.begin(), single.end());
        }
    }
    return cells;
}

// Poses every 10 cm along the line, each pointing to the next
slic3r_coverage_planner::Path linePath(std_msgs::Header &header, Polyline line, bool is_outline) {
    slic3r_coverage_planner::Path path;
    path.is_outline = is_outline;
    path.path.header = header;
    line.remove_duplicate_points();
    auto points = line.equally_spaced_points(scale_(0.1));
    if (points.size() < 2) return path;
    for (size_t k = 0; k < points.size(); k++) {
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        if (k + 1 < points.size()) {
            const auto dir = points[k + 1] - points[k];
            tf2::Quaternion q(0.0, 0.0, atan2(dir.y, dir.x));
            pose.pose.orientation = tf2::toMsg(q);
        } else {
            pose.pose.orientation = path.path.poses.back().pose.orientation;
        }
        pose.pose.position.x = unscale(points[k].x);
        pose.pose.position.y = unscale(points[k].y);
        pose.pose.position.z = 0;
        path.path.poses.push_back(pose);
    }
    return path;
}

// Appends the rounds around obstacles and the cells in the order with the least driving in between, starting where
// the paths so far end. A cell may start in any of its four corners, the rounds keep their direction
void appendCells(slic3r_coverage_planner::PlanPathResponse &res, std_msgs::Header &header, Polygon &outline_poly,
                 std::vector<Polygons> &obstacle_outlines, const std::vector<Cell> &cells, double angle) {
    std::vector<std::vector<slic3r_coverage_planner::Path>> items;
    for (auto &group: obstacle_outlines) {
        auto path = determinePathForOutline(header, outline_poly, group, true, nullptr);
        if (path.path.poses.empty()) continue;
        std::reverse(path.path.poses.begin(), path.path.poses.end());
        items.push_back({path});
    }
    for (const auto &cell: cells) {
        std::vector<slic3r_coverage_planner::Path> variants;
        if (!cell.path.points.empty()) {
            Polyline line = cell.path;
            line.rotate(angle);
            variants.push_back(linePath(header, line, false));
            line.reverse();
            variants.push_back(linePath(header, line, false));
            if (!variants.front().path.poses.empty()) items.push_back(variants);
            continue;
        }
        for (int left = 0; left < 2; left++) {
            Polyline zigzag;
            for (size_t k = 0; k < cell.lanes.size(); k++) {
                const CellLane &lane = cell.lanes[k];
                const bool right = (k % 2 == 0) == (left == 1);
                zigzag.points.push_back(Point(right ? lane.x0 : lane.x1, lane.y));
                zigzag.points.push_back(Point(right ? lane.x1 : lane.x0, lane.y));
            }
            zigzag.rotate(angle);
            auto path = linePath(header, zigzag, false);
            if (path.path.poses.empty()) continue;
            variants.push_back(path);
            zigzag.reverse();
            variants.push_back(linePath(header, zigzag, false));
        }
        if (!variants.empty()) items.push_back(variants);
    }
    if (items.empty()) return;

    const auto dist = [](const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) {
        return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
    };
    // without outer rounds the first item can start anywhere
    const bool free_start = res.paths.empty();
    const geometry_msgs::PoseStamped start =
            free_start ? items.front().front().path.poses.front() : res.paths.back().path.poses.back();
    const auto cost = [&](const std::vector<std::pair<int, int>> &order) {
        geometry_msgs::PoseStamped pos = start;
        double c = 0;
        for (size_t n = 0; n < order.size(); n++) {
            const auto &[i, v] = order[n];
            if (n > 0 || !free_start) c += dist(pos, items[i][v].path.poses.front());
            pos = items[i][v].path.poses.back();
        }
        return c;
    };

    // nearest first, then move single items to another place or corner as long as that saves driving
    std::vector<std::pair<int, int>> order;
    std::vector<bool> used(items.size(), false);
    geometry_msgs::PoseStamped pos = start;
    for (size_t n = 0; n < items.size(); n++) {
        int bi = -1, bv = 0;
        double bd = 0;
        for (size_t i = 0; i < items.size(); i++) {
            if (used[i]) continue;
            for (size_t v = 0; v < items[i].size(); v++) {
                const double d = dist(pos, items[i][v].path.poses.front());
                if (bi < 0 || d < bd) {
                    bi = i;
                    bv = v;
                    bd = d;
                }
            }
        }
        used[bi] = true;
        order.push_back({bi, bv});
        pos = items[bi][bv].path.poses.back();
    }
    for (bool improved = true; improved;) {
        improved = false;
        const double current = cost(order);
        for (size_t a = 0; a < order.size() && !improved; a++) {
            auto rest = order;
            const int item = rest[a].first;
            rest.erase(rest.begin() + a);
            for (size_t b = 0; b <= rest.size() && !improved; b++) {
                for (size_t v = 0; v < items[item].size() && !improved; v++) {
                    auto candidate = rest;
                    candidate.insert(candidate.begin() + b, {item, int(v)});
                    if (cost(candidate) < current - 1e-6) {
                        order = candidate;
                        improved = true;
                    }
                }
            }
        }
    }
    for (const auto &[i, v]: order) res.paths.push_back(items[i][v]);
}

bool planPath(slic3r_coverage_planner::PlanPathRequest &req, slic3r_coverage_planner::PlanPathResponse &res) {

    Slic3r::Polygon outline_poly;
    for (auto &pt: req.outline.points) {
        outline_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
    }

    outline_poly.make_counter_clockwise();

    // This ExPolygon contains our input area with holes.
    Slic3r::ExPolygon expoly(outline_poly);

    for (auto &hole: req.holes) {
        Slic3r::Polygon hole_poly;
        for (auto &pt: hole.points) {
            hole_poly.points.push_back(Point(scale_(pt.x), scale_(pt.y)));
        }
        hole_poly.make_clockwise();

        // Clip the hole to the outline instead of using it as-is. An obstacle that only
        // partially overlaps the area (or extends beyond it) can otherwise own the extreme
        // vertex of the whole path set, which makes Clipper's offset engine (ClipperOffset::
        // FixOrientations) flip the orientation of every path, including the outline itself.
        // That makes the planner fill inside the obstacle instead of inside the area.
        Polygons clipped_holes = intersection(outline_poly, hole_poly);
        for (auto &clipped_hole: clipped_holes) {
            clipped_hole.make_clockwise();
            expoly.holes.push_back(clipped_hole);
        }
    }





    // Results are stored here
    std::vector<Polygons> area_outlines;
    Polylines fill_lines;
    std::vector<Polygons> obstacle_outlines;


    coord_t distance = scale_(req.distance);
    coord_t outer_distance = scale_(req.outer_offset);

    // detect how many perimeters must be generated for this island
    int loops = req.outline_count;

    ROS_INFO_STREAM("generating " << loops << " outlines");

    const int loop_number = loops - 1;  // 0-indexed loops
    const int inner_loop_number = loop_number - req.outline_overlap_count;


    Polygons gaps;

    Polygons last = expoly;
    Polygons inner = last;
    if (loop_number >= 0) {  // no loops = -1

        std::vector<PerimeterGeneratorLoops> contours(loop_number + 1);    // depth => loops
        std::vector<PerimeterGeneratorLoops> holes(loop_number + 1);       // depth => loops

        for (int i = 0; i <= loop_number; ++i) {  // outer loop is 0
            Polygons offsets;

            if (i == 0) {
                offsets = offset(
                        last,
                        -outer_distance
                );
            } else {
                offsets = offset(
                        last,
                        -distance
                );
            }

            if (offsets.empty()) break;


            last = offsets;
            if (i <= inner_loop_number) {
                inner = last;
            }

            for (Polygons::const_iterator polygon = offsets.begin(); polygon != offsets.end(); ++polygon) {
                PerimeterGeneratorLoop loop(*polygon, i);
                loop.is_contour = polygon->is_counter_clockwise();
                if (loop.is_contour) {
                    contours[i].push_back(loop);
                } else {
                    holes[i].push_back(loop);
                }
            }
        }

        // nest loops: holes first
        for (int d = 0; d <= loop_number; ++d) {
            PerimeterGeneratorLoops &holes_d = holes[d];

            // loop through all holes having depth == d
            for (int i = 0; i < (int) holes_d.size(); ++i) {
                const PerimeterGeneratorLoop &loop = holes_d[i];

                // find the hole loop that contains this one, if any
                for (int t = d + 1; t <= loop_number; ++t) {
                    for (int j = 0; j < (int) holes[t].size(); ++j) {
                        PerimeterGeneratorLoop &candidate_parent = holes[t][j];
                        if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                            candidate_parent.children.push_back(loop);
                            holes_d.erase(holes_d.begin() + i);
                            --i;
                            goto NEXT_LOOP;
                        }
                    }
                }

                NEXT_LOOP:;
            }
        }

        // nest contour loops
        for (int d = loop_number; d >= 1; --d) {
            PerimeterGeneratorLoops &contours_d = contours[d];

            // loop through all contours having depth == d
            for (int i = 0; i < (int) contours_d.size(); ++i) {
                const PerimeterGeneratorLoop &loop = contours_d[i];

                // find the contour loop that contains it
                for (int t = d - 1; t >= 0; --t) {
                    for (size_t j = 0; j < contours[t].size(); ++j) {
                        PerimeterGeneratorLoop &candidate_parent = contours[t][j];
                        if (candidate_parent.polygon.contains(loop.polygon.first_point())) {
                            candidate_parent.children.push_back(loop);
                            contours_d.erase(contours_d.begin() + i);
                            --i;
                            goto NEXT_CONTOUR;
                        }
                    }
                }

                NEXT_CONTOUR:;
            }
        }

        if (!req.skip_area_outline) {
            traverse_from_right(contours[0], area_outlines);
        }

        if (!req.skip_obstacle_outlines) {
            for (auto &hole: holes) {
                traverse_from_left(hole, obstacle_outlines);
            }
            for (auto &obstacle_group: obstacle_outlines) {
                for (auto &poly: obstacle_group) {
                    std::reverse(poly.points.begin(), poly.points.end());
                }
            }
        }
    }


    std::vector<Cell> cells;
    if (!req.skip_fill && cell_fill) {
        cells = buildCells(inner, req.angle, scale_(req.distance));
    } else if (!req.skip_fill) {
        ExPolygons expp = union_ex(inner);


        // Go through the innermost poly and create the fill path using a Fill object
        for (auto &poly: expp) {
            Slic3r::Surface surface(Slic3r::SurfaceType::stBottom, poly);

            Slic3r::Fill *fill;
            if (req.fill_type == slic3r_coverage_planner::PlanPathRequest::FILL_LINEAR) {
                fill = new Slic3r::FillRectilinear();
            } else {
                fill = new Slic3r::FillConcentric();
            }
            fill->link_max_length = scale_(1.0);
            fill->angle = req.angle;
            fill->z = scale_(1.0);
            fill->endpoints_overlap = 0;
            fill->density = 1.0;
            fill->dont_connect = false;
            fill->dont_adjust = true;
            fill->min_spacing = req.distance;
            fill->complete = false;
            fill->link_max_length = 0;

            ROS_INFO_STREAM("Starting Fill. Poly size:" << surface.expolygon.contour.points.size());

            Slic3r::Polylines lines = fill->fill_surface(surface);
            append_to(fill_lines, lines);
            delete fill;
            fill = nullptr;

            ROS_INFO_STREAM("Fill Complete. Polyline count: " << lines.size());
            for (int i = 0; i < lines.size(); i++) {
                ROS_INFO_STREAM("Polyline " << i << " has point count: " << lines[i].points.size());
            }
        }
    }


    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    header.seq = 0;

    /**
     * Some postprocessing is done here. Until now we just have polygons (just points), but the ROS
     * navigation stack requires an orientation for each of those points as well.
     *
     * In order to achieve this, we split the polygon at some point to make it into a line with start and end.
     * Then we can calculate the orientation at each point by looking at the connection line between two points.
     */

    Point areaLastPoint;
    for (auto &group: area_outlines) {
        auto path = determinePathForOutline(header, outline_poly, group, false, &areaLastPoint);
        if (!path.path.poses.empty()) {
            res.paths.push_back(path);
        }
    }

    if (cell_fill) {
        appendCells(res, header, outline_poly, obstacle_outlines, cells, req.angle);
        // already in, the slic3r order below has nothing left to do
        obstacle_outlines.clear();
    }

    // The order for 3d printing seems to be to sweep across the X and then up the Y axis
    // which is very inefficient for a mower. Order the holes by distance to the previous end-point instead.
    std::vector<Slic3r::Polygons> ordered_obstacle_outlines;
    if (obstacle_outlines.size() > 0) {
        // If no prev point set to the first point in first obstacle
        // Note: back() polygon is the first (outer) loop
        auto prev_point = area_outlines.size() > 0 ? &areaLastPoint :
            &obstacle_outlines.front().back().points.front();

        while (obstacle_outlines.size()) {
            // Sort be desc distance then pop closest outline from the back of the vector
            std::sort(obstacle_outlines.begin(), obstacle_outlines.end(),
                      [prev_point](Slic3r::Polygons &a, Slic3r::Polygons &b) {
                          // Note: back() polygon is the first (outer) loop
                          auto a_firstPoint = a.back().points.front();
                          double distance_a = sqrt(
                                  (a_firstPoint.x - prev_point->x) * (a_firstPoint.x - prev_point->x) +
                                  (a_firstPoint.y - prev_point->y) * (a_firstPoint.y - prev_point->y)
                          );
                          auto b_firstPoint = b.back().points.front();
                          double distance_b = sqrt(
                                  (b_firstPoint.x - prev_point->x) * (b_firstPoint.x - prev_point->x) +
                                  (b_firstPoint.y - prev_point->y) * (b_firstPoint.y - prev_point->y)
                          );
                          return distance_a >= distance_b;
                      });
            ordered_obstacle_outlines.push_back(obstacle_outlines.back());
            obstacle_outlines.pop_back();
            // Note: front() polygon is the last (inner) loop
            prev_point = &ordered_obstacle_outlines.back().front().points.back();
        }
    }

    // At this point, the obstacles outlines are still "the wrong way" (i.e. inner first, then outer ...),
    // this is intentional, because then it's easier to find good traversal points.
    // In order to make the mower approach the obstacle, we will reverse the path later.
    for (auto &group: ordered_obstacle_outlines) {
        // Reverse here to make the mower approach the obstacle instead of starting close to the obstacle
        auto path = determinePathForOutline(header, outline_poly, group, true, nullptr);
        if (!path.path.poses.empty()) {
            std::reverse(path.path.poses.begin(), path.path.poses.end());
            res.paths.push_back(path);
        }
    }

    if (!req.skip_fill) {
        for (int i = 0; i < fill_lines.size(); i++) {
            auto &line = fill_lines[i];
            slic3r_coverage_planner::Path path;
            path.is_outline = false;
            path.path.header = header;

            line.remove_duplicate_points();


            auto equally_spaced_points = line.equally_spaced_points(scale_(0.1));
            if (equally_spaced_points.size() < 2) {
                ROS_INFO("Skipping single dot");
                continue;
            }
            ROS_INFO_STREAM("Got " << equally_spaced_points.size() << " points");

            Point *lastPoint = nullptr;
            for (auto &pt: equally_spaced_points) {
                if (lastPoint == nullptr) {
                    lastPoint = &pt;
                    continue;
                }

                // calculate pose for "lastPoint" pointing to current point

                auto dir = pt - *lastPoint;
                double orientation = atan2(dir.y, dir.x);
                tf2::Quaternion q(0.0, 0.0, orientation);

                geometry_msgs::PoseStamped pose;
                pose.header = header;
                pose.pose.orientation = tf2::toMsg(q);
                pose.pose.position.x = unscale(lastPoint->x);
                pose.pose.position.y = unscale(lastPoint->y);
                pose.pose.position.z = 0;
                path.path.poses.push_back(pose);
                lastPoint = &pt;
            }

            // finally, we add the final pose for "lastPoint" with the same orientation as the last pose
            geometry_msgs::PoseStamped pose;
            pose.header = header;
            pose.pose.orientation = path.path.poses.back().pose.orientation;
            pose.pose.position.x = unscale(lastPoint->x);
            pose.pose.position.y = unscale(lastPoint->y);
            pose.pose.position.z = 0;
            path.path.poses.push_back(pose);

            res.paths.push_back(path);
        }
    }

    if (visualize_plan) {
        visualization_msgs::MarkerArray arr;
        {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.ns = "mower_map_service_lines";
            marker.id = -1;
            marker.frame_locked = true;
            marker.action = visualization_msgs::Marker::DELETEALL;
            arr.markers.push_back(marker);
        }
        createMarkers(req, res, arr);
        marker_array_publisher.publish(arr);
    }


    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "slic3r_coverage_planner");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    visualize_plan = paramNh.param("visualize_plan", true);
    cell_fill = paramNh.param("cell_fill", false);

    if (visualize_plan) {
        marker_array_publisher = n.advertise<visualization_msgs::MarkerArray>(
                "slic3r_coverage_planner/path_marker_array", 100, true);
    }

    ros::ServiceServer plan_path_srv = n.advertiseService("slic3r_coverage_planner/plan_path", planPath);

    ros::spin();
    return 0;
}
