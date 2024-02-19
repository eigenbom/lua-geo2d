--[[
2D Computational Geometry  

This library provides a set of functions for 2D computational geometry.  

Major sections are ported from Arash Partow's Wykobi Computational Geometry Library.  

Shapes:  
* AABB - Axis aligned bounding box defined by min and max point  
* Line - Infinite line defined by two points  
* Quad - Convex quadrilateral defined by four points  
* Ray - Ray defined by an origin and a normalized direction  
* Segment - Line segment defined by two points  
* Triangle - Triangle defined by three points  
* Pcurve - A parameteric curve defined by a function that maps a parameter t in [0,1] to a point (x,y)  
* Polygon - Convex or concave polygon defined by a set of points, overlapping edges are not supported  
* Polyline - A series of connected line segments, stored as a flat array of x,y coordinates  

Notes:  
* The `intersect` functions will test for intersection of filled shapes by default  
  To test intersection with the outline of the shape instead, pass in true for the parameter `outline_only`  
  Some intersect functions like `intersect_line_quad` do not support outline only due to the infinite length of lines  
* The intersection functions are narrow, it's recommend you do a broadphase test first  
* Precision is determined by a module constant `PRECISION` and can be adjusted to suit your data,  
  alternatively you can adjust your data to fit the default precision of the library  
* Modify dependencies to use custom library functions  

Debug Mode:  
* The file is also available in debug mode (lua51_debug\geo2d.lua)  
  In debug mode the library performs some checks on parameters (e.g., that an AABB has min <= max or that a polygon is non-intersecting)  
* The constant M.DEBUG will be true if the library is built in debug mode  


Functions By Category:  
* Collinear, perpendicular, parallel, tangent  
	* collinear  
	* is_point_collinear  
	* perpendicular  
	* parallel  
	* orientation  
	* is_tangent  

* 2D vector functions  
	* rotate  
	* rotate_around_origin  
	* normalize  
	* distance  
	* sq_distance  
	* dot_product  

* Miscellaneous functions  
	* cartesian_angle  
	* vertex_angle  
	* quadrant_by_point  
	* quadrant_by_angle  
	* vertical_mirror  
	* horizontal_mirror  
	* normalize_angle  
	* point_of_reflection  

* Shape intersections  
	* intersect_aabb_aabb  
	* intersect_aabb_circle  
	* intersect_aabb_line  
	* intersect_aabb_pcurve  
	* intersect_aabb_polyline  
	* intersect_aabb_polygon  
	* intersect_circle_circle  
	* intersect_circle_pcurve  
	* intersect_circle_polyline  
	* intersect_circle_polygon  
	* intersect_line_circle  
	* intersect_line_pcurve  
	* intersect_line_polyline  
	* intersect_line_polygon  
	* intersect_line_line  
	* intersect_line_quad  
	* intersect_line_triangle  
	* intersect_pcurve_pcurve  
	* intersect_pcurve_polygon  
	* intersect_polygon_polygon  
	* intersect_polyline_polyline  
	* intersect_polyline_polygon  
	* intersect_polyline_pcurve  
	* intersect_quad_aabb  
	* intersect_quad_circle  
	* intersect_quad_pcurve  
	* intersect_quad_polyline  
	* intersect_quad_polygon  
	* intersect_quad_quad  
	* intersect_ray_aabb  
	* intersect_ray_circle  
	* intersect_ray_line  
	* intersect_ray_pcurve  
	* intersect_ray_polygon  
	* intersect_ray_polyline  
	* intersect_ray_quad  
	* intersect_ray_ray  
	* intersect_ray_segment  
	* intersect_ray_triangle  
	* intersect_segment_aabb  
	* intersect_segment_circle  
	* intersect_segment_pcurve  
	* intersect_segment_polyline  
	* intersect_segment_polygon  
	* intersect_segment_line  
	* intersect_segment_quad  
	* intersect_segment_segment  
	* intersect_segment_segment_simple  
	* intersect_segment_triangle  
	* intersect_triangle_aabb  
	* intersect_triangle_circle  
	* intersect_triangle_pcurve  
	* intersect_triangle_polyline  
	* intersect_triangle_polygon  
	* intersect_triangle_quad  
	* intersect_triangle_triangle  

* Shape intersections (return points)  
	* intersection_point_aabb_aabb  
	* intersection_point_aabb_circle  
	* intersection_point_aabb_line  
	* intersection_point_aabb_pcurve  
	* intersection_point_aabb_polygon  
	* intersection_point_aabb_polyline  
	* intersection_point_circle_circle  
	* intersection_point_circle_pcurve  
	* intersection_point_circle_polygon  
	* intersection_point_circle_polyline  
	* intersection_point_line_circle  
	* intersection_point_line_pcurve  
	* intersection_point_line_polygon  
	* intersection_point_line_polyline  
	* intersection_point_line_line  
	* intersection_point_line_quad  
	* intersection_point_line_triangle  
	* intersection_point_ray_aabb  
	* intersection_point_ray_circle  
	* intersection_point_ray_pcurve  
	* intersection_point_ray_polyline  
	* intersection_point_ray_line  
	* intersection_point_ray_polygon  
	* intersection_point_ray_quad  
	* intersection_point_ray_ray  
	* intersection_point_ray_segment  
	* intersection_point_ray_triangle  
	* intersection_point_pcurve_polygon  
	* intersection_point_pcurve_pcurve  
	* intersection_point_polygon_polygon  
	* intersection_point_polyline_polygon  
	* intersection_point_polyline_polyline  
	* intersection_point_polyline_pcurve  
	* intersection_point_segment_aabb  
	* intersection_point_segment_circle  
	* intersection_point_segment_pcurve  
	* intersection_point_segment_polygon  
	* intersection_point_segment_polyline  
	* intersection_point_segment_line  
	* intersection_point_segment_quad  
	* intersection_point_segment_segment  
	* intersection_point_segment_triangle  
	* intersection_point_triangle_aabb  
	* intersection_point_triangle_circle  
	* intersection_point_triangle_pcurve  
	* intersection_point_triangle_polygon  
	* intersection_point_triangle_polyline  
	* intersection_point_triangle_quad  
	* intersection_point_triangle_triangle  
	* intersection_point_quad_aabb  
	* intersection_point_quad_circle  
	* intersection_point_quad_pcurve  
	* intersection_point_quad_polygon  
	* intersection_point_quad_polyline  
	* intersection_point_quad_quad  

* Shape containment  
	* aabb_within_aabb  
	* circle_within_aabb  
	* circle_within_circle  
	* pcurve_within_aabb  
	* points_within_aabb  
	* quad_within_aabb  
	* segment_within_aabb  
	* triangle_within_aabb  

* Test if a point lies within a closed shape  
	* point_in_aabb  
	* point_in_circle  
	* point_in_circumcircle  
	* point_in_focus_area  
	* point_in_polygon  
	* point_in_quad  
	* point_in_triangle  

* Test if a point lies on a shape (or outline for closed shapes)  
	* point_on_aabb  
	* point_on_circle  
	* point_on_line  
	* point_on_quad  
	* point_on_ray  
	* point_on_segment  
	* point_on_triangle  

* Closest point to a closed shape  
	* closest_point_in_aabb_from_point  
	* closest_point_in_circle_from_circle  
	* closest_point_in_circle_from_point  
	* closest_point_in_circle_from_segment  
	* closest_point_in_polygon_from_point  
	* closest_point_in_quad_from_point  
	* closest_point_in_triangle_from_point  

* Closest point to a shape (or outline for closed shapes)  
	* closest_point_on_aabb_from_point  
	* closest_point_on_circle_from_circle  
	* closest_point_on_circle_from_point  
	* closest_point_on_circle_from_segment  
	* closest_point_on_line_from_point  
	* closest_point_on_pcurve_from_point  
	* closest_point_on_polygon_from_point  
	* closest_point_on_polyline_from_point  
	* closest_point_on_quad_from_point  
	* closest_point_on_ray_from_point  
	* closest_point_on_segment_from_point  
	* closest_point_on_triangle_from_point  

* AABB from shapes  
	* aabb_circle  
	* aabb_pcurve  
	* aabb_points  
	* aabb_quad  
	* aabb_segment  
	* aabb_triangle  

* AABB operations  
	* aabb_intersection  
	* aabb_union  

* Measurements  
	* area_aabb  
	* area_circle  
	* area_polygon  
	* area_quad  
	* area_triangle  
	* area_triangle_signed  

	* centroid_aabb  
	* centroid_line  
	* centroid_segment  
	* centroid_pcurve  
	* centroid_polygon  
	* centroid_points  
	* centroid_quad  
	* centroid_triangle  
	  
	* length_pcurve  
	* length_polyline  
	* length_segment  

	* perimeter_aabb  
	* perimeter_circle  
	* perimeter_polygon  
	* perimeter_quad  
	* perimeter_triangle  

* Geometric transformations  
	* rotate_aabb  
	* rotate_line  
	* rotate_point  
	* rotate_points  
	* rotate_points_in_place  
	* rotate_quad  
	* rotate_ray  
	* rotate_segment  
	* rotate_triangle  

	* rotate_aabb_around_origin  
	* rotate_line_around_origin  
	* rotate_point_around_origin  
	* rotate_points_around_origin  
	* rotate_points_in_place_around_origin  
	* rotate_quad_around_origin  
	* rotate_ray_around_origin  
	* rotate_segment_around_origin  
	* rotate_triangle_around_origin  

	* scale_aabb  
	* scale_line  
	* scale_point  
	* scale_points  
	* scale_points_in_place  
	* scale_quad  
	* scale_segment  
	* scale_triangle  

	* translate_aabb  
	* translate_line  
	* translate_points  
	* translate_points_in_place  
	* translate_quad  
	* translate_segment  
	* translate_triangle  

	* mirror_aabb  
	* mirror_line  
	* mirror_point  
	* mirror_points  
	* mirror_points_in_place  
	* mirror_quad  
	* mirror_ray  
	* mirror_segment  
	* mirror_triangle  

	* project_aabb_onto_axis  
	* project_circle_onto_axis  
	* project_pcurve_onto_axis  
	* project_point_onto_axis  
	* project_points_onto_axis  
	* project_quad_onto_axis  
	* project_ray_onto_axis  
	* project_segment_onto_axis  
	* project_triangle_onto_axis  

* Curves and splines  
	* cubic_bezier_coefficients  
	* quadratic_bezier_coefficients  
	* evaluate_quadratic_equation  
	* evaluate_cubic_equation  
	* evaluate_cubic_bezier  
	* evaluate_quadratic_bezier  
	* pcurve_from_cubic_hermite  
	* pcurve_from_cubic_bezier  
	* pcurve_from_quadratic_bezier  
	* polyline_from_cubic_bezier  
	* polyline_from_cubic_hermite  
	* polyline_from_quadratic_bezier  

* Generate random uniform points within shapes  
	* random_point_in_aabb  
	* random_point_in_circle  
	* random_points_in_polygon  
	* random_points_in_polyline  
	* random_point_in_quad  
	* random_point_in_segment  
	* random_point_in_triangle  
	* random_points_in_triangles  

* Clipping of polygons and segments  
	* clip_polygon_aabb  
	* clip_polygon_polygon (sutherland-hodgman)  
	* clip_segment_aabb  
	* clip_segment_circle  
	* clip_segment_quad  
	* clip_segment_triangle  

* Miscellaneous polygon functions:  
	* polygon_ordered_from_points  
	* is_polygon_convex  

* Polygon triangulation and decomposition  
	* triangles_from_polygon  
	* triangle_indices_from_polygon  
	* convex_polygons_from_polygon  

* Hull and Bounds  
	* convex_hull (graham scan)  
	* minimum_bounding_circle_randomized  
	* minimum_bounding_circle_ritter  
]]
---@class geo2d_module
local M = {}

------------------------------------------------------------------------
-- Debug Utils
------------------------------------------------------------------------

M.DEBUG = true

local function __bad_arg_nil_param(name)
	return string.format("bad argument '%s' (value expected, got nil)", name)
end

local function __bad_arg_param(name, expected_type, actual_type)
	return string.format("bad argument '%s' (%s expected, got type %s)", name, expected_type, actual_type)
end

---@param name string
---@param value any
---@param expected_type? string
local function _check(name, value, expected_type)
	if expected_type then
		local value_type = type(value)
		if value_type ~= expected_type then
			error(__bad_arg_param(name, expected_type, value_type), 2)
		end
	elseif value == nil then
		error(__bad_arg_nil_param(name), 2)
	end
end

local function _assert(condition, str, ...)
	if not condition then
		error(string.format(str, ...), 2)
	end
end

-- Debug function for printing out table contents
local function _dump(t, tab)
	local indent = string.rep("  ", tab or 1)
	local last_indent = string.rep("  ", tab and tab - 1 or 0)
	if type(t) ~= "table" then
		if type(t) == 'number' then
			return string.format("%.2f", t)
		else
			return tostring(t)
		end
	end
	local out = {}
	for k, v in pairs(t) do
		table.insert(out, string.format("%s=%s", k, _dump(v, tab and tab + 1 or 2)))
	end
	return "{\n" .. indent .. table.concat(out, "\n" .. indent) .. "\n" .. last_indent .. "}"
end


-----------------------------------------------------------
-- Algorithmic Constants
-----------------------------------------------------------

-- Determines the precision of floating point comparisons
-- Adjust this to suit the precision of your data
local PRECISION = 1e-9

-----------------------------------------------------------
-- Dependencies
--
-- Can redefine these to use other math libraries
-- (eg LUT-based trig)
-----------------------------------------------------------

local math_abs         = math.abs
local math_min         = math.min
local math_max         = math.max
local math_floor       = math.floor
local math_sqrt        = math.sqrt
local math_sin         = math.sin
local math_cos         = math.cos
local math_acos        = math.acos
local math_atan        = math.atan
local math_huge        = math.huge

local table_insert     = table.insert
local table_remove     = table.remove
local table_sort       = table.sort

-----------------------------------------------------------
-- Types and Enums
-----------------------------------------------------------

---An angle in degrees
---@alias degrees number
---An angle in radians
---@alias radians number

-----------------------------------------------------------
-- Constants
-----------------------------------------------------------

local PI               = 3.141592653589793238462643383279500
local PI_MUL_2         = 6.283185307179586476925286766559000
local PI_DIV_180       = 0.017453292519943295769236907684886
local _180_DIV_PI      = 57.295779513082320876798154814105000
local NAN              = 0.0 / 0.0

-- Orientation
local CLOCKWISE        = -1
local COUNTERCLOCKWISE = 1
local RIGHT_HAND_SIDE  = -1
local LEFT_HAND_SIDE   = 1
local COLLINEAR        = 0

-----------------------------------------------------------
-- Helpers
-----------------------------------------------------------

---@param a number
---@param b number
---@param epsilon? number
---@return boolean
local function is_equal(a, b, epsilon)
	local diff = a - b
	local eps = epsilon or PRECISION
	return (-eps <= diff) and (diff <= eps)
end

---@param a number
---@param b number
---@param epsilon? number
---@return boolean
local function not_equal(a, b, epsilon)
	local diff = a - b
	local eps = epsilon or PRECISION
	return (-eps > diff) or (diff > eps)
end

---@param a number
---@param b number
---@param epsilon? number
---@return boolean
local function less_than_or_equal(a, b, epsilon)
	return (a < b) or is_equal(a, b, epsilon)
end

---@param a number
---@param b number
---@param epsilon? number
---@return boolean
local function greater_than_or_equal(a, b, epsilon)
	return (a > b) or is_equal(a, b, epsilon)
end

---@param src_x number
---@param src_y number
---@param dest_x number
---@param dest_y number
---@param t number
---@return number, number
local function project_point_t(src_x, src_y, dest_x, dest_y, t)
	return src_x + t * (dest_x - src_x), src_y + t * (dest_y - src_y)
end

---@param src_x number
---@param src_y number
---@param dest_x number
---@param dest_y number
---@param dist number
---@return number, number
local function project_point(src_x, src_y, dest_x, dest_y, dist)
	local dx, dy = dest_x - src_x, dest_y - src_y
	if is_equal(dx, 0) and is_equal(dy, 0) then
		return src_x, src_y
	end
	local t = dist / math_sqrt(dx * dx + dy * dy)
	return src_x + t * dx, src_y + t * dy
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return boolean
local function are_points_equal(x1, y1, x2, y2)
	return is_equal(x1, x2) and is_equal(y1, y2)
end

-----------------------------------------------------------
-- Circle
-----------------------------------------------------------

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number
local function circumcenter(x1, y1, x2, y2, x3, y3)
	local a = x2 - x1
	local b = y2 - y1
	local c = x3 - x1
	local d = y3 - y1
	local e = a * (x1 + x2) + b * (y1 + y2)
	local f = c * (x1 + x3) + d * (y1 + y3)
	local g = 2.0 * (a * (y3 - y2) - b * (x3 - x2))
	if g == 0.0 then
		return math_huge, math_huge
	else
		return (d * e - b * f) / g, (a * f - c * e) / g
	end
end

---Return a circle passing through two points
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number,number,number -- x, y, radius
function M.circle_from_two_points(x1, y1, x2, y2)
	local x, y = (x1 + x2) / 2, (y1 + y2) / 2
	local radius = M.distance(x1, y1, x2, y2) / 2
	return x, y, radius
end

---Return a circle passing through three points (circumcircle)
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number,number,number -- x, y, radius
function M.circle_from_three_points(x1, y1, x2, y2, x3, y3)
	local x, y = circumcenter(x1, y1, x2, y2, x3, y3)
	local radius = M.distance(x, y, x1, y1)
	return x, y, radius
end

-----------------------------------------------------------
-- Vector functions
-----------------------------------------------------------

---Normalize a vector
---@param x number
---@param y number
---@return number,number -- x, y
function M.normalize(x, y)
	local length = math_sqrt(x * x + y * y)
	local inv_length = length ~= 0 and 1.0 / length or 0
	return x * inv_length, y * inv_length
end

---Distance between two points
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
function M.distance(x1, y1, x2, y2)
	local dx, dy = x2 - x1, y2 - y1
	return math_sqrt(dx * dx + dy * dy)
end

---Squared distance between two points
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
function M.sq_distance(x1, y1, x2, y2)
	local dx, dy = x2 - x1, y2 - y1
	return dx * dx + dy * dy
end

---Rotate point
---@param angle degrees
---@param x number
---@param y number
function M.rotate(angle, x, y)
	local sin_val = math_sin(angle * PI_DIV_180)
	local cos_val = math_cos(angle * PI_DIV_180)
	local nx = (x * cos_val) - (y * sin_val)
	local ny = (y * cos_val) + (x * sin_val)
	return nx, ny
end

---Rotate point around origin
---@param angle degrees
---@param x number
---@param y number
---@param ox number
---@param oy number
function M.rotate_around_origin(angle, x, y, ox, oy)
	local sin_val = math_sin(angle * PI_DIV_180)
	local cos_val = math_cos(angle * PI_DIV_180)
	local nx = ((x - ox) * cos_val) - ((y - oy) * sin_val)
	local ny = ((y - oy) * cos_val) + ((x - ox) * sin_val)
	return nx + ox, ny + oy
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
function M.dot_product(x1, y1, x2, y2)
	return x1 * x2 + y1 * y2
end

-----------------------------------------------------------
-- Area, Centroid, Perimeter calculation
-----------------------------------------------------------

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.area_aabb(min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return (max_x - min_x) * (max_y - min_y)
end

---@param radius number
function M.area_circle(radius)
	return PI * radius * radius
end

---Area of a polygon
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@return number
function M.area_polygon(polygon)
	assert(#polygon >= 6, "bad argument (polygon must have at least 3 points)")

	local result = 0
	local j = #polygon - 1
	for i = 1, #polygon, 2 do
		result = result + ((polygon[j] * polygon[i + 1]) - (polygon[j + 1] * polygon[i]))
		j = i
	end
	return math_abs(result / 2)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
function M.area_quad(x1, y1, x2, y2, x3, y3, x4, y4)
	return 0.5 * (
		(x1 * (y2 - y4)) +
		(x2 * (y3 - y1)) +
		(x3 * (y4 - y2)) +
		(x4 * (y1 - y3))
	)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
function M.area_triangle(x1, y1, x2, y2, x3, y3)
	return math_abs(((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2)
end

---Calculate signed area of a triangle
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
function M.area_triangle_signed(x1, y1, x2, y2, x3, y3)
	return ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.centroid_aabb(min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return (min_x + max_x) / 2, (min_y + max_y) / 2
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
function M.centroid_line(x1, y1, x2, y2)
	return (x1 + x2) / 2, (y1 + y2) / 2
end

---@param polygon number[] -- (x1, y1, x2, y2, ...)
function M.centroid_polygon(polygon)
	assert(#polygon >= 6, "Polygon must have at least 3 points")

	local area = 0
	local cx = 0
	local cy = 0
	local j = #polygon - 1
	for i = 1, #polygon, 2 do
		local term = polygon[j] * polygon[i + 1] - polygon[j + 1] * polygon[i]
		area = area + term
		cx = cx + (polygon[j] + polygon[i]) * term
		cy = cy + (polygon[j + 1] + polygon[i + 1]) * term
		j = i
	end
	local denom = 1 / (area * 3)
	return cx * denom, cy * denom
end

---Samples pcurve at point_count points and returns the centroid
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
function M.centroid_pcurve(point_count, pcurve)
	local t = 0.0
	local dt = 1.0 / (1.0 * point_count - 1.0)
	local sum_x, sum_y = pcurve(t)
	t = dt
	for i = 1, point_count - 1 do
		local x, y = pcurve(t)
		sum_x, sum_y = sum_x + x, sum_y + y
		t = t + dt
	end
	return sum_x / point_count, sum_y / point_count
end

---Weighted centroid of a set of points
---@param points number[] -- (x1, y1, x2, y2, ...)
function M.centroid_points(points)
	assert(#points >= 2, "points must have at least 1 point")
	local sum_x, sum_y = points[1], points[2]
	local npoints = #points / 2
	for i = 2, npoints do
		local x, y = points[2*(i)-1], points[2*(i)]
		sum_x, sum_y = sum_x + x, sum_y + y
	end
	return sum_x / npoints, sum_y / npoints
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number, number
function M.centroid_quad(x1, y1, x2, y2, x3, y3, x4, y4)
	local term = ((x4 * y1) - (y4 * x1))
	local sum = term
	local x = (x4 + x1) * term
	local y = (y4 + y1) * term

	term = ((x1 * y2) - (y1 * x2))
	sum = sum + term
	x = x + (x1 + x2) * term
	y = y + (y1 + y2) * term

	term = ((x2 * y3) - (y2 * x3))
	sum = sum + term
	x = x + (x2 + x3) * term
	y = y + (y2 + y3) * term

	term = ((x3 * y4) - (y3 * x4))
	sum = sum + term
	x = x + (x3 + x4) * term
	y = y + (y3 + y4) * term

	local denom = sum ~= 0 and (1 / (3 * sum)) or 1
	return x * denom, y * denom
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
function M.centroid_segment(x1, y1, x2, y2)
	return (x1 + x2) * 0.5, (y1 + y2) * 0.5
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number
function M.centroid_triangle(x1, y1, x2, y2, x3, y3)
	local midx1, midy1 = M.centroid_segment(x2, y2, x3, y3)
	local midx2, midy2 = M.centroid_segment(x1, y1, x3, y3)
	return M.intersection_point_line_line(x1, y1, midx1, midy1, x2, y2, midx2, midy2) --[[@as number,number]]
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.perimeter_aabb(min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return 2 * ((max_x - min_x) + (max_y - min_y))
end

---@param radius number
function M.perimeter_circle(radius)
	return 2 * PI * radius
end

---@param polygon number[] -- (x1, y1, x2, y2, ...)
function M.perimeter_polygon(polygon)
	local perimeter = 0
	local npoints = #polygon / 2
	for i = 1, npoints do
		local x1, y1 = polygon[2*(i)-1], polygon[2*(i)]
		local j = 1 + i % npoints
		local x2, y2 = polygon[2*(j)-1], polygon[2*(j)]
		perimeter = perimeter + M.distance(x1, y1, x2, y2)
	end
	return perimeter
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
function M.perimeter_quad(x1, y1, x2, y2, x3, y3, x4, y4)
	local side1 = M.distance(x2, y2, x1, y1)
	local side2 = M.distance(x3, y3, x2, y2)
	local side3 = M.distance(x4, y4, x3, y3)
	local side4 = M.distance(x1, y1, x4, y4)
	return side1 + side2 + side3 + side4
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
function M.perimeter_triangle(x1, y1, x2, y2, x3, y3)
	local side1 = M.distance(x2, y2, x1, y1)
	local side2 = M.distance(x3, y3, x2, y2)
	local side3 = M.distance(x1, y1, x3, y3)
	return side1 + side2 + side3
end

---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
function M.length_pcurve(point_count, pcurve)
	local length = 0
	local prev_x, prev_y = pcurve(0)
	local t = 1.0 / (point_count - 1)
	for i = 1, point_count - 1 do
		local x, y = pcurve(t * i)
		local dx, dy = x - prev_x, y - prev_y
		length = length + math_sqrt(dx * dx + dy * dy)
		prev_x, prev_y = x, y
	end
	return length
end

---@param polyline number[] -- (x1, y1, x2, y2, ...)
function M.length_polyline(polyline)
	assert(#polyline >= 4, "Polyline must have at least 2 points")
	local prev_x, prev_y = polyline[1], polyline[2]
	local length = 0
	local npoints = #polyline / 2
	for i = 2, npoints do
		local x, y = polyline[2*(i)-1], polyline[2*(i)]
		local dx, dy = x - prev_x, y - prev_y
		length = length + math_sqrt(dx * dx + dy * dy)
		prev_x, prev_y = x, y
	end
	return length
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
function M.length_segment(x1, y1, x2, y2)
	local dx, dy = x2 - x1, y2 - y1
	return math_sqrt(dx * dx + dy * dy)
end

-----------------------------------------------------------
-- AABB calculation
-----------------------------------------------------------

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number,number,number,number -- min_x, min_y, max_x, max_y
function M.aabb_segment(x1, y1, x2, y2)
	local min_x, max_x ---@type number, number

	if x1 < x2 then
		min_x, max_x = x1, x2
	else
		min_x, max_x = x2, x1
	end

	if y1 < y2 then
		return min_x, y1, max_x, y2
	else
		return min_x, y2, max_x, y1
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number,number,number,number -- min_x, min_y, max_x, max_y
function M.aabb_triangle(x1, y1, x2, y2, x3, y3)
	local min_x, min_y, max_x, max_y = x1, y1, x1, y1
	min_x, max_x, min_y, max_y = math_min(min_x, x2), math_max(max_x, x2), math_min(min_y, y2), math_max(max_y, y2)
	min_x, max_x, min_y, max_y = math_min(min_x, x3), math_max(max_x, x3), math_min(min_y, y3), math_max(max_y, y3)
	return min_x, min_y, max_x, max_y
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number,number,number,number -- min_x, min_y, max_x, max_y
function M.aabb_quad(x1, y1, x2, y2, x3, y3, x4, y4)
	local min_x, max_x, min_y, max_y = x1, x1, y1, y1
	min_x, max_x, min_y, max_y = math_min(min_x, x2), math_max(max_x, x2), math_min(min_y, y2), math_max(max_y, y2)
	min_x, max_x, min_y, max_y = math_min(min_x, x3), math_max(max_x, x3), math_min(min_y, y3), math_max(max_y, y3)
	min_x, max_x, min_y, max_y = math_min(min_x, x4), math_max(max_x, x4), math_min(min_y, y4), math_max(max_y, y4)
	return min_x, min_y, max_x, max_y
end

---@param x number
---@param y number
---@param radius number
---@return number,number,number,number -- min_x, min_y, max_x, max_y
function M.aabb_circle(x, y, radius)
	local min_x = x - radius
	local min_y = y - radius
	local max_x = x + radius
	local max_y = y + radius
	return min_x, min_y, max_x, max_y
end

---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
function M.aabb_pcurve(point_count, pcurve)
	local min_x, min_y = pcurve(0)
	local max_x, max_y = min_x, min_y
	for i = 1, point_count - 1 do
		local x, y = pcurve(i / (point_count - 1))
		min_x, min_y = math_min(min_x, x), math_min(min_y, y)
		max_x, max_y = math_max(max_x, x), math_max(max_y, y)
	end
	return min_x, min_y, max_x, max_y
end

---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number,number,number,number -- min_x, min_y, max_x, max_y
function M.aabb_points(points)
	assert(#points >= 2, "points must have at least 1 point")

	local min_x, min_y = points[1], points[2]
	local max_x, max_y = points[1], points[2]
	for i = 3, #points, 2 do
		min_x = math_min(min_x, points[i])
		max_x = math_max(max_x, points[i])
		min_y = math_min(min_y, points[i + 1])
		max_y = math_max(max_y, points[i + 1])
	end
	return min_x, min_y, max_x, max_y
end

---@param a_min_x number
---@param a_min_y number
---@param a_max_x number
---@param a_max_y number
---@param b_min_x number
---@param b_min_y number
---@param b_max_x number
---@param b_max_y number
---@return number?, number?, number?, number?
function M.aabb_intersection(a_min_x, a_min_y, a_max_x, a_max_y, b_min_x, b_min_y, b_max_x, b_max_y)
	_assert(a_min_x <= a_max_x and a_min_y <= a_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", a_min_x, a_max_x, a_min_y, a_max_y)
	_assert(b_min_x <= b_max_x and b_min_y <= b_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", b_min_x, b_max_x, b_min_y, b_max_y)

	if M.intersect_aabb_aabb(a_min_x, a_min_y, a_max_x, a_max_y, b_min_x, b_min_y, b_max_x, b_max_y) then
		local cx1 = a_min_x < b_min_x and b_min_x or a_min_x
		local cx2 = a_max_x > b_max_x and b_max_x or a_max_x
		local cy1 = a_min_y < b_min_y and b_min_y or a_min_y
		local cy2 = a_max_y > b_max_y and b_max_y or a_max_y
		return cx1, cy1, cx2, cy2
	end
end

---@param a_min_x number
---@param a_min_y number
---@param a_max_x number
---@param a_max_y number
---@param b_min_x number
---@param b_min_y number
---@param b_max_x number
---@param b_max_y number
---@return number, number, number, number
function M.aabb_union(a_min_x, a_min_y, a_max_x, a_max_y, b_min_x, b_min_y, b_max_x, b_max_y)
	_assert(a_min_x <= a_max_x and a_min_y <= a_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", a_min_x, a_max_x, a_min_y, a_max_y)
	_assert(b_min_x <= b_max_x and b_min_y <= b_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", b_min_x, b_max_x, b_min_y, b_max_y)

	local cx1 = a_min_x < b_min_x and a_min_x or b_min_x
	local cx2 = a_max_x > b_max_x and a_max_x or b_max_x
	local cy1 = a_min_y < b_min_y and a_min_y or b_min_y
	local cy2 = a_max_y > b_max_y and a_max_y or b_max_y
	return cx1, cy1, cx2, cy2
end

-----------------------------------------------------------
-- Function to check parallel, perpendicular, collinear
-- and orientation properties of lines and points
-----------------------------------------------------------

---Check if the lines (x1,y1)->(x2,y2) and (x3,y3)->(x4,y4) are parallel
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param epsilon? number
function M.parallel(x1, y1, x2, y2, x3, y3, x4, y4, epsilon)
	return is_equal((y1 - y2) * (x3 - x4), (y3 - y4) * (x1 - x2), epsilon)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param epsilon? number
---@package
---@private
function M._robust_parallel(x1, y1, x2, y2, x3, y3, x4, y4, epsilon)
	local px1, py1 = M.closest_point_on_line_from_point(x1, y1, x2, y2, x3, y3)
	local px2, py2 = M.closest_point_on_line_from_point(x1, y1, x2, y2, x4, y4)

	return is_equal(M.distance(x3, y3, px1, py1), M.distance(x4, y4, px2, py2), epsilon)
end

---Check if the lines (x1,y1)->(x2,y2) and (x3,y3)->(x4,y4) are perpendicular
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param epsilon? number
function M.perpendicular(x1, y1, x2, y2, x3, y3, x4, y4, epsilon)
	return is_equal(-1.0 * (y2 - y1) * (y4 - y3), (x4 - x3) * (x2 - x1), epsilon)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param epsilon? number
---@package
---@private
function M._robust_perpendicular(x1, y1, x2, y2, x3, y3, x4, y4, epsilon)
	local p1x, p1y, p2x, p2y = 0.0, 0.0, 0.0, 0.0
	p1x, p1y = M.closest_point_on_line_from_point(x1, y1, x2, y2, x3, y3)
	p2x, p2y = M.closest_point_on_line_from_point(x1, y1, x2, y2, x4, y4)
	return is_equal(M.distance(p1x, p1y, p2x, p2y), 0.0, epsilon)
end

---Check if three points are collinear
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param epsilon? number
function M.collinear(x1, y1, x2, y2, x3, y3, epsilon)
	return is_equal((x2 - x1) * (y3 - y1), (x3 - x1) * (y2 - y1), epsilon)
end

--- Are three points collinear
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param epsilon? number
---@package
---@private
function M._robust_collinear(x1, y1, x2, y2, x3, y3, epsilon)
	local d1 = M.sq_distance(x1, y1, x2, y2)
	local d2 = M.sq_distance(x2, y2, x3, y3)
	local d3 = M.sq_distance(x3, y3, x1, y1)

	if d1 >= d2 then
		if d1 >= d3 then
			return is_equal(M._minimum_distance_from_point_to_line(x3, y3, x1, y1, x2, y2), 0.0, epsilon)
		else
			return is_equal(M._minimum_distance_from_point_to_line(x2, y2, x3, y3, x1, y1), 0.0, epsilon)
		end
	elseif d2 >= d3 then
		return is_equal(M._minimum_distance_from_point_to_line(x1, y1, x2, y2, x3, y3), 0.0, epsilon)
	else
		return is_equal(M._minimum_distance_from_point_to_line(x2, y2, x3, y3, x1, y1), 0.0, epsilon)
	end
end

---Check if the point (px,py) is collinear to the line segment (x1,y1)->(x2,y2)
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param px number
---@param py number
---@param robust? boolean
function M.is_point_collinear(x1, y1, x2, y2, px, py, robust)
	if (((less_than_or_equal(x1, px) and less_than_or_equal(px, x2)) or
				(less_than_or_equal(x2, px) and less_than_or_equal(px, x1))) and
			((less_than_or_equal(y1, py) and less_than_or_equal(py, y2)) or
				(less_than_or_equal(y2, py) and less_than_or_equal(py, y1)))) then
		if robust then
			return M._robust_collinear(x1, y1, x2, y2, px, py)
		else
			return M.collinear(x1, y1, x2, y2, px, py)
		end
	end

	return false
end

---Determine if (px, py) lies on the left or right side of the line (x1,y1)->(x2,y2)
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param px number
---@param py number
---@return integer -- 1 if left, -1 if right, 0 if collinear
function M.orientation(x1, y1, x2, y2, px, py)
	local orin = (x2 - x1) * (py - y1) - (px - x1) * (y2 - y1)
	if orin > 0 then
		return LEFT_HAND_SIDE
	elseif orin < 0 then
		return RIGHT_HAND_SIDE
	else
		return COLLINEAR
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param p1x number
---@param p1y number
---@param p2x number
---@param p2y number
---@package
---@private
function M._differing_orientation(x1, y1, x2, y2, p1x, p1y, p2x, p2y)
	return (M.orientation(x1, y1, x2, y2, p1x, p1y) * M.orientation(x1, y1, x2, y2, p2x, p2y)) == -1
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param px number
---@param py number
---@package
---@private
function M._robust_orientation(x1, y1, x2, y2, px, py)
	local orin = (x2 - x1) * (py - y1) - (px - x1) * (y2 - y1)
	if is_equal(orin, 0) then
		return COLLINEAR
	elseif orin < 0 then
		return RIGHT_HAND_SIDE
	else
		return LEFT_HAND_SIDE
	end
end

--- Check if a line lies tangent to a circle
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param cx number
---@param cy number
---@param cr number
function M.is_tangent(x1, y1, x2, y2, cx, cy, cr)
	local tx1, tx2, ty1, ty2 = x1 - cx, x2 - cx, y1 - cy, y2 - cy
	local rsqr = cr * cr
	local drsqr = M.sq_distance(tx1, ty1, tx2, ty2)
	local dsqr = (x1 * y2) - (x2 * y1)
	return is_equal(rsqr * drsqr - dsqr, 0.0)
end

-----------------------------------------------------------
-- Closest point
-----------------------------------------------------------

---Return the closest point in the AABB to (px,py)
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param px number
---@param py number
---@return number,number
function M.closest_point_in_aabb_from_point(min_x, min_y, max_x, max_y, px, py)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	local nx = (px <= min_x and min_x) or (px >= max_x and max_x) or px
	local ny = (py <= min_y and min_y) or (py >= max_y and max_y) or py
	return nx, ny
end

---@param c1_x number
---@param c1_y number
---@param c1_r number
---@param c2_x number
---@param c2_y number
---@param c2_r number
---@return number, number
function M.closest_point_in_circle_from_circle(c1_x, c1_y, c1_r, c2_x, c2_y, c2_r)
	local px, py = M.closest_point_in_circle_from_point(c2_x, c2_y, c2_r, c1_x, c1_y)
	return M.closest_point_in_circle_from_point(c1_x, c1_y, c1_r, px, py)
end

---@param c_x number
---@param c_y number
---@param c_r number
---@param px number
---@param py number
---@return number,number
function M.closest_point_in_circle_from_point(c_x, c_y, c_r, px, py)
	local dx, dy = px - c_x, py - c_y
	if (dx * dx + dy * dy) <= (c_r * c_r) then
		return px, py
	end
	local ratio = c_r / math_sqrt(dx * dx + dy * dy)
	return c_x + ratio * dx, c_y + ratio * dy
end

---@param c_x number
---@param c_y number
---@param c_r number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number
function M.closest_point_in_circle_from_segment(c_x, c_y, c_r, x1, y1, x2, y2)
	local nx, ny = M.closest_point_on_segment_from_point(x1, y1, x2, y2, c_x, c_y)
	local sq_dist = M.sq_distance(c_x, c_y, nx, ny)
	if sq_dist <= c_r * c_r then
		return nx, ny
	else
		local ratio = c_r / math_sqrt(sq_dist)
		local px = c_x + ratio * (nx - c_x)
		local py = c_y + ratio * (ny - c_y)
		return px, py
	end
end

---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param px number
---@param py number
---@return number, number
function M.closest_point_in_polygon_from_point(polygon, px, py)
	if M.point_in_polygon(px, py, polygon) then
		return px, py
	end
	return M.closest_point_on_polygon_from_point(polygon, px, py)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param px number
---@param py number
function M.closest_point_in_quad_from_point(x1, y1, x2, y2, x3, y3, x4, y4, px, py)
	if M.point_in_quad(px, py, x1, y1, x2, y2, x3, y3, x4, y4) then
		return px, py
	end

	return M.closest_point_on_quad_from_point(x1, y1, x2, y2, x3, y3, x4, y4, px, py)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param px number
---@param py number
---@return number,number
function M.closest_point_in_triangle_from_point(x1, y1, x2, y2, x3, y3, px, py)
	-- BP NOTE: (or1 * or2 <= 0) to fix invalid results for points collinear with bisectors
	local or1 = M.orientation((x2 + x3) * 0.5, (y2 + y3) * 0.5, x1, y1, px, py)
	local or2 = M.orientation((x1 + x3) * 0.5, (y1 + y3) * 0.5, x2, y2, px, py)

	if M._differing_orientation(x1, y1, x2, y2, px, py, x3, y3) and (or1 * or2 <= 0) then
		return M.closest_point_on_segment_from_point(x1, y1, x2, y2, px, py)
	end

	local or3 = M.orientation((x1 + x2) * 0.5, (y1 + y2) * 0.5, x3, y3, px, py)
	if M._differing_orientation(x2, y2, x3, y3, px, py, x1, y1) and (or2 * or3 <= 0) then
		return M.closest_point_on_segment_from_point(x2, y2, x3, y3, px, py)
	end

	if M._differing_orientation(x3, y3, x1, y1, px, py, x2, y2) and (or3 * or1 <= 0) then
		return M.closest_point_on_segment_from_point(x3, y3, x1, y1, px, py)
	end

	return px, py
end




---Return the closest point on the AABB outline to (px,py)
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param px number
---@param py number
---@return number,number
function M.closest_point_on_aabb_from_point(min_x, min_y, max_x, max_y, px, py)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	if px <= min_x then
		if py <= min_y then
			return min_x, min_y
		elseif py >= max_y then
			return min_x, max_y
		else
			return min_x, py
		end
	elseif px >= max_x then
		if py <= min_y then
			return max_x, min_y
		elseif py >= max_y then
			return max_x, max_y
		else
			return max_x, py
		end
	elseif py <= min_y then
		return px, min_y
	elseif py >= max_y then
		return px, max_y
	else
		local dx1, dy1 = px - min_x, py - min_y
		local dx2, dy2 = max_x - px, max_y - py
		if dx1 < dx2 and dx1 < dy1 and dx1 < dy2 then
			return min_x, py
		elseif dx2 < dx1 and dx2 < dy1 and dx2 < dy2 then
			return max_x, py
		elseif dy1 < dy2 then
			return px, min_y
		else
			return px, max_y
		end
	end
end

---Return the furthest point on the AABB from px, py
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param px number
---@param py number
---@return number,number
local function furthest_point_on_aabb_from_point(min_x, min_y, max_x, max_y, px, py)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	local mid_x, mid_y = M.centroid_aabb(min_x, min_y, max_x, max_y)
	if px <= mid_x then
		if py <= mid_y then
			return max_x, max_y
		else
			return max_x, min_y
		end
	else
		if py <= mid_y then
			return min_x, max_y
		else
			return min_x, min_y
		end
	end
end

---@param c1_x number
---@param c1_y number
---@param c1_r number
---@param c2_x number
---@param c2_y number
---@param c2_r number
---@return number, number
function M.closest_point_on_circle_from_circle(c1_x, c1_y, c1_r, c2_x, c2_y, c2_r)
	local px, py = M.closest_point_on_circle_from_point(c2_x, c2_y, c2_r, c1_x, c1_y)
	return M.closest_point_on_circle_from_point(c1_x, c1_y, c1_r, px, py)
end


---@param c_x number
---@param c_y number
---@param c_r number
---@param px number
---@param py number
---@return number,number
function M.closest_point_on_circle_from_point(c_x, c_y, c_r, px, py)
	local dx, dy = px - c_x, py - c_y
	local ratio = c_r / math_sqrt(dx * dx + dy * dy)
	return c_x + ratio * dx, c_y + ratio * dy
end

---@param c_x number
---@param c_y number
---@param c_r number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number
function M.closest_point_on_circle_from_segment(c_x, c_y, c_r, x1, y1, x2, y2)
	local nx, ny = M.closest_point_on_segment_from_point(x1, y1, x2, y2, c_x, c_y)
	local sq_dist = M.sq_distance(c_x, c_y, nx, ny)
	local ratio = c_r / math_sqrt(sq_dist)
	local px = c_x + ratio * (nx - c_x)
	local py = c_y + ratio * (ny - c_y)
	return px, py
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param px number
---@param py number
---@return number,number
function M.closest_point_on_line_from_point(x1, y1, x2, y2, px, py)
	local dx, dy = x2 - x1, y2 - y1
	local dp_x, dp_y = px - x1, py - y1
	local c1 = dx * dp_x + dy * dp_y
	local c2 = dx * dx + dy * dy
	local ratio = c1 / c2
	return x1 + ratio * dx, y1 + ratio * dy
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param px number
---@param py number
function M.closest_point_on_quad_from_point(x1, y1, x2, y2, x3, y3, x4, y4, px, py)
	local nx, ny = M.closest_point_on_segment_from_point(x1, y1, x2, y2, px, py)
	local min_dist = M.distance(nx, ny, px, py)

	local tx, ty = M.closest_point_on_segment_from_point(x2, y2, x3, y3, px, py)
	local temp_dist = M.distance(tx, ty, px, py)

	if min_dist > temp_dist then
		min_dist = temp_dist
		nx = tx
		ny = ty
	end

	tx, ty = M.closest_point_on_segment_from_point(x3, y3, x4, y4, px, py)
	temp_dist = M.distance(tx, ty, px, py)

	if min_dist > temp_dist then
		min_dist = temp_dist
		nx = tx
		ny = ty
	end

	tx, ty = M.closest_point_on_segment_from_point(x4, y4, x1, y1, px, py)
	temp_dist = M.distance(tx, ty, px, py)

	if min_dist > temp_dist then
		nx = tx
		ny = ty
	end

	return nx, ny
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param px number
---@param py number
---@return number,number
function M.closest_point_on_ray_from_point(ray_x, ray_y, ray_dx, ray_dy, px, py)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	local t = M.dot_product(ray_dx, ray_dy, px - ray_x, py - ray_y)
	if t < 0.0 then
		return ray_x, ray_y
	else
		return ray_x + ray_dx * t, ray_y + ray_dy * t
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param px number
---@param py number
---@return number,number
function M.closest_point_on_segment_from_point(x1, y1, x2, y2, px, py)
	local vx, vy = x2 - x1, y2 - y1
	local wx, wy = px - x1, py - y1

	local c1 = vx * wx + vy * wy

	if c1 <= 0.0 then
		return x1, y1
	end

	local c2 = vx * vx + vy * vy

	if c2 <= c1 then
		return x2, y2
	end

	local ratio = c1 / c2
	local nx = x1 + ratio * vx
	local ny = y1 + ratio * vy
	return nx, ny
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param px number
---@param py number
---@return number,number,number,number
function M.closest_and_furthest_points_on_segment_from_point(x1, y1, x2, y2, px, py)
	local vx, vy = x2 - x1, y2 - y1
	local wx, wy = px - x1, py - y1

	local c1 = vx * wx + vy * wy

	if c1 <= 0.0 then
		return x1, y1, x2, y2
	end

	local c2 = vx * vx + vy * vy

	if c2 <= c1 then
		return x2, y2, x1, x2
	end

	local ratio = c1 / c2
	local nx = x1 + ratio * vx
	local ny = y1 + ratio * vy
	if ratio < 0.5 then
		return nx, ny, x2, y2
	else
		return nx, ny, x1, y1
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param px number
---@param py number
---@return number,number
function M.closest_point_on_triangle_from_point(x1, y1, x2, y2, x3, y3, px, py)
	-- BP NOTE: (or1 * or2 <= 0) to fix invalid results for points collinear with bisectors
	local or1 = M.orientation((x2 + x3) * 0.5, (y2 + y3) * 0.5, x1, y1, px, py)
	local or2 = M.orientation((x1 + x3) * 0.5, (y1 + y3) * 0.5, x2, y2, px, py)

	if M._differing_orientation(x1, y1, x2, y2, px, py, x3, y3) and (or1 * or2 <= 0) then
		return M.closest_point_on_segment_from_point(x1, y1, x2, y2, px, py)
	end

	local or3 = M.orientation((x1 + x2) * 0.5, (y1 + y2) * 0.5, x3, y3, px, py)
	if M._differing_orientation(x2, y2, x3, y3, px, py, x1, y1) and (or2 * or3 <= 0) then
		return M.closest_point_on_segment_from_point(x2, y2, x3, y3, px, py)
	end

	if M._differing_orientation(x3, y3, x1, y1, px, py, x2, y2) and (or3 * or1 <= 0) then
		return M.closest_point_on_segment_from_point(x3, y3, x1, y1, px, py)
	end

	-- Point is within the triangle, need to compute distance to each edge
	local ax, ay = M.closest_point_on_segment_from_point(x1, y1, x2, y2, px, py)
	local bx, by = M.closest_point_on_segment_from_point(x2, y2, x3, y3, px, py)
	local cx, cy = M.closest_point_on_segment_from_point(x3, y3, x1, y1, px, py)

	local adist = M.sq_distance(ax, ay, px, py)
	local bdist = M.sq_distance(bx, by, px, py)
	local cdist = M.sq_distance(cx, cy, px, py)

	if adist < bdist then
		if adist < cdist then
			return ax, ay
		else
			return cx, cy
		end
	else
		if bdist < cdist then
			return bx, by
		else
			return cx, cy
		end
	end
end

---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@param px number
---@param py number
---@return number, number
function M.closest_point_on_pcurve_from_point(point_count, pcurve, px, py)
	local min_dist = math_huge
	local nx, ny = 0.0, 0.0
	local t = 1.0 / (point_count - 1)
	local prev_x, prev_y = pcurve(0)
	for i = 1, point_count - 1 do
		local x, y = pcurve(t * i)
		local tx, ty = M.closest_point_on_segment_from_point(prev_x, prev_y, x, y, px, py)
		prev_x, prev_y = x, y
		local dist = M.sq_distance(tx, ty, px, py)
		if min_dist > dist then
			min_dist = dist
			nx, ny = tx, ty
		end
	end
	return nx, ny
end

---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param px number
---@param py number
---@return number, number
function M.closest_point_on_polygon_from_point(polygon, px, py)
	local min_dist = math_huge
	local nx, ny = 0.0, 0.0
	local npolygon = #polygon
	for i = 1, npolygon - 1, 2 do
		local x1, y1 = polygon[i], polygon[i + 1]
		local x2, y2 = polygon[1 + (i + 1) % npolygon], polygon[1 + (i + 2) % npolygon]
		local tx, ty = M.closest_point_on_segment_from_point(x1, y1, x2, y2, px, py)
		local dist = M.sq_distance(tx, ty, px, py)
		if min_dist > dist then
			min_dist = dist
			nx, ny = tx, ty
		end
	end
	return nx, ny
end

---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param px number
---@param py number
---@return number, number
function M.closest_point_on_polyline_from_point(polyline, px, py)
	local min_dist = math_huge
	local nx, ny = 0.0, 0.0
	local npolyline = #polyline / 2
	for i = 1, npolyline - 1 do
		local x1, y1 = polyline[2*(i)-1], polyline[2*(i)]
		local x2, y2 = polyline[2*(i + 1)-1], polyline[2*(i + 1)]
		local tx, ty = M.closest_point_on_segment_from_point(x1, y1, x2, y2, px, py)
		local dist = M.sq_distance(tx, ty, px, py)
		if min_dist > dist then
			min_dist = dist
			nx, ny = tx, ty
		end
	end
	return nx, ny
end

-----------------------------------------------------------
-- Minimum Distance
-----------------------------------------------------------

---@param px number
---@param py number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number
---@package
---@private
function M._minimum_distance_from_point_to_line(px, py, x1, y1, x2, y2)
	local nx, ny = M.closest_point_on_line_from_point(x1, y1, x2, y2, px, py)
	return M.distance(px, py, nx, ny)
end

-----------------------------------------------------------
-- Angles
-----------------------------------------------------------

---Normalize angle in degrees to range [0,360]
---@param angle degrees
function M.normalize_angle(angle)
	if angle > 360.0 then
		angle = angle - (math_floor(angle / 360.0) * 360.0)
	elseif angle < 0.0 then
		while angle < 0.0 do
			angle = angle + 360.0
		end
	end
	return angle
end

---Reflect angle
---@param angle degrees
function M.vertical_mirror(angle)
	if is_equal(angle, 0.0) or is_equal(angle, 180.0) or is_equal(angle, 360.0) then
		return angle
	end
	return 360.0 - angle
end

---Reflect angle
---@param angle degrees
function M.horizontal_mirror(angle)
	if angle <= 180.0 then
		return 180.0 - angle
	else
		return 540.0 - angle
	end
end

---Return the quadrant of an angle
---```
--- 2|1
--- -+-
--- 3|4
---```
---Note: The quadrant is 0 for angles outside of range [0,360]
---@param angle degrees
---@return integer quadrant
function M.quadrant_by_angle(angle)
	if angle >= 0.0 and angle < 90.0 then
		return 1
	elseif angle >= 90.0 and angle < 180.0 then
		return 2
	elseif angle >= 180.0 and angle < 270.0 then
		return 3
	elseif angle >= 270.0 and angle < 360.0 then
		return 4
	elseif angle == 360.0 then
		return 1
	else
		return 0
	end
end

---Return the quadrant of a point
---```
--- 2|1
--- -+-
--- 3|4
---```
---Note: The quadrant is 0 for point (0,0)
---@param x number
---@param y number
---@return integer quadrant
function M.quadrant_by_point(x, y)
	if x > 0.0 and y >= 0.0 then
		return 1
	elseif x <= 0.0 and y > 0.0 then
		return 2
	elseif x < 0.0 and y <= 0.0 then
		return 3
	elseif x >= 0.0 and y < 0.0 then
		return 4
	else
		return 0
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number degrees
function M.vertex_angle(x1, y1, x2, y2, x3, y3)
	-- Quantify coordinates
	local x1_ = x1 - x2
	local x3_ = x3 - x2
	local y1_ = y1 - y2
	local y3_ = y3 - y2

	local dist = (x1_ * x1_ + y1_ * y1_) * (x3_ * x3_ + y3_ * y3_)

	if dist == 0.0 then
		return 0.0
	else
		local input_term = (x1_ * x3_ + y1_ * y3_) / math_sqrt(dist)

		if input_term == 1.0 then
			return 0.0
		elseif input_term == -1.0 then
			return 180.0
		else
			return math_acos(input_term) * _180_DIV_PI
		end
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param orient integer
---@return number degrees
---@package
---@private
function M._oriented_vertex_angle(x1, y1, x2, y2, x3, y3, orient)
	if M.orientation(x1, y1, x2, y2, x3, y3) ~= orient then
		return 360.0 - M.vertex_angle(x1, y1, x2, y2, x3, y3)
	else
		return M.vertex_angle(x1, y1, x2, y2, x3, y3)
	end
end

---@param x number
---@param y number
---@return number degrees
function M.cartesian_angle(x, y)
	if (x > 0.0 and y > 0.0) then
		return math_atan(y / x) * _180_DIV_PI
	elseif (x < 0.0 and y > 0.0) then
		return math_atan(-x / y) * _180_DIV_PI + 90.0
	elseif (x < 0.0 and y < 0.0) then
		return math_atan(y / x) * _180_DIV_PI + 180.0
	elseif (x > 0.0 and y < 0.0) then
		return math_atan(-x / y) * _180_DIV_PI + 270.0
	elseif (x == 0.0 and y > 0.0) then
		return 90.0
	elseif (x < 0.0 and y == 0.0) then
		return 180.0
	elseif (x == 0.0 and y < 0.0) then
		return 270.0
	else
		return 0.0
	end
end

---@param x number
---@param y number
---@return number degrees
local function robust_cartesian_angle(x, y)
	if ((x > 0.0) and (y > 0.0)) then
		return math_atan(y / x) * _180_DIV_PI
	elseif ((x < 0.0) and (y > 0.0)) then
		return math_atan(-x / y) * _180_DIV_PI + 90.0
	elseif ((x < 0.0) and (y < 0.0)) then
		return math_atan(y / x) * _180_DIV_PI + 180.0
	elseif ((x > 0.0) and (y < 0.0)) then
		return math_atan(-x / y) * _180_DIV_PI + 270.0
	elseif (is_equal(x, 0.0) and (y > 0.0)) then
		return 90.0
	elseif ((x < 0.0) and is_equal(y, 0.0)) then
		return 180.0
	elseif (is_equal(x, 0.0) and (y < 0.0)) then
		return 270.0
	else
		return 0.0
	end
end

-----------------------------------------------------------
-- Point in shape tests
-----------------------------------------------------------

---Checks whether (px,py) is inside the circle defined by the three points
---@param px number
---@param py number
---@param cx number
---@param cy number
---@param cr number
---@return boolean
function M.point_in_circle(px, py, cx, cy, cr)
	return less_than_or_equal(M.sq_distance(px, py, cx, cy), (cr * cr))
end

---@param px number
---@param py number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
function M.point_in_circumcircle(px, py, x1, y1, x2, y2, x3, y3)
	return M._point_test_circumcircle(x1, y1, x2, y2, x3, y3, px, py) >= 0
end

---Test if a point is in the "focus area" of a triangle
---
---Example: `a` is in the focus area, `b` is not
---```txt
---    /1
---   /   a
---  3
---   \
--- b  \2
---```
---@param px number
---@param py number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
function M.point_in_focus_area(px, py, x1, y1, x2, y2, x3, y3)
	local or1 = M.orientation((x2 + x3) * 0.5, (y2 + y3) * 0.5, x1, y1, px, py)
	local or2 = M.orientation((x1 + x3) * 0.5, (y1 + y3) * 0.5, x2, y2, px, py)
	return -1 == (or1 * or2)
end

---@param px number
---@param py number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
function M.point_in_quad(px, py, x1, y1, x2, y2, x3, y3, x4, y4)
	local or1 = M.orientation(x1, y1, x2, y2, px, py)
	local or2 = M.orientation(x2, y2, x3, y3, px, py)
	local or3 = M.orientation(x3, y3, x4, y4, px, py)
	local or4 = M.orientation(x4, y4, x1, y1, px, py)

	if (or1 == or2) and (or2 == or3) and (or3 == or4) then
		return true
	elseif 0 == or1 then
		return (0 == (or2 * or4))
	elseif 0 == or2 then
		return (0 == (or1 * or3))
	elseif 0 == or3 then
		return (0 == (or2 * or4))
	elseif 0 == or4 then
		return (0 == (or1 * or3))
	else
		return false
	end
end

---@param px number
---@param py number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.point_in_aabb(px, py, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return (less_than_or_equal(min_x, px) and less_than_or_equal(px, max_x) and less_than_or_equal(min_y, py) and less_than_or_equal(py, max_y))
end

---@param px number
---@param py number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
function M.point_in_polygon(px, py, polygon)
	local npolygon = #polygon
	local inside = false
	for i = 1, npolygon, 2 do
		local x1, y1 = polygon[i], polygon[i + 1]
		local x2, y2 = polygon[1 + (i + 1) % npolygon], polygon[1 + (i + 2) % npolygon]
		if ((y1 < py and y2 >= py) or (y2 < py and y1 >= py)) and (x1 <= px or x2 <= px) then
			local intersect = x1 + (py - y1) / (y2 - y1) * (x2 - x1)
			if intersect < px then
				inside = not inside
			end
		end
	end
	return inside
end

---@param px number
---@param py number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
function M.point_in_triangle(px, py, x1, y1, x2, y2, x3, y3)
	local or1 = M.orientation(x1, y1, x2, y2, px, py)
	local or2 = M.orientation(x2, y2, x3, y3, px, py)

	if (or1 * or2) == -1 then
		return false
	else
		local or3 = M.orientation(x3, y3, x1, y1, px, py)
		if or1 == or3 or or3 == 0 then
			return true
		elseif or1 == 0 then
			return (or2 * or3) >= 0
		elseif or2 == 0 then
			return (or1 * or3) >= 0
		else
			return false
		end
	end
end

function M.point_on_circle(px, py, cx, cy, radius)
	return is_equal(M.sq_distance(px, py, cx, cy), radius * radius)
end

function M.point_on_quad(px, py, x1, y1, x2, y2, x3, y3, x4, y4)
	return M.is_point_collinear(x1, y1, x2, y2, px, py, true) or
		M.is_point_collinear(x2, y2, x3, y3, px, py, true) or
		M.is_point_collinear(x3, y3, x4, y4, px, py, true) or
		M.is_point_collinear(x4, y4, x1, y1, px, py, true)
end

---If the point (px,py) is on the ray (ray_x,ray_y)->(ray_dx,ray_dy), return the distance from origin to point
---@param px number
---@param py number
---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@return number?
---@package
---@private
function M._point_on_ray_distance(px, py, ray_x, ray_y, ray_dx, ray_dy)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	local t = M.dot_product(ray_dx, ray_dy, px - ray_x, py - ray_y)
	if greater_than_or_equal(t, 0) then
		local rx, ry = ray_x + t * ray_dx, ray_y + t * ray_dy
		if is_equal(px, rx) and is_equal(py, ry) then
			return t
		end
	end
end

---@param px number
---@param py number
---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@return boolean
function M.point_on_ray(px, py, ray_x, ray_y, ray_dx, ray_dy)
	return M._point_on_ray_distance(px, py, ray_x, ray_y, ray_dx, ray_dy) ~= nil
end

---@param px number
---@param py number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return boolean
function M.point_on_line(px, py, x1, y1, x2, y2)
	local x, y = M.closest_point_on_line_from_point(x1, y1, x2, y2, px, py)
	return is_equal(px, x) and is_equal(py, y)
end

---@param px number
---@param py number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.point_on_aabb(px, py, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return ((less_than_or_equal(min_x, px) and less_than_or_equal(px, max_x)) and (py == min_y or py == max_y)) or
		((less_than_or_equal(min_y, py) and less_than_or_equal(py, max_y)) and (px == min_x or px == max_x))
end

---@param px number
---@param py number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
function M.point_on_segment(px, py, x1, y1, x2, y2)
	return M.is_point_collinear(x1, y1, x2, y2, px, py, true)
end

---@param px number
---@param py number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
function M.point_on_triangle(px, py, x1, y1, x2, y2, x3, y3)
	return M.is_point_collinear(x1, y1, x2, y2, px, py, true) or
		M.is_point_collinear(x2, y2, x3, y3, px, py, true) or
		M.is_point_collinear(x3, y3, x1, y1, px, py, true)
end

---Checks whether (px,py) is inside, outside, or on the circle circumscribed by the three points (x1,y1), (x2,y2), (x3,y3)
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param px number
---@param py number
---@return integer status (1 = inside, -1 = outside, 0 = cocircular)
---@package
---@private
function M._point_test_circumcircle(x1, y1, x2, y2, x3, y3, px, py)
	-- fix order of points to be counter clockwise
	if M.orientation(x1, y1, x2, y2, x3, y3) < 0 then
		x1, y1, x3, y3 = x3, y3, x1, y1
	end
	
	local dx1, dx2, dx3 = x1 - px, x2 - px, x3 - px
	local dy1, dy2, dy3 = y1 - py, y2 - py, y3 - py
	local det1 = dx1 * dy2 - dx2 * dy1
	local det2 = dx2 * dy3 - dx3 * dy2
	local det3 = dx3 * dy1 - dx1 * dy3
	local lift1 = dx1 * dx1 + dy1 * dy1
	local lift2 = dx2 * dx2 + dy2 * dy2
	local lift3 = dx3 * dx3 + dy3 * dy3
	local result = lift1 * det2 + lift2 * det3 + lift3 * det1
	if is_equal(result, 0) then
		return 0
	elseif (result > 0) then
		return 1
	else
		return -1
	end
end

-----------------------------------------------------------
-- Intersection / Overlap tests
-----------------------------------------------------------

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number?,number?,number?,number?,number?,number?
local function intersect_segment_helper(x1, y1, x2, y2, x3, y3, x4, y4)
	local ax, bx = x2 - x1, x3 - x4

	do
		-- Check x-axis overlap
		local lowerx, upperx

		if ax < 0.0 then
			lowerx, upperx = x2, x1
		else
			lowerx, upperx = x1, x2
		end

		if bx > 0.0 then
			if upperx < x4 or x3 < lowerx then
				return
			end
		elseif upperx < x3 or x4 < lowerx then
			return
		end
	end

	local ay, by = y2 - y1, y3 - y4

	do
		-- Check y-axis overlap
		local lowery, uppery

		if ay < 0.0 then
			lowery, uppery = y2, y1
		else
			lowery, uppery = y1, y2
		end

		if by > 0.0 then
			if uppery < y4 or y3 < lowery then
				return
			end
		elseif uppery < y3 or y4 < lowery then
			return
		end
	end

	local cx, cy = x1 - x3, y1 - y3

	do
		local d = (by * cx) - (bx * cy)
		local f = (ay * bx) - (ax * by)
		
		if not_equal(d, f) then
			if f > 0.0 then
				if d < 0.0 or d > f then
					return
				end
			elseif d > 0.0 or d < f then
				return
			end
		end

		local e = (ax * cy) - (ay * cx)
		if not_equal(e, f) then
			if f > 0.0 then
				if e < 0.0 or e > f then
					return
				end
			elseif e > 0.0 or e < f then
				return
			end
		end
	end

	return ax, ay, bx, by, cx, cy
end

---@param a_min_x number
---@param a_min_y number
---@param a_max_x number
---@param a_max_y number
---@param b_min_x number
---@param b_min_y number
---@param b_max_x number
---@param b_max_y number
---@param outline_only? boolean
function M.intersect_aabb_aabb(a_min_x, a_min_y, a_max_x, a_max_y, b_min_x, b_min_y, b_max_x, b_max_y, outline_only)
	_assert(a_min_x <= a_max_x and a_min_y <= a_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", a_min_x, a_max_x, a_min_y, a_max_y)
	_assert(b_min_x <= b_max_x and b_min_y <= b_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", b_min_x, b_max_x, b_min_y, b_max_y)

	local overlap = less_than_or_equal(a_min_x, b_max_x) and greater_than_or_equal(a_max_x, b_min_x) and less_than_or_equal(a_min_y, b_max_y) and greater_than_or_equal(a_max_y, b_min_y)
	if not outline_only then
		return overlap
	elseif overlap then
		-- Discard the cases where either aabb is fully included in the other
		if greater_than_or_equal(a_min_x, b_min_x) and less_than_or_equal(a_max_x, b_max_x) and greater_than_or_equal(a_min_y, b_min_y) and less_than_or_equal(a_max_y, b_max_y) then
			return false
		elseif less_than_or_equal(a_min_x, b_min_x) and greater_than_or_equal(a_max_x, b_max_x) and less_than_or_equal(a_min_y, b_min_y) and greater_than_or_equal(a_max_y, b_max_y) then
			return false
		else
			return true
		end
	else
		return false
	end
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param c_x number
---@param c_y number
---@param c_r number
---@param outline_only? boolean
function M.intersect_aabb_circle(min_x, min_y, max_x, max_y, c_x, c_y, c_r, outline_only)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)

	local closest_x, closest_y = M.closest_point_on_aabb_from_point(min_x, min_y, max_x, max_y, c_x, c_y)
	local dist_sq = M.sq_distance(closest_x, closest_y, c_x, c_y)
	local aabb_within_or_overlap = less_than_or_equal(dist_sq, c_r*c_r)
	if outline_only then
		if aabb_within_or_overlap then
			-- Determine if aabb is completely within circle and return false if so
			local furthest_x, furthest_y = furthest_point_on_aabb_from_point(min_x, min_y, max_x, max_y, c_x, c_y)
			local fdist_sq = M.sq_distance(furthest_x, furthest_y, c_x, c_y)
			return not less_than_or_equal(fdist_sq, c_r*c_r)
		else
			return false
		end
	elseif not aabb_within_or_overlap then
		local px, py = M.closest_point_in_aabb_from_point(min_x, min_y, max_x, max_y, c_x, c_y)
		local pdist_sq = M.sq_distance(px, py, c_x, c_y)
		return less_than_or_equal(pdist_sq, c_r*c_r)
	else
		return true
	end
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return boolean
function M.intersect_aabb_line(min_x, min_y, max_x, max_y, x1, y1, x2, y2)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)

	-- Check orientation of each corner of the aabb
	local or1 = M.orientation(x1, y1, x2, y2, min_x, min_y)
	local or2 = M.orientation(x1, y1, x2, y2, max_x, min_y)
	local or3 = M.orientation(x1, y1, x2, y2, max_x, max_y)
	local or4 = M.orientation(x1, y1, x2, y2, min_x, max_y)

	-- If all corners have the same orientation, the line does not intersect the aabb
	if or1 == or2 and or2 == or3 and or3 == or4 then
		return false
	end

	return true
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
---@return boolean
function M.intersect_aabb_polyline(min_x, min_y, max_x, max_y, polyline, outline_only)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return M._intersect_polyline(polyline, M.intersect_segment_aabb, min_x, min_y, max_x, max_y, outline_only)
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
---@return boolean
function M.intersect_aabb_polygon(min_x, min_y, max_x, max_y, polygon, outline_only)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	if not outline_only and M._polygon_contains_points(polygon, {min_x, min_y, min_x, max_y, max_x, max_y, max_x, min_y}) then
		return true
	end
	return M._intersect_polygon_segments(polygon, M.intersect_segment_aabb, min_x, min_y, max_x, max_y, outline_only)
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param pcurve fun(t:number):number,number
---@param outline_only? boolean
---@return boolean
function M.intersect_aabb_pcurve(min_x, min_y, max_x, max_y, point_count, pcurve, outline_only)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return M._intersect_pcurve(point_count, pcurve, M.intersect_segment_aabb, min_x, min_y, max_x, max_y, outline_only)
end

---@param c1_x number
---@param c1_y number
---@param c1_r number
---@param c2_x number
---@param c2_y number
---@param c2_r number
---@param outline_only? boolean
function M.intersect_circle_circle(c1_x, c1_y, c1_r, c2_x, c2_y, c2_r, outline_only)
	local sq_dist = M.sq_distance(c1_x, c1_y, c2_x, c2_y)
	local overlap = less_than_or_equal(sq_dist, (c1_r + c2_r) * (c1_r + c2_r))
	if not outline_only then
		return overlap
	elseif not overlap then
		return false
	else
		-- project circles onto axis connecting their centres then check the radius overlap
		local dx, dy = M.normalize(c2_x - c1_x, c2_y - c1_y)
		local c1_proj = M.dot_product(dx, dy, c1_x, c1_y)
		local c2_proj = M.dot_product(dx, dy, c2_x, c2_y)
		local c1_min, c1_max = c1_proj - c1_r, c1_proj + c1_r
		local c2_min, c2_max = c2_proj - c2_r, c2_proj + c2_r
		return (less_than_or_equal(c2_min, c1_max) and greater_than_or_equal(c2_max, c1_max) and greater_than_or_equal(c2_min, c1_min)) or
			(greater_than_or_equal(c1_min, c2_min) and less_than_or_equal(c1_min, c2_max) and less_than_or_equal(c2_max, c1_max))
	end
end

---@param c_x number
---@param c_y number
---@param c_r number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@param outline_only? boolean
---@return boolean
function M.intersect_circle_pcurve(c_x, c_y, c_r, point_count, pcurve, outline_only)
	local intersector = function(sx1, sy1, sx2, sy2)
		return M.intersect_segment_circle(sx1, sy1, sx2, sy2, c_x, c_y, c_r, outline_only)
	end
	return M._intersect_pcurve(point_count, pcurve, intersector)
end

---@param c_x number
---@param c_y number
---@param c_r number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_circle_polyline(c_x, c_y, c_r, polyline, outline_only)
	return M._intersect_polyline(polyline, M.intersect_segment_circle, c_x, c_y, c_r, outline_only)
end

---@param c_x number
---@param c_y number
---@param c_r number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_circle_polygon(c_x, c_y, c_r, polygon, outline_only)
	if not outline_only and M.point_in_polygon(c_x, c_y, polygon) then
		return true
	end
	return M._intersect_polygon_segments(polygon, M.intersect_segment_circle, c_x, c_y, c_r, outline_only)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param c_x number
---@param c_y number
---@param c_r number
function M.intersect_line_circle(x1, y1, x2, y2, c_x, c_y, c_r)
	local rx1, ry1 = x1 - c_x, y1 - c_y
	local rx2, ry2 = x2 - c_x, y2 - c_y
	return greater_than_or_equal(((c_r * c_r) * M.sq_distance(rx1, ry1, rx2, ry2) - ((rx1 * ry2) - (rx2 * ry1)) ^ 2), 0)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param pcurve fun(t:number):number,number
---@return boolean
function M.intersect_line_pcurve(x1, y1, x2, y2, point_count, pcurve)
	local intersector = function(sx1, sy1, sx2, sy2)
		return M.intersect_segment_line(sx1, sy1, sx2, sy2, x1, y1, x2, y2)
	end
	return M._intersect_pcurve(point_count, pcurve, intersector)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
function M.intersect_line_polyline(x1, y1, x2, y2, polyline)
	return M._intersect_polyline(polyline, M.intersect_segment_line, x1, y1, x2, y2)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
function M.intersect_line_polygon(x1, y1, x2, y2, polygon)
	return M._intersect_polygon_segments(polygon, M.intersect_segment_line, x1, y1, x2, y2)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
function M.intersect_line_line(x1, y1, x2, y2, x3, y3, x4, y4)
	if not_equal((x1 - x2) * (y3 - y4), (y1 - y2) * (x3 - x4)) then
		return true
	elseif M.collinear(x1, y1, x2, y2, x3, y3) then
		return true
	else
		return false
	end
end

---@param line_x1 number
---@param line_y1 number
---@param line_x2 number
---@param line_y2 number
---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
function M.intersect_line_quad(line_x1, line_y1, line_x2, line_y2, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)
	local or1 = M.orientation(line_x1, line_y1, line_x2, line_y2, quad_x1, quad_y1)
	if or1 == 0 then return true end

	local or2 = M.orientation(line_x1, line_y1, line_x2, line_y2, quad_x2, quad_y2)
	if or2 ~= or1 then return true end

	or2 = M.orientation(line_x1, line_y1, line_x2, line_y2, quad_x3, quad_y3)
	if or2 ~= or1 then return true end

	or2 = M.orientation(line_x1, line_y1, line_x2, line_y2, quad_x4, quad_y4)
	return or2 ~= or1
end

---@param line_x1 number
---@param line_y1 number
---@param line_x2 number
---@param line_y2 number
---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
function M.intersect_line_triangle(line_x1, line_y1, line_x2, line_y2, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3)
	local or1 = M.orientation(line_x1, line_y1, line_x2, line_y2, tri_x1, tri_y1)
	if or1 == 0 then return true end

	local or2 = M.orientation(line_x1, line_y1, line_x2, line_y2, tri_x2, tri_y2)
	if or2 ~= or1 then return true end

	or2 = M.orientation(line_x1, line_y1, line_x2, line_y2, tri_x3, tri_y3)
	return or2 ~= or1
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param c_x number
---@param c_y number
---@param c_r number
function M.intersect_ray_circle(ray_x, ray_y, ray_dx, ray_dy, c_x, c_y, c_r)
	local dx = ray_x - c_x
	local dy = ray_y - c_y
	local c = dx * dx + dy * dy - c_r * c_r
	if c <= 0 then
		return true
	end
	local b = dx * ray_dx + dy * ray_dy
	if b >= 0 then
		return false
	end
	return b * b >= c
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
function M.intersect_ray_pcurve(ray_x, ray_y, ray_dx, ray_dy, point_count, pcurve)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, sx1, sy1, sx2, sy2)
	end
	return M._intersect_pcurve(point_count, pcurve, intersector)
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
function M.intersect_ray_polyline(ray_x, ray_y, ray_dx, ray_dy, polyline)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, sx1, sy1, sx2, sy2)
	end
	return M._intersect_polyline(polyline, intersector)
end

---@param ray1_x number
---@param ray1_y number
---@param ray1_dx number
---@param ray1_dy number
---@param line_x1 number
---@param line_y1 number
---@param line_x2 number
---@param line_y2 number
function M.intersect_ray_line(ray1_x, ray1_y, ray1_dx, ray1_dy, line_x1, line_y1, line_x2, line_y2)
	local line_dx, line_dy = M.normalize(line_x2 - line_x1, line_y2 - line_y1)
	local denom = -ray1_dy * line_dx + ray1_dx * line_dy

	if denom ~= 0.0 then
		local diff_x = ray1_x - line_x1
		local diff_y = ray1_y - line_y1
		local t = (-line_dy * diff_x + line_dx * diff_y) / denom
		return greater_than_or_equal(t, 0)
	else
		return M.point_on_ray(line_x1, line_y1, ray1_x, ray1_y, ray1_dx, ray1_dy) or
			M.point_on_line(ray1_x, ray1_y, line_x1, line_y1, line_x2, line_y2)
	end
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param polygon number[] -- polygon with 3 or more vertices, as flat array of x,y coordinates [x1, y1, ..., xn, yn]
function M.intersect_ray_polygon(ray_x, ray_y, ray_dx, ray_dy, polygon)
	assert(#polygon >= 6, "Polygon must have at least 3 vertices")

	local j = #polygon - 1
	for i = 1, #polygon, 2 do
		if M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, polygon[i], polygon[i + 1], polygon[j], polygon[j + 1]) then
			return true
		end
		j = i
	end

	return false
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
function M.intersect_ray_quad(ray_x, ray_y, ray_dx, ray_dy, x1, y1, x2, y2, x3, y3, x4, y4)
	_assert(M._quad_is_valid(x1, y1, x2, y2, x3, y3, x4, y4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", x1, y1, x2, y2, x3, y3, x4, y4)

	if M.point_in_quad(ray_x, ray_y, x1, y1, x2, y2, x3, y3, x4, y4) then
		return true
	else
		return M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, x1, y1, x2, y2) or
			M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, x2, y2, x3, y3) or
			M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, x3, y3, x4, y4) or
			M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, x4, y4, x1, y1)
	end
end

---@param ray1_x number
---@param ray1_y number
---@param ray1_dx number
---@param ray1_dy number
---@param ray2_x number
---@param ray2_y number
---@param ray2_dx number
---@param ray2_dy number
function M.intersect_ray_ray(ray1_x, ray1_y, ray1_dx, ray1_dy, ray2_x, ray2_y, ray2_dx, ray2_dy)
	local denom = -ray1_dy * ray2_dx + ray1_dx * ray2_dy

	if denom ~= 0.0 then
		local diff_x = ray1_x - ray2_x
		local diff_y = ray1_y - ray2_y
		local s = (-ray1_dy * diff_x + ray1_dx * diff_y) / denom
		local t = (-ray2_dy * diff_x + ray2_dx * diff_y) / denom
		return (t >= 0.0) and (s >= 0.0)
	else
		return M.point_on_ray(ray2_x, ray2_y, ray1_x, ray1_y, ray1_dx, ray1_dy) or
			M.point_on_ray(ray1_x, ray1_y, ray2_x, ray2_y, ray2_dx, ray2_dy)
	end
end

---@param point_count1 integer
---@param pcurve1 fun(t:number):number,number
---@param point_count2 integer
---@param pcurve2 fun(t:number):number,number
function M.intersect_pcurve_pcurve(point_count1, pcurve1, point_count2, pcurve2)
	return M._intersect_pcurve(point_count1, pcurve1, M.intersect_segment_pcurve, point_count2,
		pcurve2)
end

---@param point_count1 integer
---@param pcurve1 fun(t:number):number,number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_pcurve_polygon(point_count1, pcurve1, polygon, outline_only)
	return M._intersect_pcurve(point_count1, pcurve1, M.intersect_segment_polygon, polygon, outline_only)
end

---@param polygon1 number[] -- (x1, y1, x2, y2, ...)
---@param polygon2 number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_polygon_polygon(polygon1, polygon2, outline_only)
	if not outline_only and M._polygon_contains_points(polygon1, polygon2) then
		return true
	end
	return M._intersect_polygon_segments(polygon1, M.intersect_segment_polygon, polygon2, outline_only)
end

---@param polyline1 number[] -- (x1, y1, x2, y2, ...)
---@param polyline2 number[] -- (x1, y1, x2, y2, ...)
function M.intersect_polyline_polyline(polyline1, polyline2)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersect_segment_polyline(sx1, sy1, sx2, sy2, polyline2)
	end
	return M._intersect_polyline(polyline1, intersector)
end

---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_polyline_polygon(polyline, polygon, outline_only)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersect_segment_polygon(sx1, sy1, sx2, sy2, polygon, outline_only)
	end
	return M._intersect_polyline(polyline, intersector)
end

---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param point_count integer
---@param pcurve fun(t:number):number,number
function M.intersect_polyline_pcurve(polyline, point_count, pcurve)
	return M._intersect_polyline(polyline, M.intersect_segment_pcurve, point_count, pcurve)
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.intersect_ray_aabb(ray_x, ray_y, ray_dx, ray_dy, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)

	local tmin, tmax = 0.0, math_huge

	if not_equal(ray_dx, 0) then
		local recip_dir_x = 1.0 / ray_dx

		if ray_dx > 0.0 then
			local t = (max_x - ray_x) * recip_dir_x
			if t < tmin then return false end
			tmax = math_min(t, tmax)

			t = (min_x - ray_x) * recip_dir_x
			if t > tmax then return false end
			tmin = math_max(t, tmin)
		else
			local t = (min_x - ray_x) * recip_dir_x
			if t < tmin then return false end
			tmax = math_min(t, tmax)

			t = (max_x - ray_x) * recip_dir_x
			if t > tmax then return false end
			tmin = math_max(t, tmin)
		end
	elseif (ray_x < min_x) or (ray_x > max_x) then
		return false
	end

	if not_equal(ray_dy, 0) then
		local recip_dir_y = 1.0 / ray_dy

		if ray_dy > 0.0 then
			local t = (max_y - ray_y) * recip_dir_y
			if t < tmin then return false end
			tmax = math_min(t, tmax)

			t = (min_y - ray_y) * recip_dir_y
			if t > tmax then return false end
			tmin = math_max(t, tmin)
		else
			local t = (min_y - ray_y) * recip_dir_y
			if t < tmin then return false end
			tmax = math_min(t, tmax)

			t = (max_y - ray_y) * recip_dir_y
			if t > tmax then return false end
			tmin = math_max(t, tmin)
		end
	elseif (ray_y < min_y) or (ray_y > max_y) then
		return false
	end

	return (tmin < tmax)
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
function M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, x1, y1, x2, y2)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	return M.intersection_point_ray_segment(ray_x, ray_y, ray_dx, ray_dy, x1, y1, x2, y2) ~= nil
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
function M.intersect_ray_triangle(ray_x, ray_y, ray_dx, ray_dy, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3)
	if M.point_in_triangle(ray_x, ray_y, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3) then
		return true
	else
		return M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, tri_x1, tri_y1, tri_x2, tri_y2) or
			M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, tri_x2, tri_y2, tri_x3, tri_y3) or
			M.intersect_ray_segment(ray_x, ray_y, ray_dx, ray_dy, tri_x3, tri_y3, tri_x1, tri_y1)
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param c_x number
---@param c_y number
---@param c_r number
---@param outline_only? boolean
function M.intersect_segment_circle(x1, y1, x2, y2, c_x, c_y, c_r, outline_only)
	local px, py, qx, qy = M.closest_and_furthest_points_on_segment_from_point(x1, y1, x2, y2, c_x, c_y)
	local closest_inside = less_than_or_equal(M.sq_distance(px, py, c_x, c_y), c_r * c_r)
	if not outline_only then
		return closest_inside
	elseif closest_inside then
		local furthest_inside = less_than_or_equal(M.sq_distance(qx, qy, c_x, c_y), c_r * c_r)
		return not furthest_inside
	else
		return false
	end
end

---@param seg_x1 number
---@param seg_y1 number
---@param seg_x2 number
---@param seg_y2 number
---@param line_x1 number
---@param line_y1 number
---@param line_x2 number
---@param line_y2 number
function M.intersect_segment_line(seg_x1, seg_y1, seg_x2, seg_y2, line_x1, line_y1, line_x2, line_y2)
	return (M.orientation(line_x1, line_y1, line_x2, line_y2, seg_x1, seg_y1) * M.orientation(line_x1, line_y1, line_x2, line_y2, seg_x2, seg_y2)) <= 0
end

---@param seg_x1 number
---@param seg_y1 number
---@param seg_x2 number
---@param seg_y2 number
---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
---@param outline_only? boolean
function M.intersect_segment_quad(seg_x1, seg_y1, seg_x2, seg_y2, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4, outline_only)
	if not outline_only then
		if M.point_in_quad(seg_x1, seg_y1, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4) or
		M.point_in_quad(seg_x2, seg_y2, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4) then
			return true
		end
	end

	-- Test each segment for intersection
	if M.intersect_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, quad_x1, quad_y1, quad_x2, quad_y2) or
		M.intersect_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, quad_x2, quad_y2, quad_x3, quad_y3) or
		M.intersect_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, quad_x3, quad_y3, quad_x4, quad_y4) or
		M.intersect_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, quad_x4, quad_y4, quad_x1, quad_y1) then
		return true
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param outline_only? boolean
function M.intersect_segment_aabb(x1, y1, x2, y2, min_x, min_y, max_x, max_y, outline_only)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	local bmin_x, bmin_y = math_min(x1, x2), math_min(y1, y2)
	local bmax_x, bmax_y = math_max(x1, x2), math_max(y1, y2)
	if outline_only and M.aabb_within_aabb(bmin_x, bmin_y, bmax_x, bmax_y, min_x, min_y, max_x, max_y) then
		return false
	elseif M.intersect_aabb_aabb(min_x, min_y, max_x, max_y, bmin_x, bmin_y, bmax_x, bmax_y) then
		local orien = M.orientation(x1, y1, x2, y2, min_x, min_y)
		if ((M.orientation(x1, y1, x2, y2, min_x, max_y) == orien) and
				(M.orientation(x1, y1, x2, y2, max_x, max_y) == orien) and
				(M.orientation(x1, y1, x2, y2, max_x, min_y) == orien)) then
			return false
		else
			return true
		end
	else
		return false
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
function M.intersect_segment_pcurve(x1, y1, x2, y2, point_count, pcurve)
	return M._intersect_pcurve(point_count, pcurve, M.intersect_segment_segment, x1, y1, x2, y2)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
function M.intersect_segment_polyline(x1, y1, x2, y2, polyline)
	return M._intersect_polyline(polyline, M.intersect_segment_segment, x1, y1, x2, y2)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_segment_polygon(x1, y1, x2, y2, polygon, outline_only)
	if not outline_only and M._polygon_contains_points(polygon, {x1, y1, x2, y2}) then
		return true
	end
	return M._intersect_polygon_segments(polygon, M.intersect_segment_segment, x1, y1, x2, y2)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return boolean
function M.intersect_segment_segment(x1, y1, x2, y2, x3, y3, x4, y4)
	return intersect_segment_helper(x1, y1, x2, y2, x3, y3, x4, y4) ~= nil
end

---Check segment-segment intersection (naive version)
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return boolean
function M.intersect_segment_segment_simple(x1, y1, x2, y2, x3, y3, x4, y4)
	return (
		(M.orientation(x1, y1, x2, y2, x3, y3) * M.orientation(x1, y1, x2, y2, x4, y4) <= 0) and
		(M.orientation(x3, y3, x4, y4, x1, y1) * M.orientation(x3, y3, x4, y4, x2, y2) <= 0)
	)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param t1x number
---@param t1y number
---@param t2x number
---@param t2y number
---@param t3x number
---@param t3y number
---@param outline_only? boolean
function M.intersect_segment_triangle(x1, y1, x2, y2, t1x, t1y, t2x, t2y, t3x, t3y, outline_only)
	if not outline_only then
		if M.point_in_triangle(x1, y1, t1x, t1y, t2x, t2y, t3x, t3y) or
			M.point_in_triangle(x2, y2, t1x, t1y, t2x, t2y, t3x, t3y) then
			return true
		end
	end

	-- Test each segment for intersection
	if M.intersect_segment_segment(x1, y1, x2, y2, t1x, t1y, t2x, t2y) or
		M.intersect_segment_segment(x1, y1, x2, y2, t2x, t2y, t3x, t3y) or
		M.intersect_segment_segment(x1, y1, x2, y2, t3x, t3y, t1x, t1y) then
		return true
	end
end

---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
---@param c_x number
---@param c_y number
---@param c_r number
---@param outline_only? boolean
function M.intersect_triangle_circle(tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3, c_x, c_y, c_r, outline_only)
	if not outline_only then
		local closest_x, closest_y = M.closest_point_on_triangle_from_point(tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3,
			c_x, c_y)
		return M.point_in_circle(closest_x, closest_y, c_x, c_y, c_r)
	else
		-- Test each segment for intersection
		if M.intersect_segment_circle(tri_x1, tri_y1, tri_x2, tri_y2, c_x, c_y, c_r, true) or
			M.intersect_segment_circle(tri_x2, tri_y2, tri_x3, tri_y3, c_x, c_y, c_r, true) or
			M.intersect_segment_circle(tri_x3, tri_y3, tri_x1, tri_y1, c_x, c_y, c_r, true) then
			return true
		end
	end
end

---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param outline_only? boolean
function M.intersect_triangle_aabb(tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3, min_x, min_y,
								   max_x, max_y, outline_only)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)

	local isect = M.intersect_segment_aabb(tri_x1, tri_y1, tri_x2, tri_y2, min_x, min_y, max_x, max_y) or
		M.intersect_segment_aabb(tri_x2, tri_y2, tri_x3, tri_y3, min_x, min_y, max_x, max_y) or
		M.intersect_segment_aabb(tri_x3, tri_y3, tri_x1, tri_y1, min_x, min_y, max_x, max_y)
	if outline_only then
		return isect
	else
		return isect or
			M.point_in_triangle(min_x, min_y, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3) or
			M.point_in_triangle(max_x, min_y, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3) or
			M.point_in_triangle(max_x, max_y, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3) or
			M.point_in_triangle(min_x, max_y, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3) 
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@param outline_only? boolean
function M.intersect_triangle_pcurve(x1, y1, x2, y2, x3, y3, point_count, pcurve, outline_only)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersect_segment_triangle(sx1, sy1, sx2, sy2, x1, y1, x2, y2, x3, y3, outline_only)
	end
	return M._intersect_pcurve(point_count, pcurve, intersector)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_triangle_polyline(x1, y1, x2, y2, x3, y3, polyline, outline_only)
	return M._intersect_polyline(polyline, M.intersect_segment_triangle, x1, y1, x2, y2, x3, y3, outline_only)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_triangle_polygon(x1, y1, x2, y2, x3, y3, polygon, outline_only)
	if not outline_only and M._polygon_contains_points(polygon, {x1, y1, x2, y2, x3, y3}) then
		return true
	end
	return M._intersect_polygon_segments(polygon, M.intersect_segment_triangle, x1, y1, x2, y2, x3, y3, outline_only)
end

---@param tri1_x1 number
---@param tri1_y1 number
---@param tri1_x2 number
---@param tri1_y2 number
---@param tri1_x3 number
---@param tri1_y3 number
---@param tri2_x1 number
---@param tri2_y1 number
---@param tri2_x2 number
---@param tri2_y2 number
---@param tri2_x3 number
---@param tri2_y3 number
---@param outline_only? boolean
function M.intersect_triangle_triangle(tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri1_x3, tri1_y3, tri2_x1, tri2_y1, tri2_x2, tri2_y2, tri2_x3, tri2_y3, outline_only)
	if not outline_only then
		-- Check if one vertex of t1 is contained in t2
		if M.point_in_triangle(tri1_x1, tri1_y1, tri2_x1, tri2_y1, tri2_x2, tri2_y2, tri2_x3, tri2_y3) or
			M.point_in_triangle(tri1_x2, tri1_y2, tri2_x1, tri2_y1, tri2_x2, tri2_y2, tri2_x3, tri2_y3) or
			M.point_in_triangle(tri1_x3, tri1_y3, tri2_x1, tri2_y1, tri2_x2, tri2_y2, tri2_x3, tri2_y3) then
			return true
		end
			
		-- Check if one vertex of t2 is contained in t1
		if M.point_in_triangle(tri2_x1, tri2_y1, tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri1_x3, tri1_y3) or
			M.point_in_triangle(tri2_x2, tri2_y2, tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri1_x3, tri1_y3) or
			M.point_in_triangle(tri2_x3, tri2_y3, tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri1_x3, tri1_y3) then
			return true
		end
	end

	-- Check every segment pair for intersection
	if M.intersect_segment_segment(tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri2_x1, tri2_y1, tri2_x2, tri2_y2) or
		M.intersect_segment_segment(tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri2_x2, tri2_y2, tri2_x3, tri2_y3) or
		M.intersect_segment_segment(tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri2_x3, tri2_y3, tri2_x1, tri2_y1) or

		M.intersect_segment_segment(tri1_x2, tri1_y2, tri1_x3, tri1_y3, tri2_x1, tri2_y1, tri2_x2, tri2_y2) or
		M.intersect_segment_segment(tri1_x2, tri1_y2, tri1_x3, tri1_y3, tri2_x2, tri2_y2, tri2_x3, tri2_y3) or
		M.intersect_segment_segment(tri1_x2, tri1_y2, tri1_x3, tri1_y3, tri2_x3, tri2_y3, tri2_x1, tri2_y1) or

		M.intersect_segment_segment(tri1_x3, tri1_y3, tri1_x1, tri1_y1, tri2_x1, tri2_y1, tri2_x2, tri2_y2) or
		M.intersect_segment_segment(tri1_x3, tri1_y3, tri1_x1, tri1_y1, tri2_x2, tri2_y2, tri2_x3, tri2_y3) or
		M.intersect_segment_segment(tri1_x3, tri1_y3, tri1_x1, tri1_y1, tri2_x3, tri2_y3, tri2_x1, tri2_y1) then
		return true
	end

	return false
end

---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
---@param outline_only? boolean
---@return boolean
function M.intersect_triangle_quad(tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4, outline_only)
	if not outline_only then
		return M.intersect_triangle_triangle(tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3) or
			M.intersect_triangle_triangle(tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3, quad_x1, quad_y1, quad_x3, quad_y3, quad_x4, quad_y4)
	else
		-- Test each segment for intersection
		if M.intersect_segment_quad(tri_x1, tri_y1, tri_x2, tri_y2, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4, true) or
			M.intersect_segment_quad(tri_x2, tri_y2, tri_x3, tri_y3, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4, true) or
			M.intersect_segment_quad(tri_x3, tri_y3, tri_x1, tri_y1, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4, true) then
			return true
		else
			return false
		end
	end
end

---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param outline_only? boolean
----@return boolean
function M.intersect_quad_aabb(quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4, min_x, min_y, max_x, max_y, outline_only)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	_assert(M._quad_is_valid(quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)

	if outline_only then
		return M.intersect_segment_aabb(quad_x1, quad_y1, quad_x2, quad_y2, min_x, min_y, max_x, max_y, true) or
			M.intersect_segment_aabb(quad_x2, quad_y2, quad_x3, quad_y3, min_x, min_y, max_x, max_y, true) or
			M.intersect_segment_aabb(quad_x3, quad_y3, quad_x4, quad_y4, min_x, min_y, max_x, max_y, true) or
			M.intersect_segment_aabb(quad_x4, quad_y4, quad_x1, quad_y1, min_x, min_y, max_x, max_y, true)
	else
		return M.intersect_segment_aabb(quad_x1, quad_y1, quad_x2, quad_y2, min_x, min_y, max_x, max_y) or
			M.intersect_segment_aabb(quad_x2, quad_y2, quad_x3, quad_y3, min_x, min_y, max_x, max_y) or
			M.intersect_segment_aabb(quad_x3, quad_y3, quad_x4, quad_y4, min_x, min_y, max_x, max_y) or
			M.intersect_segment_aabb(quad_x4, quad_y4, quad_x1, quad_y1, min_x, min_y, max_x, max_y) or
			M.point_in_quad(min_x, min_y, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4) or
			M.point_in_quad(max_x, min_y, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4) or
			M.point_in_quad(max_x, max_y, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4) or
			M.point_in_quad(min_x, max_y, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4) 
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param c_x number
---@param c_y number
---@param c_r number
---@param outline_only? boolean
function M.intersect_quad_circle(x1, y1, x2, y2, x3, y3, x4, y4, c_x, c_y, c_r, outline_only)
	if not outline_only then
		local px, py = M.closest_point_on_quad_from_point(x1, y1, x2, y2, x3, y3, x4, y4, c_x, c_y)
		return M.point_in_circle(px, py, c_x, c_y, c_r)
	else
		-- Test each segment for intersection
		if M.intersect_segment_circle(x1, y1, x2, y2, c_x, c_y, c_r, true) or
			M.intersect_segment_circle(x2, y2, x3, y3, c_x, c_y, c_r, true) or
			M.intersect_segment_circle(x3, y3, x4, y4, c_x, c_y, c_r, true) or
			M.intersect_segment_circle(x4, y4, x1, y1, c_x, c_y, c_r, true) then
			return true
		else
			return false
		end
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@param outline_only? boolean
function M.intersect_quad_pcurve(x1, y1, x2, y2, x3, y3, x4, y4, point_count, pcurve, outline_only)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersect_segment_quad(sx1, sy1, sx2, sy2, x1, y1, x2, y2, x3, y3, x4, y4, outline_only)
	end
	return M._intersect_pcurve(point_count, pcurve, intersector)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_quad_polyline(x1, y1, x2, y2, x3, y3, x4, y4, polyline, outline_only)
	return M._intersect_polyline(polyline, M.intersect_segment_quad, x1, y1, x2, y2, x3, y3, x4, y4, outline_only)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param outline_only? boolean
function M.intersect_quad_polygon(x1, y1, x2, y2, x3, y3, x4, y4, polygon, outline_only)
	if not outline_only and M._polygon_contains_points(polygon, {x1, y1, x2, y2, x3, y3, x4, y4}) then
		return true
	end
	return M._intersect_polygon_segments(polygon, M.intersect_segment_quad, x1, y1, x2, y2, x3, y3, x4, y4, outline_only)
end

---@param quad1_x1 number
---@param quad1_y1 number
---@param quad1_x2 number
---@param quad1_y2 number
---@param quad1_x3 number
---@param quad1_y3 number
---@param quad1_x4 number
---@param quad1_y4 number
---@param quad2_x1 number
---@param quad2_y1 number
---@param quad2_x2 number
---@param quad2_y2 number
---@param quad2_x3 number
---@param quad2_y3 number
---@param quad2_x4 number
---@param quad2_y4 number
---@param outline_only? boolean
---@return boolean
function M.intersect_quad_quad(quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4, quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4, outline_only)
	_assert(M._quad_is_valid(quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4)
	_assert(M._quad_is_valid(quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4)

	if not outline_only then
		-- Check if any quad vertex is contained in the other
		if M.point_in_quad(quad1_x1, quad1_y1, quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4) or
			M.point_in_quad(quad1_x2, quad1_y2, quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4) or
			M.point_in_quad(quad1_x3, quad1_y3, quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4) or
			M.point_in_quad(quad1_x4, quad1_y4, quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4) or

			M.point_in_quad(quad2_x1, quad2_y1, quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4) or
			M.point_in_quad(quad2_x2, quad2_y2, quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4) or
			M.point_in_quad(quad2_x3, quad2_y3, quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4) or
			M.point_in_quad(quad2_x4, quad2_y4, quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4) then
			return true
		end
	end

	-- Check if any edges overlap
	if M.intersect_segment_segment(quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad2_x1, quad2_y1, quad2_x2, quad2_y2) or
		M.intersect_segment_segment(quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad2_x2, quad2_y2, quad2_x3, quad2_y3) or
		M.intersect_segment_segment(quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4) or
		M.intersect_segment_segment(quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad2_x4, quad2_y4, quad2_x1, quad2_y1) or

		M.intersect_segment_segment(quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad2_x1, quad2_y1, quad2_x2, quad2_y2) or 
		M.intersect_segment_segment(quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad2_x2, quad2_y2, quad2_x3, quad2_y3) or
		M.intersect_segment_segment(quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad2_x3, quad2_y3, quad2_x4, quad2_y4) or
		M.intersect_segment_segment(quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad2_x4, quad2_y4, quad2_x1, quad2_y1) or

		M.intersect_segment_segment(quad1_x3, quad1_y3, quad1_x4, quad1_y4, quad2_x1, quad2_y1, quad2_x2, quad2_y2) or
		M.intersect_segment_segment(quad1_x3, quad1_y3, quad1_x4, quad1_y4, quad2_x2, quad2_y2, quad2_x3, quad2_y3) or
		M.intersect_segment_segment(quad1_x3, quad1_y3, quad1_x4, quad1_y4, quad2_x3, quad2_y3, quad2_x4, quad2_y4) or
		M.intersect_segment_segment(quad1_x3, quad1_y3, quad1_x4, quad1_y4, quad2_x4, quad2_y4, quad2_x1, quad2_y1) or

		M.intersect_segment_segment(quad1_x4, quad1_y4, quad1_x1, quad1_y1, quad2_x1, quad2_y1, quad2_x2, quad2_y2) or
		M.intersect_segment_segment(quad1_x4, quad1_y4, quad1_x1, quad1_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3) or
		M.intersect_segment_segment(quad1_x4, quad1_y4, quad1_x1, quad1_y1, quad2_x3, quad2_y3, quad2_x4, quad2_y4) or
		M.intersect_segment_segment(quad1_x4, quad1_y4, quad1_x1, quad1_y1, quad2_x4, quad2_y4, quad2_x1, quad2_y1) then
		return true
	end
		
	return false
end


-----------------------------------------------------------
-- Intersection points
-----------------------------------------------------------


---@param a_min_x number
---@param a_min_y number
---@param a_max_x number
---@param a_max_y number
---@param b_min_x number
---@param b_min_y number
---@param b_max_x number
---@param b_max_y number
---@return number?, number?, number?, number?, number?, number?, number?, number?
function M.intersection_point_aabb_aabb(a_min_x, a_min_y, a_max_x, a_max_y, b_min_x, b_min_y, b_max_x, b_max_y)
	_assert(a_min_x <= a_max_x and a_min_y <= a_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", a_min_x, a_max_x, a_min_y, a_max_y)
	_assert(b_min_x <= b_max_x and b_min_y <= b_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", b_min_x, b_max_x, b_min_y, b_max_y)

	if a_min_x == b_min_x and a_min_y == b_min_y and a_max_x == b_max_x and a_max_y == b_max_y then
		-- Rectangles are perfectly overlapping
		return a_min_x, a_min_y, a_min_x, a_max_y, a_max_x, a_max_y, a_max_x, a_min_y
	end

	if not M.intersect_aabb_aabb(a_min_x, a_min_y, a_max_x, a_max_y, b_min_x, b_min_y, b_max_x, b_max_y) then
		-- No intersection
		return
	end

	-- Swap a and b so a is to the left of b
	if a_min_x > b_min_x then
		a_min_x, b_min_x = b_min_x, a_min_x
		a_max_x, b_max_x = b_max_x, a_max_x
		a_min_y, b_min_y = b_min_y, a_min_y
		a_max_y, b_max_y = b_max_y, a_max_y
	end

	if a_min_x < b_min_x and a_min_y < b_min_y and a_max_x > b_max_x and a_max_y > b_max_y then
		-- Rectangle a entirely contains b
		return
	end

	-- Post conditions:
	-- * a_min_x <= b_min_x (horizontally sorted)
	-- * b_min_x <= a_max_x (intersecting)
	-- * either b_min_y >= a_min_y or b_max_y <= a_max_y (intersecting)

	local _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 ---@type number?,number?,number?,number?,number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
		elseif _x3 == nil or (_px == _x3 and _py == _y3) then
			_x3, _y3 = _px, _py
		elseif _x4 == nil or (_px == _x4 and _py == _y4) then
			_x4, _y4 = _px, _py
			return true
		end
	end

	if b_min_y >= a_min_y then
		if b_min_x == a_min_x then
			if _add_point(b_min_x, math_min(a_max_y, b_min_y)) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
		if b_max_x >= a_max_x then
			if _add_point(a_max_x, math_min(a_max_y, b_min_y)) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	if b_max_y <= a_max_y then
		if b_min_x == a_min_x then
			if _add_point(b_min_x, math_max(a_min_y, b_max_y)) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
		if b_max_x >= a_max_x then
			if _add_point(a_max_x, math_max(a_min_y, b_max_y)) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	if b_min_x >= a_min_x then
		if b_min_y <= a_min_y then
			if _add_point(b_min_x, math_max(a_min_y, b_min_y)) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
		if b_max_y >= a_max_y then
			if _add_point(b_min_x, math_min(a_max_y, b_max_y)) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	if b_max_x <= a_max_x then
		if b_min_y <= a_min_y then
			if _add_point(b_max_x, math_max(a_min_y, b_min_y)) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
		if b_max_y >= a_max_y then
			if _add_point(b_max_x, math_min(a_max_y, b_max_y)) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@return number[]
function M.intersection_point_aabb_pcurve(min_x, min_y, max_x, max_y, point_count, pcurve)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return M._intersection_point_pcurve(point_count, pcurve, M.intersection_point_segment_aabb, min_x, min_y,
		max_x, max_y)
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.intersection_point_aabb_polyline(min_x, min_y, max_x, max_y, polyline)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return M._intersection_point_polyline(polyline, M.intersection_point_segment_aabb, min_x, min_y, max_x, max_y)
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.intersection_point_aabb_polygon(min_x, min_y, max_x, max_y, polygon)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return M._intersection_point_polygon(polygon, M.intersection_point_segment_aabb, min_x, min_y, max_x, max_y)
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param cx number
---@param cy number
---@param radius number
---@return number?, number?, number?, number?, number?, number?, number?, number?
function M.intersection_point_aabb_circle(min_x, min_y, max_x, max_y, cx, cy, radius)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)

	local _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 ---@type number?,number?,number?,number?,number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
		elseif _x3 == nil or (_px == _x3 and _py == _y3) then
			_x3, _y3 = _px, _py
		elseif _x4 == nil or (_px == _x4 and _py == _y4) then
			_x4, _y4 = _px, _py
			return true
		end
	end

	local x1, y1, x2, y2 = M.intersection_point_segment_circle(min_x, min_y, max_x, min_y, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_circle(max_x, min_y, max_x, max_y, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_circle(max_x, max_y, min_x, max_y, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_circle(min_x, max_y, min_x, min_y, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4
end

---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number?, number?, number?, number?
function M.intersection_point_aabb_line(min_x, min_y, max_x, max_y, x1, y1, x2, y2)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)

	local _x1, _y1, _x2, _y2 ---@type number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
			return true
		end
	end

	local ix, iy = M.intersection_point_segment_line(min_x, min_y, max_x, min_y, x1, y1, x2, y2)
	if ix and iy then
		if _add_point(ix, iy) then return _x1, _y1, _x2, _y2 end
	end

	ix, iy = M.intersection_point_segment_line(max_x, min_y, max_x, max_y, x1, y1, x2, y2)
	if ix and iy then
		if _add_point(ix, iy) then return _x1, _y1, _x2, _y2 end
	end

	ix, iy = M.intersection_point_segment_line(max_x, max_y, min_x, max_y, x1, y1, x2, y2)
	if ix and iy then
		if _add_point(ix, iy) then return _x1, _y1, _x2, _y2 end
	end

	ix, iy = M.intersection_point_segment_line(min_x, max_y, min_x, min_y, x1, y1, x2, y2)
	if ix and iy then
		if _add_point(ix, iy) then return _x1, _y1, _x2, _y2 end
	end

	return _x1, _y1, _x2, _y2
end


---Calculate up to two intersection points of two circles
---Special cases:
--- * Returns nil if no intersection
--- * Returns only one point if the circles overlap exactly
---@param x1 number
---@param y1 number
---@param r1 number
---@param x2 number
---@param y2 number
---@param r2 number
---@return number?, number?, number?, number?
function M.intersection_point_circle_circle(x1, y1, r1, x2, y2, r2)
	local dist = M.distance(x1, y1, x2, y2)
	if dist > r1 + r2 then
		--- Special case: No intersection
		return
	end

	if dist < PRECISION then
		if is_equal(r1, r2) then
			--- Special case: Return single point
			return x1 + r1, y1
		else
			--- Special case: No intersection
			return
		end
	end

	local dstsqr = dist * dist
	local r1sqr = r1 * r1
	local r2sqr = r2 * r2
	local a = (dstsqr - r2sqr + r1sqr) / (2 * dist)
	local h = math_sqrt(r1sqr - (a * a))

	local ratio_a = a / dist
	local ratio_h = h / dist

	local dx = x2 - x1
	local dy = y2 - y1

	local phix = x1 + (ratio_a * dx)
	local phiy = y1 + (ratio_a * dy)

	dx = dx * ratio_h
	dy = dy * ratio_h

	if math_abs(dx) < PRECISION and math_abs(dy) < PRECISION then
		--- Special case: Circles touching at one point
		return phix + dy, phiy - dx
	end

	return phix + dy, phiy - dx, phix - dy, phiy + dx
end

---@param cx number
---@param cy number
---@param cr number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@return number[]
function M.intersection_point_circle_pcurve(cx, cy, cr, point_count, pcurve)
	return M._intersection_point_pcurve(point_count, pcurve, M.intersection_point_segment_circle, cx, cy, cr)
end

---@param cx number
---@param cy number
---@param cr number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.intersection_point_circle_polygon(cx, cy, cr, polygon)
	return M._intersection_point_polygon(polygon, M.intersection_point_segment_circle, cx, cy, cr)
end

---@param cx number
---@param cy number
---@param cr number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.intersection_point_circle_polyline(cx, cy, cr, polyline)
	return M._intersection_point_polyline(polyline, M.intersection_point_segment_circle, cx, cy, cr)
end

---@param line1_x1 number
---@param line1_y1 number
---@param line1_x2 number
---@param line1_y2 number
---@param line2_x1 number
---@param line2_y1 number
---@param line2_x2 number
---@param line2_y2 number
---@return number?, number?
function M.intersection_point_line_line(line1_x1, line1_y1, line1_x2, line1_y2, line2_x1, line2_y1, line2_x2, line2_y2)
	local ux, uy = line1_x2 - line1_x1, line1_y2 - line1_y1
	local vx, vy = line2_x2 - line2_x1, line2_y2 - line2_y1
	local wx, wy = line1_x1 - line2_x1, line1_y1 - line2_y1

	local a = M.dot_product(ux, uy, ux, uy)
	local b = M.dot_product(ux, uy, vx, vy)
	local c = M.dot_product(vx, vy, vx, vy)
	local d = M.dot_product(ux, uy, wx, wy)
	local e = M.dot_product(vx, vy, wx, wy)

	local dt = a * c - b * b
	local sc, tc = 0, 0

	if is_equal(dt, 0) then
		if b > c then
			tc = d / b
		else
			tc = e / c
		end
	else
		sc = (b * e - c * d) / dt
		tc = (a * e - b * d) / dt
	end

	local dvx, dvy = wx + sc * ux - tc * vx, wy + sc * uy - tc * vy
	if less_than_or_equal(M.dot_product(dvx, dvy, dvx, dvy), 0) then
		return (line1_x1 + sc * ux + line2_x1 + tc * vx) * 0.5, (line1_y1 + sc * uy + line2_y1 + tc * vy) * 0.5
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param cx number
---@param cy number
---@param radius number
---@return number?, number?, number?, number?
function M.intersection_point_line_circle(x1, y1, x2, y2, cx, cy, radius)
	local a = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)
	local b = 2 * ((x2 - x1) * (x1 - cx) + (y2 - y1) * (y1 - cy))
	local c = cx * cx + cy * cy + x1 * x1 + y1 * y1 - 2 * (cx * x1 + cy * y1) - radius * radius
	local det = b * b - 4 * a * c

	if det < 0.0 then
		return
	elseif is_equal(det, 0) then
		local delta = -b / (2 * a)
		return x1 + delta * (x2 - x1), y1 + delta * (y2 - y1)
	else -- det > 0
		local sqrt_det = math_sqrt(det)

		local pdelta = (-b + sqrt_det) / (2 * a)
		local px = x1 + pdelta * (x2 - x1)
		local py = y1 + pdelta * (y2 - y1)

		local qdelta = (-b - sqrt_det) / (2 * a)
		local qx = x1 + qdelta * (x2 - x1)
		local qy = y1 + qdelta * (y2 - y1)

		return px, py, qx, qy
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_line_pcurve(x1, y1, x2, y2, point_count, pcurve)
	return M._intersection_point_pcurve(point_count, pcurve, M.intersection_point_segment_line, x1, y1, x2, y2)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_line_polygon(x1, y1, x2, y2, polygon)
	return M._intersection_point_polygon(polygon, M.intersection_point_segment_line, x1, y1, x2, y2)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_line_polyline(x1, y1, x2, y2, polyline)
	return M._intersection_point_polyline(polyline, M.intersection_point_segment_line, x1, y1, x2, y2)
end

---@param line_x1 number
---@param line_y1 number
---@param line_x2 number
---@param line_y2 number
---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
---@return number?, number?, number?, number?
function M.intersection_point_line_quad(line_x1, line_y1, line_x2, line_y2, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)
	_assert(M._quad_is_valid(quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)

	local _x1, _y1, _x2, _y2 ---@type number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
			return true
		end
	end

	local x1, y1 = M.intersection_point_segment_line(quad_x1, quad_y1, quad_x2, quad_y2, line_x1, line_y1, line_x2, line_y2)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
	end

	x1, y1 = M.intersection_point_segment_line(quad_x2, quad_y2, quad_x3, quad_y3, line_x1, line_y1, line_x2, line_y2)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
	end

	x1, y1 = M.intersection_point_segment_line(quad_x3, quad_y3, quad_x4, quad_y4, line_x1, line_y1, line_x2, line_y2)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
	end

	x1, y1 = M.intersection_point_segment_line(quad_x4, quad_y4, quad_x1, quad_y1, line_x1, line_y1, line_x2, line_y2)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
	end

	return _x1, _y1, _x2, _y2
end

---@param line_x1 number
---@param line_y1 number
---@param line_x2 number
---@param line_y2 number
---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
---@return number?, number?, number?, number?
function M.intersection_point_line_triangle(line_x1, line_y1, line_x2, line_y2, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3)
	local _x1, _y1, _x2, _y2 ---@type number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
			return true
		end
	end

	local x1, y1 = M.intersection_point_segment_line(tri_x1, tri_y1, tri_x2, tri_y2, line_x1, line_y1, line_x2, line_y2)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
	end

	x1, y1 = M.intersection_point_segment_line(tri_x2, tri_y2, tri_x3, tri_y3, line_x1, line_y1, line_x2, line_y2)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
	end

	x1, y1 = M.intersection_point_segment_line(tri_x3, tri_y3, tri_x1, tri_y1, line_x1, line_y1, line_x2, line_y2)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
	end

	return _x1, _y1, _x2, _y2
end

---@param point_count1 integer
---@param pcurve1 fun(t:number):number,number
---@param point_count2 integer
---@param pcurve2 fun(t:number):number,number
function M.intersection_point_pcurve_pcurve(point_count1, pcurve1, point_count2, pcurve2)
	return M._intersection_point_pcurve(point_count1, pcurve1, M.intersection_point_segment_pcurve,
		point_count2, pcurve2)
end

---@param point_count integer
---@param pcurve fun(t:number):number,number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_pcurve_polygon(point_count, pcurve, polygon)
	return M._intersection_point_polygon(polygon, M.intersection_point_segment_pcurve, point_count, pcurve)
end

---@param polygon1 number[] -- (x1, y1, x2, y2, ...)
---@param polygon2 number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_polygon_polygon(polygon1, polygon2)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_segment_polygon(sx1, sy1, sx2, sy2, polygon2)
	end
	return M._intersection_point_polygon(polygon1, intersector)
end

---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_polyline_polygon(polyline, polygon)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_segment_polygon(sx1, sy1, sx2, sy2, polygon)
	end
	return M._intersection_point_polyline(polyline, intersector)
end

---@param polyline1 number[] -- (x1, y1, x2, y2, ...)
---@param polyline2 number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_polyline_polyline(polyline1, polyline2)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_segment_polyline(sx1, sy1, sx2, sy2, polyline2)
	end
	return M._intersection_point_polyline(polyline1, intersector)
end

---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param point_count integer
---@param pcurve fun(t:number):number,number
function M.intersection_point_polyline_pcurve(polyline, point_count, pcurve)
	return M._intersection_point_polyline(polyline, M.intersection_point_segment_pcurve, point_count, pcurve)
end

---Returns the intersection points of a ray and a circle
---The closest point is returned first
---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param c_x number
---@param c_y number
---@param c_r number
---@return number?, number?, number?, number?
function M.intersection_point_ray_circle(ray_x, ray_y, ray_dx, ray_dy, c_x, c_y, c_r)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)

	local rx, ry = ray_x - c_x, ray_y - c_y
	local c = rx * rx + ry * ry - c_r * c_r
	local b = rx * ray_dx + ry * ray_dy
	local root_term = b * b >= c and math_sqrt(b * b - c) or 0

	if c > 0 and b >= 0 then
		-- Ray starts outside circle and points away
		return
	end

	if c <= 0 then
		-- Ray starts inside circle
		local t = -b + root_term
		local px, py = ray_x + t * ray_dx, ray_y + t * ray_dy
		if c < -PRECISION then
			return px, py
		else
			local u = -b - root_term
			return ray_x + u * ray_dx, ray_y + u * ray_dy, px, py
		end
	end

	if b * b >= c then
		local t = -b - root_term
		local px, py = ray_x + t * ray_dx, ray_y + t * ray_dy
		if root_term == 0 then
			return px, py
		elseif root_term > 0 then
			local u = -b + root_term
			return px, py, ray_x + u * ray_dx, ray_y + u * ray_dy
		end
	end
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
function M.intersection_point_ray_pcurve(ray_x, ray_y, ray_dx, ray_dy, point_count, pcurve)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_ray_segment(ray_x, ray_y, ray_dx, ray_dy, sx1, sy1, sx2, sy2)
	end
	return M._intersection_point_pcurve(point_count, pcurve, intersector)
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_ray_polyline(ray_x, ray_y, ray_dx, ray_dy, polyline)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_ray_segment(ray_x, ray_y, ray_dx, ray_dy, sx1, sy1, sx2, sy2)
	end
	return M._intersection_point_polyline(polyline, intersector)
end

---@param ray1_x number
---@param ray1_y number
---@param ray1_dx number
---@param ray1_dy number
---@param line_x1 number
---@param line_y1 number
---@param line_x2 number
---@param line_y2 number
---@return number?, number?
function M.intersection_point_ray_line(ray1_x, ray1_y, ray1_dx, ray1_dy, line_x1, line_y1, line_x2, line_y2)
	_assert(is_equal(ray1_dx * ray1_dx + ray1_dy * ray1_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray1_dx, ray1_dy)

	local line_dx, line_dy = M.normalize(line_x2 - line_x1, line_y2 - line_y1)
	local denom = (line_dy * ray1_dx - line_dx * ray1_dy)
	if denom ~= 0 then
		local ta = (line_dx * (ray1_y - line_y1) - line_dy * (ray1_x - line_x1)) / denom
		local tb = (ray1_dy * (line_x1 - ray1_x) - ray1_dx * (line_y1 - ray1_y)) / denom
		if ta >= 0 then
			return ray1_x + (ray1_dx * ta), ray1_y + (ray1_dy * ta)
		end
	elseif M.point_on_ray(line_x1, line_y1, ray1_x, ray1_y, ray1_dx, ray1_dy) then
		return line_x1, line_y1
	elseif M.point_on_line(ray1_x, ray1_y, line_x1, line_y1, line_x2, line_y2) then
		return ray1_x, ray1_y
	end
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param polygon number[] -- polygon with 3 or more vertices, as flat array of x,y coordinates [x1, y1, ..., xn, yn]
---@return number[]? -- Sorted list of intersection points (x1, y1, x2, y2, ...)
function M.intersection_point_ray_polygon(ray_x, ray_y, ray_dx, ray_dy, polygon)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	assert(polygon and #polygon >= 6, "Polygon must have at least 3 vertices")
	local npolygon = #polygon

	-- Distance-sorted list of intersection points
	local points = {}
	local points_t = {}
	local num_points = 0

	local j = npolygon - 1
	for i = 1, npolygon, 2 do
		local px, py, pt = M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, polygon[i],
			polygon[i + 1], polygon[j], polygon[j + 1])
		if px and py then
			local index = 1
			while index <= num_points and points_t[index] < pt do
				index = index + 1
			end
			table_insert(points_t, index, pt)
			table_insert(points, index * 2 - 1, px)
			table_insert(points, index * 2, py)
			num_points = num_points + 1
		end
		j = i
	end

	return num_points > 0 and points or nil
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param px1 number
---@param py1 number
---@param px2 number
---@param py2 number
---@param px3 number
---@param py3 number
---@param px4 number
---@param py4 number
---@return number?, number?, number?, number?
function M.intersection_point_ray_quad(ray_x, ray_y, ray_dx, ray_dy, px1, py1, px2, py2, px3, py3, px4, py4)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	_assert(M._quad_is_valid(px1, py1, px2, py2, px3, py3, px4, py4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", px1, py1, px2, py2, px3, py3, px4, py4)

	local x1, y1, t1, x2, y2, t2 ---@type number?,number?,number?,number?,number?,number?

	--- Add ordered point into one of the free variables
	local function add_ordered_point(px, py, pt)
		if not x1 then
			x1, y1, t1 = px, py, pt
		elseif t1 > pt then
			x2, y2, t2, x1, y1, t1 = x1, y1, t1, px, py, pt
		elseif not x2 or t2 > pt then
			x2, y2, t2 = px, py, pt
		end
	end

	local px, py, pt = M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, px1, py1, px2, py2)
	if px then
		add_ordered_point(px, py, pt)
	end

	px, py, pt = M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, px2, py2, px3, py3)
	if px then
		add_ordered_point(px, py, pt)
	end

	px, py, pt = M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, px3, py3, px4, py4)
	if px then
		add_ordered_point(px, py, pt)
	end

	px, py, pt = M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, px4, py4, px1, py1)
	if px then
		add_ordered_point(px, py, pt)
	end

	return x1, y1, x2, y2
end

---@param ray1_x number
---@param ray1_y number
---@param ray1_dx number
---@param ray1_dy number
---@param ray2_x number
---@param ray2_y number
---@param ray2_dx number
---@param ray2_dy number
---@return number?, number?
function M.intersection_point_ray_ray(ray1_x, ray1_y, ray1_dx, ray1_dy, ray2_x,
									  ray2_y, ray2_dx, ray2_dy)
	_assert(is_equal(ray1_dx * ray1_dx + ray1_dy * ray1_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray1_dx, ray1_dy)
	_assert(is_equal(ray2_dx * ray2_dx + ray2_dy * ray2_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray2_dx, ray2_dy)

	local denom = (ray2_dy * ray1_dx - ray2_dx * ray1_dy)

	if denom ~= 0 then
		local ta = (ray2_dx * (ray1_y - ray2_y) - ray2_dy * (ray1_x - ray2_x)) / denom
		local tb = (ray1_dy * (ray2_x - ray1_x) - ray1_dx * (ray2_y - ray1_y)) / denom

		if ta >= 0 and tb >= 0 then
			return ray1_x + (ray1_dx * ta), ray1_y + (ray1_dy * ta)
		end
	elseif M.point_on_ray(ray2_x, ray2_y, ray1_x, ray1_y, ray1_dx, ray1_dy) then
		return ray2_x, ray2_y
	elseif M.point_on_ray(ray1_x, ray1_y, ray2_x, ray2_y, ray2_dx, ray2_dy) then
		return ray1_x, ray1_y
	end
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return number?, number?, number?, number?
function M.intersection_point_ray_aabb(ray_x, ray_y, ray_dx, ray_dy, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)

	local tmin, tmax = -math_huge, math_huge

	if not_equal(ray_dx, 0) then
		local recip_dir_x = 1.0 / ray_dx

		if ray_dx > 0.0 then
			local t = (max_x - ray_x) * recip_dir_x
			if t < tmin then return end
			tmax = math_min(t, tmax)

			t = (min_x - ray_x) * recip_dir_x
			if t > tmax then return end
			tmin = math_max(t, tmin)
		else
			local t = (min_x - ray_x) * recip_dir_x
			if t < tmin then return end
			tmax = math_min(t, tmax)

			t = (max_x - ray_x) * recip_dir_x
			if t > tmax then return end
			tmin = math_max(t, tmin)
		end
	elseif (ray_x < min_x) or (ray_x > max_x) then
		return
	end

	if not_equal(ray_dy, 0) then
		local recip_dir_y = 1.0 / ray_dy

		if ray_dy > 0.0 then
			local t = (max_y - ray_y) * recip_dir_y
			if t < tmin then return end
			tmax = math_min(t, tmax)

			t = (min_y - ray_y) * recip_dir_y
			if t > tmax then return end
			tmin = math_max(t, tmin)
		else
			local t = (min_y - ray_y) * recip_dir_y
			if t < tmin then return end
			tmax = math_min(t, tmax)

			t = (max_y - ray_y) * recip_dir_y
			if t > tmax then return end
			tmin = math_max(t, tmin)
		end
	elseif (ray_y < min_y) or (ray_y > max_y) then
		return
	end

	if tmin < tmax then
		local px, py ---@type number?,number
		if greater_than_or_equal(tmin, 0) then
			px, py = ray_x + tmin * ray_dx, ray_y + tmin * ray_dy
		end
		if tmax ~= math_huge then
			if px and py then
				return px, py, ray_x + tmax * ray_dx, ray_y + tmax * ray_dy
			else
				return ray_x + tmax * ray_dx, ray_y + tmax * ray_dy
			end
		elseif px and py then
			return px, py
		end
	end
end

---Checks for intersection of a ray and line segment
---
---NOTE: Returns `x, y, dist`, where `x,y` is the intersection point and `dist` is the distance from the ray origin to the intersection point
---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number?, number?, number?
---@package
---@private
function M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, x1, y1, x2, y2)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)
	local perp_x, perp_y = y2 - y1, -(x2 - x1)
	local denom = perp_x * ray_dx + perp_y * ray_dy
	if not_equal(denom, 0.0) then
		local diff_x, diff_y = x1 - ray_x, y1 - ray_y
		local s = (perp_x * diff_x + perp_y * diff_y) / denom
		local t = (ray_dy * diff_x - ray_dx * diff_y) / denom
		if greater_than_or_equal(s, 0.0) and
			greater_than_or_equal(t, 0.0) and
			less_than_or_equal(t, 1.0) then
			return ray_x + ray_dx * s, ray_y + ray_dy * s, s
		end
	else
		local t = M._point_on_ray_distance(x1, y1, ray_x, ray_y, ray_dx, ray_dy)
		if t then
			return x1, y1, t
		end
	end
end

---Checks for intersection of a ray and line segment
---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number?, number?
function M.intersection_point_ray_segment(ray_x, ray_y, ray_dx, ray_dy, x1, y1, x2, y2)
	local x, y, _ = M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, x1, y1, x2, y2)
	if x and y then
		return x, y
	end
end

---@param ray_x number
---@param ray_y number
---@param ray_dx number
---@param ray_dy number
---@param px1 number
---@param py1 number
---@param px2 number
---@param py2 number
---@param px3 number
---@param py3 number
---@return number?, number?, number?, number?
function M.intersection_point_ray_triangle(ray_x, ray_y, ray_dx, ray_dy, px1, py1, px2, py2, px3, py3)
	_assert(is_equal(ray_dx * ray_dx + ray_dy * ray_dy, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", ray_dx, ray_dy)

	local x1, y1, t1, x2, y2, t2 ---@type number?,number?,number?,number?,number?,number?

	--- Add ordered point into one of the free variables
	local function add_ordered_point(px, py, pt)
		if not x1 then
			x1, y1, t1 = px, py, pt
		elseif t1 > pt then
			x2, y2, t2, x1, y1, t1 = x1, y1, t1, px, py, pt
		elseif not x2 or t2 > pt then
			x2, y2, t2 = px, py, pt
		end
	end

	local px, py, pt = M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, px1, py1, px2, py2)
	if px then
		add_ordered_point(px, py, pt)
	end

	px, py, pt = M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, px2, py2, px3, py3)
	if px then
		add_ordered_point(px, py, pt)
	end

	px, py, pt = M._intersection_point_ray_segment_dist(ray_x, ray_y, ray_dx, ray_dy, px3, py3, px1, py1)
	if px then
		add_ordered_point(px, py, pt)
	end

	return x1, y1, x2, y2
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return number?, number?, number?, number?
function M.intersection_point_segment_aabb(x1, y1, x2, y2, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)

	if not M.intersect_segment_aabb(x1, y1, x2, y2, min_x, min_y, max_x, max_y) then
		return
	end

	local _x1, _y1, _x2, _y2 ---@type number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
			return true
		end
	end

	local ix, iy = M.intersection_point_segment_segment(x1, y1, x2, y2, min_x, min_y, max_x, min_y)
	if ix and iy then
		if _add_point(ix, iy) then return _x1, _y1, _x2, _y2 end
	end

	ix, iy = M.intersection_point_segment_segment(x1, y1, x2, y2, max_x, min_y, max_x, max_y)
	if ix and iy then
		if _add_point(ix, iy) then return _x1, _y1, _x2, _y2 end
	end

	ix, iy = M.intersection_point_segment_segment(x1, y1, x2, y2, max_x, max_y, min_x, max_y)
	if ix and iy then
		if _add_point(ix, iy) then return _x1, _y1, _x2, _y2 end
	end

	ix, iy = M.intersection_point_segment_segment(x1, y1, x2, y2, min_x, max_y, min_x, min_y)
	if ix and iy then
		if _add_point(ix, iy) then return _x1, _y1, _x2, _y2 end
	end

	return _x1, _y1, _x2, _y2
end

---@param seg_x1 number
---@param seg_y1 number
---@param seg_x2 number
---@param seg_y2 number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
function M.intersection_point_segment_pcurve(seg_x1, seg_y1, seg_x2, seg_y2, point_count, pcurve)
	return M._intersection_point_pcurve(point_count, pcurve, M.intersection_point_segment_segment, seg_x1,
		seg_y1, seg_x2, seg_y2)
end

---@param seg_x1 number
---@param seg_y1 number
---@param seg_x2 number
---@param seg_y2 number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_segment_polygon(seg_x1, seg_y1, seg_x2, seg_y2, polygon)
	return M._intersection_point_polygon(polygon, M.intersection_point_segment_segment, seg_x1, seg_y1, seg_x2, seg_y2)
end

---@param seg_x1 number
---@param seg_y1 number
---@param seg_x2 number
---@param seg_y2 number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_segment_polyline(seg_x1, seg_y1, seg_x2, seg_y2, polyline)
	return M._intersection_point_polyline(polyline, M.intersection_point_segment_segment, seg_x1, seg_y1, seg_x2, seg_y2)
end

---Checks for intersection of a segment and line
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number?, number?
function M.intersection_point_segment_line(x1, y1, x2, y2, x3, y3, x4, y4)
	local perp_x, perp_y = y2 - y1, -(x2 - x1)
	local line_dx, line_dy = M.normalize(x4 - x3, y4 - y3)
	local denom = perp_x * line_dx + perp_y * line_dy
	if not_equal(denom, 0.0) then
		local diff_x, diff_y = x1 - x3, y1 - y3
		local s = (perp_x * diff_x + perp_y * diff_y) / denom
		local t = (line_dy * diff_x - line_dx * diff_y) / denom
		if greater_than_or_equal(t, 0.0) and less_than_or_equal(t, 1.0) then
			return x3 + line_dx * s, y3 + line_dy * s
		end
	else
		local t = M.dot_product(line_dx, line_dy, x1 - x3, y1 - y3)
		local rx, ry = x3 + t * line_dx, y3 + t * line_dy
		if is_equal(x1, rx) and is_equal(y1, ry) then
			return x1, y1
		end
	end
end

---@param seg_x1 number
---@param seg_y1 number
---@param seg_x2 number
---@param seg_y2 number
---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
---@return number?, number?, number?, number?
function M.intersection_point_segment_quad(seg_x1, seg_y1, seg_x2, seg_y2, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3,
										   quad_y3, quad_x4, quad_y4)
	_assert(M._quad_is_valid(quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)
	local _x1, _y1, _x2, _y2 ---@type number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
			return true
		end
	end

	-- Test each segment of quad
	local x1, y1, x2, y2 = M.intersection_point_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, quad_x1, quad_y1, quad_x2,
		quad_y2)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
		if x2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, quad_x2, quad_y2, quad_x3,
		quad_y3)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
		if x2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, quad_x3, quad_y3, quad_x4,
		quad_y4)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
		if x2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, quad_x4, quad_y4, quad_x1,
		quad_y1)
	if x1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2 end
		if x2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2 end
		end
	end

	return _x1, _y1, _x2, _y2
end

---Calculate up to two intersection points of two line segments
---Two points are returned if the line segments overlap
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number?, number?, number?, number?
function M.intersection_point_segment_segment(x1, y1, x2, y2, x3, y3, x4, y4)
	local ax, ay, bx, by, cx, cy = intersect_segment_helper(x1, y1, x2, y2, x3, y3, x4, y4)
	
	if not ax then
		return
	end

	local denom = (ax * -by) - (ay * -bx)
	if not_equal(denom, 0) then
		local ratio = ((cy * -bx) - (cx * -by)) / denom
		return x1 + (ratio * ax), y1 + (ratio * ay)
	elseif is_equal((ax * -cy), (-cx * ay)) then
		local dot = (ax * cx) + (ay * cy)
		if dot < 0 then
			return x3, y3, x2, y2
		else
			return x1, y1, x4, y4
		end
	else
		return x4, y4
	end
end

---@param seg_x1 number
---@param seg_y1 number
---@param seg_x2 number
---@param seg_y2 number
---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
---@return number?, number?, number?, number?
function M.intersection_point_segment_triangle(seg_x1, seg_y1, seg_x2, seg_y2, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3,
											   tri_y3)
	-- NOTE: Ignore overlapping segments as they will be captured by the other segments
	local px, py ---@type number?, number?

	local ix, iy = M.intersection_point_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, tri_x1, tri_y1, tri_x2, tri_y2) ---@type number?, number?
	if ix and iy then
		px, py = ix, iy
	end

	ix, iy = M.intersection_point_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, tri_x2, tri_y2, tri_x3, tri_y3) ---@type number?, number?
	if ix and iy then
		if not px then
			px, py = ix, iy
		else
			return px, py, ix, iy
		end
	end

	ix, iy = M.intersection_point_segment_segment(seg_x1, seg_y1, seg_x2, seg_y2, tri_x3, tri_y3, tri_x1, tri_y1) ---@type number?, number?
	if ix and iy then
		if not px then
			return ix, iy
		else
			return px, py, ix, iy
		end
	end

	return px, py
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param cx number
---@param cy number
---@param radius number
---@return number?, number?, number?, number?
function M.intersection_point_segment_circle(x1, y1, x2, y2, cx, cy, radius)
	local p1_in_circle = M.point_in_circle(x1, y1, cx, cy, radius)
	local p2_in_circle = M.point_in_circle(x2, y2, cx, cy, radius)

	if p1_in_circle and p2_in_circle then
		-- return x1, y1, x2, y2
		return -- BP NOTE: Segment is inside, no "intersection"
	end

	local sq_radius = radius * radius

	if p1_in_circle or p2_in_circle then
		local px, py = M.closest_point_on_line_from_point(x1, y1, x2, y2, cx, cy)
		if p1_in_circle then
			local h = M.sq_distance(px, py, cx, cy)
			local a = math_sqrt(sq_radius - h)
			local ix, iy = project_point(px, py, x2, y2, a)
			return ix, iy -- BP NOTE: Don't include internal point
		elseif p2_in_circle then
			local h = M.sq_distance(px, py, cx, cy)
			local a = math_sqrt(sq_radius - h)
			local ix, iy = project_point(px, py, x1, y1, a)
			-- return x2, y2, ix, iy
			return ix, iy -- BP NOTE: Don't include internal point
		end
	end

	local px, py = M.closest_point_on_segment_from_point(x1, y1, x2, y2, cx, cy)
	if (is_equal(x1, px) and is_equal(y1, py)) or (is_equal(x2, px) and is_equal(y2, py)) then
		return
	else
		local h = M.sq_distance(px, py, cx, cy)
		if is_equal(h, sq_radius) then
			return px, py
		elseif h > sq_radius then
			return
		elseif is_equal(h, 0) then
			-- Segment goes through center
			local ix, iy = project_point(cx, cy, x1, y1, radius)
			local ix2, iy2 = project_point(cx, cy, x2, y2, radius)
			return ix, iy, ix2, iy2
		else
			local a = math_sqrt(sq_radius - h)
			local ix, iy = project_point(px, py, x1, y1, a)
			local ix2, iy2 = project_point(px, py, x2, y2, a)
			return ix, iy, ix2, iy2
		end
	end
end

---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
---@param cx number
---@param cy number
---@param radius number
---@return number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?
function M.intersection_point_triangle_circle(tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3, cx, cy, radius)
	-- If triangle is fully contained then there are no intersections
	if M.point_in_circle(tri_x1, tri_y1, cx, cy, radius) and
		M.point_in_circle(tri_x2, tri_y2, cx, cy, radius) and
		M.point_in_circle(tri_x3, tri_y3, cx, cy, radius) then
		return
	end

	local _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 ---@type number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
		elseif _x3 == nil or (_px == _x3 and _py == _y3) then
			_x3, _y3 = _px, _py
		elseif _x4 == nil or (_px == _x4 and _py == _y4) then
			_x4, _y4 = _px, _py
		elseif _x5 == nil or (_px == _x5 and _py == _y5) then
			_x5, _y5 = _px, _py
		elseif _x6 == nil or (_px == _x6 and _py == _y6) then
			_x6, _y6 = _px, _py
			return true
		end
	end

	local x1, y1, x2, y2 = M.intersection_point_segment_circle(tri_x1, tri_y1, tri_x2, tri_y2, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_circle(tri_x2, tri_y2, tri_x3, tri_y3, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_circle(tri_x3, tri_y3, tri_x1, tri_y1, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
		end
	end

	return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_triangle_pcurve(x1, y1, x2, y2, x3, y3, point_count, pcurve)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_segment_triangle(sx1, sy1, sx2, sy2, x1, y1, x2, y2, x3, y3)
	end
	return M._intersection_point_pcurve(point_count, pcurve, intersector)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_triangle_polygon(x1, y1, x2, y2, x3, y3, polygon)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_segment_triangle(sx1, sy1, sx2, sy2, x1, y1, x2, y2, x3, y3)
	end
	return M._intersection_point_polygon(polygon, intersector)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_triangle_polyline(x1, y1, x2, y2, x3, y3, polyline)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_segment_triangle(sx1, sy1, sx2, sy2, x1, y1, x2, y2, x3, y3)
	end
	return M._intersection_point_polyline(polyline, intersector)
end

---@param tri1_x1 number
---@param tri1_y1 number
---@param tri1_x2 number
---@param tri1_y2 number
---@param tri1_x3 number
---@param tri1_y3 number
---@param tri2_x1 number
---@param tri2_y1 number
---@param tri2_x2 number
---@param tri2_y2 number
---@param tri2_x3 number
---@param tri2_y3 number
---@return number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?
function M.intersection_point_triangle_triangle(tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri1_x3, tri1_y3, tri2_x1, tri2_y1,
												tri2_x2, tri2_y2, tri2_x3, tri2_y3)
	local _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 ---@type number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
		elseif _x3 == nil or (_px == _x3 and _py == _y3) then
			_x3, _y3 = _px, _py
		elseif _x4 == nil or (_px == _x4 and _py == _y4) then
			_x4, _y4 = _px, _py
		elseif _x5 == nil or (_px == _x5 and _py == _y5) then
			_x5, _y5 = _px, _py
		elseif _x6 == nil or (_px == _x6 and _py == _y6) then
			_x6, _y6 = _px, _py
			return true
		end
	end

	local x1, y1, x2, y2 = M.intersection_point_segment_triangle(tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri2_x1, tri2_y1,
		tri2_x2, tri2_y2, tri2_x3, tri2_y3)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
	end
	if x2 and y2 then
		if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_triangle(tri1_x2, tri1_y2, tri1_x3, tri1_y3, tri2_x1, tri2_y1, tri2_x2,
		tri2_y2, tri2_x3, tri2_y3)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
	end
	if x2 and y2 then
		if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_triangle(tri1_x3, tri1_y3, tri1_x1, tri1_y1, tri2_x1, tri2_y1, tri2_x2,
		tri2_y2, tri2_x3, tri2_y3)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
	end
	if x2 and y2 then
		if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6 end
	end

	return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6
end

---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return number?, number?, number?, number?, number?, number?, number?, number?
function M.intersection_point_triangle_aabb(tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3, min_x, min_y, max_x,
											max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)

	-- Check for total containment
	local emin_x, emin_y, emax_x, emax_y = min_x + PRECISION, min_y + PRECISION, max_x - PRECISION, max_y - PRECISION
	if M.point_in_aabb(tri_x1, tri_y1, emin_x, emin_y, emax_x, emax_y) and
		M.point_in_aabb(tri_x2, tri_y2, emin_x, emin_y, emax_x, emax_y) and
		M.point_in_aabb(tri_x3, tri_y3, emin_x, emin_y, emax_x, emax_y) then
		-- Triangle is fully contained
		return
	end

	local _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 ---@type number?,number?,number?,number?,number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
		elseif _x3 == nil or (_px == _x3 and _py == _y3) then
			_x3, _y3 = _px, _py
		elseif _x4 == nil or (_px == _x4 and _py == _y4) then
			_x4, _y4 = _px, _py
			return true
		end
	end

	local x1, y1, x2, y2 = M.intersection_point_segment_aabb(tri_x1, tri_y1, tri_x2, tri_y2, min_x, min_y, max_x,
		max_y)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_aabb(tri_x2, tri_y2, tri_x3, tri_y3, min_x, min_y, max_x, max_y)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_aabb(tri_x3, tri_y3, tri_x1, tri_y1, min_x, min_y, max_x, max_y)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4
end

---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
---@return number?, number?, number?, number?, number?, number?, number?, number?
function M.intersection_point_triangle_quad(tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3, quad_x1, quad_y1, quad_x2,
											quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)
	_assert(M._quad_is_valid(quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)

	local _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 ---@type number?,number?,number?,number?,number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
		elseif _x3 == nil or (_px == _x3 and _py == _y3) then
			_x3, _y3 = _px, _py
		elseif _x4 == nil or (_px == _x4 and _py == _y4) then
			_x4, _y4 = _px, _py
			return true
		end
	end

	local x1, y1, x2, y2 = M.intersection_point_segment_quad(tri_x1, tri_y1, tri_x2, tri_y2, quad_x1, quad_y1, quad_x2,
		quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_quad(tri_x2, tri_y2, tri_x3, tri_y3, quad_x1, quad_y1, quad_x2,
		quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_quad(tri_x3, tri_y3, tri_x1, tri_y1, quad_x1, quad_y1, quad_x2,
		quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4
end

---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return number?, number?, number?, number?, number?, number?, number?, number?
function M.intersection_point_quad_aabb(quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4, min_x,
										min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)

	-- Check for total containment
	local emin_x, emin_y, emax_x, emax_y = min_x + PRECISION, min_y + PRECISION, max_x - PRECISION, max_y - PRECISION
	if M.point_in_aabb(quad_x1, quad_y1, emin_x, emin_y, emax_x, emax_y) and
		M.point_in_aabb(quad_x2, quad_y2, emin_x, emin_y, emax_x, emax_y) and
		M.point_in_aabb(quad_x3, quad_y3, emin_x, emin_y, emax_x, emax_y) and
		M.point_in_aabb(quad_x4, quad_y4, emin_x, emin_y, emax_x, emax_y) then
		-- Quad is fully contained
		return
	end

	local _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 ---@type number?,number?,number?,number?,number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
		elseif _x3 == nil or (_px == _x3 and _py == _y3) then
			_x3, _y3 = _px, _py
		elseif _x4 == nil or (_px == _x4 and _py == _y4) then
			_x4, _y4 = _px, _py
			return true
		end
	end

	local x1, y1, x2, y2 = M.intersection_point_segment_aabb(quad_x1, quad_y1, quad_x2, quad_y2, min_x, min_y, max_x,
		max_y)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_aabb(quad_x2, quad_y2, quad_x3, quad_y3, min_x, min_y, max_x, max_y)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_aabb(quad_x3, quad_y3, quad_x4, quad_y4, min_x, min_y, max_x, max_y)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_aabb(quad_x4, quad_y4, quad_x1, quad_y1, min_x, min_y, max_x, max_y)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4 end
		end
	end

	return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4
end

---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
---@param cx number
---@param cy number
---@param radius number
---@return number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?
function M.intersection_point_quad_circle(quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4, cx, cy, radius)
	-- If quad is fully contained then there are no intersections
	if M.point_in_circle(quad_x1, quad_y1, cx, cy, radius) and
		M.point_in_circle(quad_x2, quad_y2, cx, cy, radius) and
		M.point_in_circle(quad_x3, quad_y3, cx, cy, radius) and
		M.point_in_circle(quad_x4, quad_y4, cx, cy, radius) then
			return
	end

	local _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 ---@type number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
		elseif _x3 == nil or (_px == _x3 and _py == _y3) then
			_x3, _y3 = _px, _py
		elseif _x4 == nil or (_px == _x4 and _py == _y4) then
			_x4, _y4 = _px, _py
		elseif _x5 == nil or (_px == _x5 and _py == _y5) then
			_x5, _y5 = _px, _py
		elseif _x6 == nil or (_px == _x6 and _py == _y6) then
			_x6, _y6 = _px, _py
		elseif _x7 == nil or (_px == _x7 and _py == _y7) then
			_x7, _y7 = _px, _py
		elseif _x8 == nil or (_px == _x8 and _py == _y8) then
			_x8, _y8 = _px, _py
			return true
		end
	end

	local x1, y1, x2, y2 = M.intersection_point_segment_circle(quad_x1, quad_y1, quad_x2, quad_y2, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_circle(quad_x2, quad_y2, quad_x3, quad_y3, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_circle(quad_x3, quad_y3, quad_x4, quad_y4, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_circle(quad_x4, quad_y4, quad_x1, quad_y1, cx, cy, radius)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		end
	end

	return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_quad_pcurve(x1, y1, x2, y2, x3, y3, x4, y4, point_count, pcurve)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_segment_quad(sx1, sy1, sx2, sy2, x1, y1, x2, y2, x3, y3, x4, y4)
	end
	return M._intersection_point_pcurve(point_count, pcurve, intersector)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_quad_polygon(x1, y1, x2, y2, x3, y3, x4, y4, polygon)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_segment_quad(sx1, sy1, sx2, sy2, x1, y1, x2, y2, x3, y3, x4, y4)
	end
	return M._intersection_point_polygon(polygon, intersector)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.intersection_point_quad_polyline(x1, y1, x2, y2, x3, y3, x4, y4, polyline)
	local function intersector(sx1, sy1, sx2, sy2)
		return M.intersection_point_segment_quad(sx1, sy1, sx2, sy2, x1, y1, x2, y2, x3, y3, x4, y4)
	end
	return M._intersection_point_polyline(polyline, intersector)
end

---@param quad1_x1 number
---@param quad1_y1 number
---@param quad1_x2 number
---@param quad1_y2 number
---@param quad1_x3 number
---@param quad1_y3 number
---@param quad1_x4 number
---@param quad1_y4 number
---@param quad2_x1 number
---@param quad2_y1 number
---@param quad2_x2 number
---@param quad2_y2 number
---@param quad2_x3 number
---@param quad2_y3 number
---@param quad2_x4 number
---@param quad2_y4 number
function M.intersection_point_quad_quad(quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4,
										quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4)
	_assert(M._quad_is_valid(quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad1_x4, quad1_y4)
	_assert(M._quad_is_valid(quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4), "bad argument (quad is not convex) (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)", quad2_x1, quad2_y1, quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4)

	local _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 ---@type number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?,number?
	local function _add_point(_px, _py)
		if _x1 == nil or (_px == _x1 and _py == _y1) then
			_x1, _y1 = _px, _py
		elseif _x2 == nil or (_px == _x2 and _py == _y2) then
			_x2, _y2 = _px, _py
		elseif _x3 == nil or (_px == _x3 and _py == _y3) then
			_x3, _y3 = _px, _py
		elseif _x4 == nil or (_px == _x4 and _py == _y4) then
			_x4, _y4 = _px, _py
		elseif _x5 == nil or (_px == _x5 and _py == _y5) then
			_x5, _y5 = _px, _py
		elseif _x6 == nil or (_px == _x6 and _py == _y6) then
			_x6, _y6 = _px, _py
		elseif _x7 == nil or (_px == _x7 and _py == _y7) then
			_x7, _y7 = _px, _py
		elseif _x8 == nil or (_px == _x8 and _py == _y8) then
			_x8, _y8 = _px, _py
			return true
		end
	end

	local x1, y1, x2, y2 = M.intersection_point_segment_quad(quad1_x1, quad1_y1, quad1_x2, quad1_y2, quad2_x1, quad2_y1,
		quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_quad(quad1_x2, quad1_y2, quad1_x3, quad1_y3, quad2_x1, quad2_y1,
		quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_quad(quad1_x3, quad1_y3, quad1_x4, quad1_y4, quad2_x1, quad2_y1,
		quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		end
	end

	x1, y1, x2, y2 = M.intersection_point_segment_quad(quad1_x4, quad1_y4, quad1_x1, quad1_y1, quad2_x1, quad2_y1,
		quad2_x2, quad2_y2, quad2_x3, quad2_y3, quad2_x4, quad2_y4)
	if x1 and y1 then
		if _add_point(x1, y1) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		if x2 and y2 then
			if _add_point(x2, y2) then return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8 end
		end
	end

	return _x1, _y1, _x2, _y2, _x3, _y3, _x4, _y4, _x5, _y5, _x6, _y6, _x7, _y7, _x8, _y8
end

-----------------------------------------------------------
-- Containment (Within) tests
-----------------------------------------------------------

---Check if `a` is contained within `b`
---@param a_min_x number
---@param a_min_y number
---@param a_max_x number
---@param a_max_y number
---@param b_min_x number
---@param b_min_y number
---@param b_max_x number
---@param b_max_y number
---@return boolean
function M.aabb_within_aabb(a_min_x, a_min_y, a_max_x, a_max_y, b_min_x, b_min_y, b_max_x, b_max_y)
	_assert(a_min_x <= a_max_x and a_min_y <= a_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", a_min_x, a_max_x, a_min_y, a_max_y)
	_assert(b_min_x <= b_max_x and b_min_y <= b_max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", b_min_x, b_max_x, b_min_y, b_max_y)
	return a_min_x >= b_min_x and a_min_y >= b_min_y and a_max_x <= b_max_x and a_max_y <= b_max_y
end

---@param x number
---@param y number
---@param radius number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return boolean
function M.circle_within_aabb(x, y, radius, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	local cmin_x, cmin_y, cmax_x, cmax_y = M.aabb_circle(x, y, radius)
	return M.aabb_within_aabb(cmin_x, cmin_y, cmax_x, cmax_y, min_x, min_y, max_x, max_y)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return boolean
function M.triangle_within_aabb(x1, y1, x2, y2, x3, y3, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return M.point_in_aabb(x1, y1, min_x, min_y, max_x, max_y) and
		M.point_in_aabb(x2, y2, min_x, min_y, max_x, max_y) and
		M.point_in_aabb(x3, y3, min_x, min_y, max_x, max_y)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return boolean
function M.segment_within_aabb(x1, y1, x2, y2, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return M.point_in_aabb(x1, y1, min_x, min_y, max_x, max_y) and
		M.point_in_aabb(x2, y2, min_x, min_y, max_x, max_y)
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return boolean
function M.quad_within_aabb(x1, y1, x2, y2, x3, y3, x4, y4, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return M.point_in_aabb(x1, y1, min_x, min_y, max_x, max_y) and
		M.point_in_aabb(x2, y2, min_x, min_y, max_x, max_y) and
		M.point_in_aabb(x3, y3, min_x, min_y, max_x, max_y) and
		M.point_in_aabb(x4, y4, min_x, min_y, max_x, max_y)
end

---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return boolean
function M.pcurve_within_aabb(point_count, pcurve, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	assert(point_count >= 2, "point_count must be >= 2")
	for i = 1, point_count do
		local x, y = pcurve((i - 1) / (point_count - 1))
		if not M.point_in_aabb(x, y, min_x, min_y, max_x, max_y) then
			return false
		end
	end
	return true
end

---@param points number[] -- (x1, y1, x2, y2, ...)
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return boolean
function M.points_within_aabb(points, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	for i = 1, #points, 2 do
		if not M.point_in_aabb(points[i], points[i + 1], min_x, min_y, max_x, max_y) then
			return false
		end
	end
	return true
end

---@param x1 number
---@param y1 number
---@param r1 number
---@param x2 number
---@param y2 number
---@param r2 number
---@return boolean
function M.circle_within_circle(x1, y1, r1, x2, y2, r2)
	return r2 >= (M.distance(x1, y1, x2, y2) + r1)
end

-----------------------------------------------------------
-- Reflection
-----------------------------------------------------------

---Given a segment (sx1,sy1)->(sx2,sy2) and two points (p1x, p1y) and (p2x, p2y)
---calculate the point of reflection (x, y) that lies on the segment
---```txt
---    p1   p2
---     \   /
---      \ /
---  s1---*---s2
---```
---@param sx1 number
---@param sy1 number
---@param sx2 number
---@param sy2 number
---@param p1x number
---@param p1y number
---@param p2x number
---@param p2y number
---@return number?,number?
function M.point_of_reflection(sx1, sy1, sx2, sy2, p1x, p1y, p2x, p2y)
	if not M.collinear(sx1, sy1, sx2, sy2, p1x, p1y) and
		not M.collinear(sx1, sy1, sx2, sy2, p2x, p2y) and
		M.orientation(sx1, sy1, sx2, sy2, p1x, p1y) == M.orientation(sx1, sy1, sx2, sy2, p2x, p2y)
	then
		local p1px, p1py = M.closest_point_on_line_from_point(sx1, sy1, sx2, sy2, p1x, p1y)
		local p2px, p2py = M.closest_point_on_line_from_point(sx1, sy1, sx2, sy2, p2x, p2y)
		-- NOTE: Assume these segments aren't collinear
		local ix, iy = M.intersection_point_segment_segment(p1x, p1y, p2px, p2py, p2x, p2y, p1px, p1py) ---@type number?, number?
		if ix and iy then
			local rpx, rpy = M.closest_point_on_line_from_point(sx1, sy1, sx2, sy2, ix, iy)
			if M.is_point_collinear(sx1, sy1, sx2, sy2, rpx, rpy) then
				return rpx, rpy
			end
		end
	end
end

-----------------------------------------------------------
-- Shape Transformations
-----------------------------------------------------------

---@param angle degrees
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.rotate_aaab(angle, min_x, min_y, max_x, max_y)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local x1, y1 = (min_x * cos) - (min_y * sin), (min_y * cos) + (min_x * sin)
	local x2, y2 = (max_x * cos) - (min_y * sin), (min_y * cos) + (max_x * sin)
	local x3, y3 = (max_x * cos) - (max_y * sin), (max_y * cos) + (max_x * sin)
	local x4, y4 = (min_x * cos) - (max_y * sin), (max_y * cos) + (min_x * sin)
	return math_min(x1, x2, x3, x4), math_min(y1, y2, y3, y4), math_max(x1, x2, x3, x4), math_max(y1, y2, y3, y4)
end

---@param angle degrees
---@param x number
---@param y number
---@return number, number
function M.rotate_point(angle, x, y)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local nx, ny = (x * cos) - (y * sin), (y * cos) + (x * sin)
	return nx, ny
end

---@param angle degrees
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.rotate_points(angle, points)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local rotated_points = {}
	for i = 1, #points, 2 do
		local x = points[i]
		local y = points[i + 1]
		local nx = (x * cos) - (y * sin)
		local ny = (y * cos) + (x * sin)
		table_insert(rotated_points, nx)
		table_insert(rotated_points, ny)
	end
	return rotated_points
end

---@param angle degrees
---@param points number[] -- (x1, y1, x2, y2, ...)
function M.rotate_points_in_place(angle, points)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	for i = 1, #points, 2 do
		local x = points[i]
		local y = points[i + 1]
		local nx = (x * cos) - (y * sin)
		local ny = (y * cos) + (x * sin)
		points[i] = nx
		points[i + 1] = ny
	end
	return points
end

---@param angle degrees
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number, number, number, number, number, number, number, number
function M.rotate_quad(angle, x1, y1, x2, y2, x3, y3, x4, y4)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local nx1, ny1 = (x1 * cos) - (y1 * sin), (y1 * cos) + (x1 * sin)
	local nx2, ny2 = (x2 * cos) - (y2 * sin), (y2 * cos) + (x2 * sin)
	local nx3, ny3 = (x3 * cos) - (y3 * sin), (y3 * cos) + (x3 * sin)
	local nx4, ny4 = (x4 * cos) - (y4 * sin), (y4 * cos) + (x4 * sin)
	return nx1, ny1, nx2, ny2, nx3, ny3, nx4, ny4
end

---@param angle degrees
---@param x number
---@param y number
---@param dir_x number
---@param dir_y number
---@return number, number, number, number
function M.rotate_ray(angle, x, y, dir_x, dir_y)
	_assert(is_equal(dir_x * dir_x + dir_y * dir_y, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", dir_x, dir_y)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local nx, ny = (dir_x * cos) - (dir_y * sin), (dir_y * cos) + (dir_x * sin)
	return x, y, nx, ny
end

---@param angle degrees
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number, number, number
function M.rotate_segment(angle, x1, y1, x2, y2)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local nx1, ny1 = (x1 * cos) - (y1 * sin), (y1 * cos) + (x1 * sin)
	local nx2, ny2 = (x2 * cos) - (y2 * sin), (y2 * cos) + (x2 * sin)
	return nx1, ny1, nx2, ny2
end

M.rotate_line = M.rotate_segment

---@param angle degrees
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number, number, number, number, number
function M.rotate_triangle(angle, x1, y1, x2, y2, x3, y3)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local nx1, ny1 = (x1 * cos) - (y1 * sin), (y1 * cos) + (x1 * sin)
	local nx2, ny2 = (x2 * cos) - (y2 * sin), (y2 * cos) + (x2 * sin)
	local nx3, ny3 = (x3 * cos) - (y3 * sin), (y3 * cos) + (x3 * sin)
	return nx1, ny1, nx2, ny2, nx3, ny3
end

---@param angle degrees
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@param ox number
---@param oy number
function M.rotate_aabb_around_origin(angle, min_x, min_y, max_x, max_y, ox, oy)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local tmin_x, tmin_y, tmax_x, tmax_y = min_x - ox, min_y - oy, max_x - ox, max_y - oy
	local x1, y1 = ox + (tmin_x * cos) - (tmin_y * sin), oy + (tmin_y * cos) + (tmin_x * sin)
	local x2, y2 = ox + (tmax_x * cos) - (tmin_y * sin), oy + (tmin_y * cos) + (tmax_x * sin)
	local x3, y3 = ox + (tmax_x * cos) - (tmax_y * sin), oy + (tmax_y * cos) + (tmax_x * sin)
	local x4, y4 = ox + (tmin_x * cos) - (tmax_y * sin), oy + (tmax_y * cos) + (tmin_x * sin)
	return math_min(x1, x2, x3, x4), math_min(y1, y2, y3, y4), math_max(x1, x2, x3, x4), math_max(y1, y2, y3, y4)
end

---@param angle degrees
---@param x number
---@param y number
---@param ox number
---@param oy number
---@return number, number
function M.rotate_point_around_origin(angle, x, y, ox, oy)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local rx, ry = x - ox, y - oy
	local nx, ny = (rx * cos) - (ry * sin) + ox, (ry * cos) + (rx * sin) + oy
	return nx, ny
end

---@param angle degrees
---@param points number[] -- (x1, y1, x2, y2, ...)
---@param ox number
---@param oy number
---@return number[]
function M.rotate_points_around_origin(angle, points, ox, oy)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local rotated_points = {}
	for i = 1, #points, 2 do
		local x = points[i] - ox
		local y = points[i + 1] - oy
		local nx = (x * cos) - (y * sin)
		local ny = (y * cos) + (x * sin)
		table_insert(rotated_points, nx + ox)
		table_insert(rotated_points, ny + oy)
	end
	return rotated_points
end

---@param angle degrees
---@param points number[] -- (x1, y1, x2, y2, ...)
---@param ox number
---@param oy number
function M.rotate_points_in_place_around_origin(angle, points, ox, oy)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	for i = 1, #points, 2 do
		local x = points[i] - ox
		local y = points[i + 1] - oy
		local nx = (x * cos) - (y * sin)
		local ny = (y * cos) + (x * sin)
		points[i] = nx + ox
		points[i + 1] = ny + oy
	end
	return points
end

---@param angle degrees
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param ox number
---@param oy number
---@return number, number, number, number, number, number, number, number
function M.rotate_quad_around_origin(angle, x1, y1, x2, y2, x3, y3, x4, y4, ox, oy)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local nx1, ny1 = (x1 - ox) * cos - (y1 - oy) * sin, (y1 - oy) * cos + (x1 - ox) * sin
	local nx2, ny2 = (x2 - ox) * cos - (y2 - oy) * sin, (y2 - oy) * cos + (x2 - ox) * sin
	local nx3, ny3 = (x3 - ox) * cos - (y3 - oy) * sin, (y3 - oy) * cos + (x3 - ox) * sin
	local nx4, ny4 = (x4 - ox) * cos - (y4 - oy) * sin, (y4 - oy) * cos + (x4 - ox) * sin
	return nx1 + ox, ny1 + oy, nx2 + ox, ny2 + oy, nx3 + ox, ny3 + oy, nx4 + ox, ny4 + oy
end

---@param angle degrees
---@param x number
---@param y number
---@param dir_x number
---@param dir_y number
---@param ox number
---@param oy number
---@return number, number, number, number
function M.rotate_ray_around_origin(angle, x, y, dir_x, dir_y, ox, oy)
	_assert(is_equal(dir_x * dir_x + dir_y * dir_y, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", dir_x, dir_y)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local new_x, new_y = ox + (x - ox) * cos - (y - oy) * sin, oy + (y - oy) * cos + (x - ox) * sin
	local new_dir_x, new_dir_y = (dir_x * cos) - (dir_y * sin), (dir_y * cos) + (dir_x * sin)
	return new_x, new_y, new_dir_x, new_dir_y
end

---@param angle degrees
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param ox number
---@param oy number
---@return number, number, number, number
function M.rotate_segment_around_origin(angle, x1, y1, x2, y2, ox, oy)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local nx1, ny1 = (x1 - ox) * cos - (y1 - oy) * sin, (y1 - oy) * cos + (x1 - ox) * sin
	local nx2, ny2 = (x2 - ox) * cos - (y2 - oy) * sin, (y2 - oy) * cos + (x2 - ox) * sin
	return nx1 + ox, ny1 + oy, nx2 + ox, ny2 + oy
end

M.rotate_line_around_origin = M.rotate_segment_around_origin

---@param angle degrees
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param ox number
---@param oy number
---@return number, number, number, number, number, number
function M.rotate_triangle_around_origin(angle, x1, y1, x2, y2, x3, y3, ox, oy)
	local cos, sin = math_cos(angle * PI_DIV_180), math_sin(angle * PI_DIV_180)
	local nx1, ny1 = (x1 - ox) * cos - (y1 - oy) * sin, (y1 - oy) * cos + (x1 - ox) * sin
	local nx2, ny2 = (x2 - ox) * cos - (y2 - oy) * sin, (y2 - oy) * cos + (x2 - ox) * sin
	local nx3, ny3 = (x3 - ox) * cos - (y3 - oy) * sin, (y3 - oy) * cos + (x3 - ox) * sin
	return nx1 + ox, ny1 + oy, nx2 + ox, ny2 + oy, nx3 + ox, ny3 + oy
end

---@param sx number
---@param sy number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.scale_aabb(sx, sy, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	local x1, y1 = min_x * sx, min_y * sy
	local x2, y2 = max_x * sx, max_y * sy
	return math_min(x1, x2), math_min(y1, y2), math_max(x1, x2), math_max(y1, y2)
end

---@param sx number
---@param sy number
---@param x number
---@param y number
function M.scale_point(sx, sy, x, y)
	return x * sx, y * sy
end

---@param sx number
---@param sy number
---@param points number[] -- (x1, y1, x2, y2, ...)
function M.scale_points(sx, sy, points)
	local result = {}
	for i = 1, #points, 2 do
		table_insert(result, points[i] * sx)
		table_insert(result, points[i + 1] * sy)
	end
	return result
end

---@param sx number
---@param sy number
---@param points number[] -- (x1, y1, x2, y2, ...)
function M.scale_points_in_place(sx, sy, points)
	for i = 1, #points, 2 do
		points[i] = points[i] * sx
		points[i + 1] = points[i + 1] * sy
	end
	return points
end

---@param sx number
---@param sy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number, number, number, number, number, number, number, number
function M.scale_quad(sx, sy, x1, y1, x2, y2, x3, y3, x4, y4)
	return x1 * sx, y1 * sy, x2 * sx, y2 * sy, x3 * sx, y3 * sy, x4 * sx, y4 * sy
end

---@param sx number
---@param sy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number, number, number
function M.scale_segment(sx, sy, x1, y1, x2, y2)
	return x1 * sx, y1 * sy, x2 * sx, y2 * sy
end

---@param sx number
---@param sy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number, number, number, number, number
function M.scale_triangle(sx, sy, x1, y1, x2, y2, x3, y3)
	return x1 * sx, y1 * sy, x2 * sx, y2 * sy, x3 * sx, y3 * sy
end

---@param dx number
---@param dy number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return number, number, number, number
function M.translate_aabb(dx, dy, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return min_x + dx, min_y + dy, max_x + dx, max_y + dy
end

---@param dx number
---@param dy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number, number, number, number, number, number, number, number
function M.translate_quad(dx, dy, x1, y1, x2, y2, x3, y3, x4, y4)
	return x1 + dx, y1 + dy, x2 + dx, y2 + dy, x3 + dx, y3 + dy, x4 + dx, y4 + dy
end

---@param dx number
---@param dy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number, number, number
function M.translate_segment(dx, dy, x1, y1, x2, y2)
	return x1 + dx, y1 + dy, x2 + dx, y2 + dy
end

M.translate_line = M.translate_segment

---@param dx number
---@param dy number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number, number, number, number, number
function M.translate_triangle(dx, dy, x1, y1, x2, y2, x3, y3)
	return x1 + dx, y1 + dy, x2 + dx, y2 + dy, x3 + dx, y3 + dy
end

---@param dx number
---@param dy number
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.translate_points(dx, dy, points)
	local result = {}
	for i = 1, #points, 2 do
		result[i] = points[i] + dx
		result[i + 1] = points[i + 1] + dy
	end
	return result
end

---@param dx number
---@param dy number
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.translate_points_in_place(dx, dy, points)
	for i = 1, #points, 2 do
		points[i] = points[i] + dx
		points[i + 1] = points[i + 1] + dy
	end
	return points
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return number, number, number, number
function M.mirror_aabb(axis_x1, axis_y1, axis_x2, axis_y2, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	-- mirror all corners and return bounding box
	local mx1, my1 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, min_x, min_y)
	local mx2, my2 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, max_x, min_y)
	local mx3, my3 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, max_x, max_y)
	local mx4, my4 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, min_x, max_y)
	return math_min(mx1, math_min(mx2, math_min(mx3, mx4))),
		math_min(my1, math_min(my2, math_min(my3, my4))),
		math_max(mx1, math_max(mx2, math_max(mx3, mx4))),
		math_max(my1, math_max(my2, math_max(my3, my4)))
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x number
---@param y number
---@return number, number
function M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x, y)
	local cx, cy = M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, x, y)
	return project_point_t(x, y, cx, cy, 2.0)
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number, number, number
function M.mirror_segment(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1, x2, y2)
	local mx1, my1 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1)
	local mx2, my2 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x2, y2)
	return mx1, my1, mx2, my2
end

M.mirror_line = M.mirror_segment

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
function M.mirror_triangle(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1, x2, y2, x3, y3)
	local mx1, my1 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1)
	local mx2, my2 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x2, y2)
	local mx3, my3 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x3, y3)
	return mx1, my1, mx2, my2, mx3, my3
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
function M.mirror_quad(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1, x2, y2, x3, y3, x4, y4)
	local mx1, my1 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1)
	local mx2, my2 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x2, y2)
	local mx3, my3 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x3, y3)
	local mx4, my4 = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x4, y4)
	return mx1, my1, mx2, my2, mx3, my3, mx4, my4
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x number
---@param y number
---@param dir_x number
---@param dir_y number
---@return number, number, number, number -- (x,y,dir_x,dir_y)
function M.mirror_ray(axis_x1, axis_y1, axis_x2, axis_y2, x, y, dir_x, dir_y)
	_assert(is_equal(dir_x * dir_x + dir_y * dir_y, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", dir_x, dir_y)
	local mx, my = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x, y)
	local mdir_x, mdir_y = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, x + dir_x, y + dir_y)
	return mx, my, mdir_x - mx, mdir_y - my
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.mirror_points(axis_x1, axis_y1, axis_x2, axis_y2, points)
	local mirrored = {}
	for i = 1, #points, 2 do
		local x, y = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, points[i], points[i + 1])
		table_insert(mirrored, x)
		table_insert(mirrored, y)
	end
	return mirrored
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.mirror_points_in_place(axis_x1, axis_y1, axis_x2, axis_y2, points)
	for i = 1, #points, 2 do
		local x, y = M.mirror_point(axis_x1, axis_y1, axis_x2, axis_y2, points[i], points[i + 1])
		points[i], points[i + 1] = x, y
	end
	return points
end

---@param a {[1]:number,[2]:number}
---@param b {[1]:number,[2]:number}
local function sort_point(a, b)
	return a[1] < b[1] or (a[1] == b[1] and a[2] < b[2])
end

local function project_points_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, ...)
	local input_points = { ... }
	local points = {}
	for i = 1, #input_points/2 do
		points[i] = { M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, input_points[2*(i)-1], input_points[2*(i)]) }
	end
	table_sort(points, sort_point)
	return points[1][1], points[1][2], points[#points][1], points[#points][2]
end

local function sort_two_points(x1, y1, x2, y2)
	if x1 > x2 then
		return x2, y2, x1, y1
	elseif x1 == x2 and y1 > y2 then
		return x2, y2, x1, y1
	else
		return x1, y1, x2, y2
	end
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return number, number, number, number -- (x1, y1, x2, y2)
function M.project_aabb_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return project_points_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, min_x, min_y, max_x, min_y, max_x, max_y, min_x, max_y)
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x number
---@param y number
---@param r number
---@return number, number, number, number -- (x1, y1, x2, y2)
function M.project_circle_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, x, y, r)
	local dx, dy = M.normalize(axis_x2 - axis_x1, axis_y2 - axis_y1)
	if dx < 0 then
		dx, dy = -dx, -dy
	end
	local cx, cy = M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, x, y)
	return cx - dx*r, cy - dy*r, cx + dx*r, cy + dy*r
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x number
---@param y number
---@return number, number
function M.project_point_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, x, y)
	return M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, x, y)
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number, number, number -- (x1, y1, x2, y2)
function M.project_triangle_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1, x2, y2, x3, y3)
	return project_points_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1, x2, y2, x3, y3)
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number, number, number, number -- (x1, y1, x2, y2)
function M.project_quad_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1, x2, y2, x3, y3, x4, y4)
	return project_points_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1, x2, y2, x3, y3, x4, y4)
end

---Project ray onto axis, returning a point (x,y) and a projected direction (dir_x, dir_y) which may not be normalized
---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x number
---@param y number
---@param dir_x number
---@param dir_y number
---@return number, number, number, number -- (x, y, proj_dir_x, proj_dir_y)
function M.project_ray_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, x, y, dir_x, dir_y)
	_assert(is_equal(dir_x * dir_x + dir_y * dir_y, 1), "bad argument (vector (%.2f, %.2f) is not normalized)", dir_x, dir_y)
	local px, py = M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, x, y)
	local pdx, pdy = M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, x + dir_x, y + dir_y)
	return px, py, pdx - px, pdy - py
end


---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param point_count integer -- >= 2
---@param pcurve fun(t:number):number,number
---@return number, number, number, number -- (x1, y1, x2, y2)
function M.project_pcurve_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, point_count, pcurve)
	local result = {
		{ M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, pcurve(0)) }
	}
	for i = 1, point_count-1 do
		result[i + 1] = { M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, pcurve(i / (point_count - 1))) }
	end
	table_sort(result, sort_point)
	return result[1][1], result[1][2], result[#result][1], result[#result][2]
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number, number, number, number -- (x1, y1, x2, y2)
function M.project_points_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, points)
	local result = {}
	for i = 1, #points/2 do
		result[i] = { M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, points[2*(i)-1], points[2*(i)]) }
	end
	table_sort(result, sort_point)
	return result[1][1], result[1][2], result[#result][1], result[#result][2]
end

---@param axis_x1 number
---@param axis_y1 number
---@param axis_x2 number
---@param axis_y2 number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number, number, number -- (x1, y1, x2, y2)
function M.project_segment_onto_axis(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1, x2, y2)
	local px1, py1 = M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, x1, y1)
	local px2, py2 = M.closest_point_on_line_from_point(axis_x1, axis_y1, axis_x2, axis_y2, x2, y2)
	return sort_two_points(px1, py1, px2, py2)
end

-----------------------------------------------------------
-- Beziers, Parametric Curves, Polylines
-----------------------------------------------------------

---Return the cofficients for a quadratic bezier curve `(ax,ay)t^2 + (bx,by)t + (x1,y1)`
---@see geo2d_module.evaluate_quadratic_equation
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number, number, number -- ax, bx, ay, by
function M.quadratic_bezier_coefficients(x1, y1, x2, y2, x3, y3)
	local bx = 2.0 * (x2 - x1)
	local by = 2.0 * (y2 - y1)
	local ax = x3 - x1 - bx
	local ay = y3 - y1 - by
	return ax, bx, ay, by
end

---Return the cofficients for a cubic bezier curve `(ax,ay)t^3 + (bx,by)t^2 + (cx,cy)t + (x1,y1)`
---@see geo2d_module.evaluate_cubic_equation
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number, number, number, number, number, number -- ax, bx, cx, ay, by, cy
function M.cubic_bezier_coefficients(x1, y1, x2, y2, x3, y3, x4, y4)
	local cx = 3.0 * (x2 - x1)
	local cy = 3.0 * (y2 - y1)
	local bx = 3.0 * (x3 - x2) - cx
	local by = 3.0 * (y3 - y2) - cy
	local ax = x4 - x1 - cx - bx
	local ay = y4 - y1 - cy - by
	return ax, bx, cx, ay, by, cy
end

---Evaluate quadratic bezier at position t from inital point (x1, y1) and coefficients
---@see geo2d_module.quadratic_bezier_coefficients
---@param t number
---@param x1 number
---@param y1 number
---@param ax number
---@param bx number
---@param ay number
---@param by number
---@return number, number
function M.evaluate_quadratic_equation(t, x1, y1, ax, bx, ay, by)
	local tsqr = t * t
	local x = (ax * tsqr) + (bx * t) + x1
	local y = (ay * tsqr) + (by * t) + y1
	return x, y
end

---Evaluate cubic bezier at position t from inital point (x1, y1) and coefficients
---@see geo2d_module.quadratic_cubic_coefficients
---@param t number
---@param x1 number
---@param y1 number
---@param ax number
---@param bx number
---@param cx number
---@param ay number
---@param by number
---@param cy number
---@return number, number
function M.evaluate_cubic_equation(t, x1, y1, ax, bx, cx, ay, by, cy)
	local tsqr = t * t
	local tcube = tsqr * t
	local x = (ax * tcube) + (bx * tsqr) + (cx * t) + x1
	local y = (ay * tcube) + (by * tsqr) + (cy * t) + y1
	return x, y
end

---Evaluate quadratic bezier at position t from control points
---@param t number -- [0,1]
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number
function M.evaluate_quadratic_bezier(t, x1, y1, x2, y2, x3, y3)
	local ax, bx, ay, by = M.quadratic_bezier_coefficients(x1, y1, x2, y2, x3, y3)
	return M.evaluate_quadratic_equation(t, x1, y1, ax, bx, ay, by)
end

---Evaluate the cubic bezier curve at position t from control points
---@param t number -- [0,1]
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number, number
function M.evaluate_cubic_bezier(t, x1, y1, x2, y2, x3, y3, x4, y4)
	local ax, bx, cx, ay, by, cy = M.cubic_bezier_coefficients(x1, y1, x2, y2, x3, y3, x4, y4)
	return M.evaluate_cubic_equation(t, x1, y1, ax, bx, cx, ay, by, cy)
end

---Returns a parametric curve function (t)->(x,y) from the control points of a quadratic bezier curve
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return fun(t:number):number,number
function M.pcurve_from_quadratic_bezier(x1, y1, x2, y2, x3, y3)
	local ax, bx, ay, by = M.quadratic_bezier_coefficients(x1, y1, x2, y2, x3, y3)
	return function(t)
		return M.evaluate_quadratic_equation(t, x1, y1, ax, bx, ay, by)
	end
end

---Returns a parametric curve function (t)->(x,y) from the control points of a cubic bezier curve
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return fun(t:number):number,number
function M.pcurve_from_cubic_bezier(x1, y1, x2, y2, x3, y3, x4, y4)
	local ax, bx, cx, ay, by, cy = M.cubic_bezier_coefficients(x1, y1, x2, y2, x3, y3, x4, y4)
	return function(t)
		return M.evaluate_cubic_equation(t, x1, y1, ax, bx, cx, ay, by, cy)
	end
end

---Returns a parametric curve function (t)->(x,y) from the a start point (x1, y1) with velocity (dx1, dy1) and end point (x2, y2) with velocity (dx2, dy2)
---@param x1 number
---@param y1 number
---@param dx1 number
---@param dy1 number
---@param x2 number
---@param y2 number
---@param dx2 number
---@param dy2 number
function M.pcurve_from_cubic_hermite(x1, y1, dx1, dy1, x2, y2, dx2, dy2)
	return M.pcurve_from_cubic_bezier(x1, y1, x1 + dx1 / 3, y1 + dy1 / 3, x2 - dx2 / 3, y2 - dy2 / 3, x2, y2)
end

---Creates a polyline (x1, y1, x2, y2, ...) by sampling a cubic bezier curve
---@param point_count integer -- >= 2
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number[] -- (x1, y1, x2, y2, ...)
function M.polyline_from_cubic_bezier(point_count, x1, y1, x2, y2, x3, y3, x4, y4)
	assert(point_count >= 2, "bad argument (point_count must be >= 2)")

	local t = 0.0
	local dt = 1.0 / (1.0 * point_count - 1.0)
	local ax, bx, cx, ay, by, cy = M.cubic_bezier_coefficients(x1, y1, x2, y2, x3, y3, x4, y4)

	local points = {}
	for i = 1, point_count do
		local x, y = M.evaluate_cubic_equation(t, x1, y1, ax, bx, cx, ay, by, cy)
		table_insert(points, x)
		table_insert(points, y)
		t = t + dt
	end
	return points
end

---Creates a polyline (x1, y1, x2, y2, ...) by sampling a quadratic bezier curve
---@param point_count integer -- >= 2
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number[] -- (x1, y1, x2, y2, ...)
function M.polyline_from_quadratic_bezier(point_count, x1, y1, x2, y2, x3, y3)
	assert(point_count >= 2, "bad argument (point_count must be >= 2)")

	local t = 0.0
	local dt = 1.0 / (1.0 * point_count - 1.0)
	local ax, bx, ay, by = M.quadratic_bezier_coefficients(x1, y1, x2, y2, x3, y3)

	local points = {}
	for i = 1, point_count do
		local x, y = M.evaluate_quadratic_equation(t, x1, y1, ax, bx, ay, by)
		table_insert(points, x)
		table_insert(points, y)
		t = t + dt
	end

	return points
end

---Creates a polyline (x1, y1, x2, y2, ...) by sampling a cubic hermite curve with  a start point (x1, y1) with velocity (dx1, dy1) and end point (x2, y2) with velocity (dx2, dy2)
---@param point_count integer -- >= 2
---@param x1 number
---@param y1 number
---@param dx1 number
---@param dy1 number
---@param x2 number
---@param y2 number
---@param dx2 number
---@param dy2 number
---@return number[] -- (x1, y1, x2, y2, ...)
function M.polyline_from_cubic_hermite(point_count, x1, y1, dx1, dy1, x2, y2, dx2, dy2)
	return M.polyline_from_cubic_bezier(point_count, x1, y1, x1 + dx1 / 3, y1 + dy1 / 3, x2 - dx2 / 3, y2 - dy2 / 3, x2, y2)
end

---Helper function to test segments of a bezier curve for intersection
---@param point_count integer -- >= 2
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@param segment_intersector fun(x1:number, y1:number, x2:number, y2:number, ...):boolean
---@return boolean
---@package
---@private
function M._intersect_cubic_bezier_shape(point_count, x1, y1, x2, y2, x3, y3, x4, y4, segment_intersector, ...)
	local ax, bx, cx, ay, by, cy = M.cubic_bezier_coefficients(x1, y1, x2, y2, x3, y3, x4, y4)
	local function sampler(t)
		return M.evaluate_cubic_equation(t, x1, y1, ax, bx, cx, ay, by, cy)
	end
	return M._intersect_pcurve(point_count, sampler, segment_intersector, ...)
end

---Helper function to test segments of a bezier curve for intersection
---@param point_count integer -- >= 2
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param segment_intersector fun(x1:number, y1:number, x2:number, y2:number, ...):boolean
---@return boolean
---@package
---@private
function M._intersect_quadratic_bezier_shape(point_count, x1, y1, x2, y2, x3, y3, segment_intersector, ...)
	local ax, bx, ay, by = M.quadratic_bezier_coefficients(x1, y1, x2, y2, x3, y3)
	local function sampler(t)
		return M.evaluate_quadratic_equation(t, x1, y1, ax, bx, ay, by)
	end
	return M._intersect_pcurve(point_count, sampler, segment_intersector, ...)
end

---Helper function to test segments of a parametric curve
---@param point_count integer -- >= 2
---@param sampler fun(t:number):number,number
---@param segment_intersector fun(x1:number, y1:number, x2:number, y2:number, ...):boolean
---@package
---@private
function M._intersect_pcurve(point_count, sampler, segment_intersector, ...)
	assert(point_count >= 2, "bad argument (point_count must be >= 2)")
	local t = 0.0
	local dt = 1.0 / (1.0 * point_count - 1.0)
	local prev_x, prev_y = sampler(t)
	t = dt
	for i = 1, point_count - 1 do
		local x, y = sampler(t)
		if segment_intersector(prev_x, prev_y, x, y, ...) then
			return true
		end
		prev_x, prev_y = x, y
		t = t + dt
	end
	return false
end

---Helper function to test points of a parametric curve
---@param point_count integer -- >= 2
---@param sampler fun(t:number):number,number
---@param point_checker fun(x1:number, y1:number, ...):boolean
---@package
---@private
function M._contain_pcurve(point_count, sampler, point_checker, ...)
	assert(point_count >= 2, "bad argument (point_count must be >= 2)")
	local t = 0.0
	local dt = 1.0 / (point_count - 1.0)
	for i = 1, point_count do
		local x, y = sampler(t)
		if point_checker(x, y, ...) then
			return true
		end
		t = t + dt
	end
	return false
end

---Helper function to test segments of a bezier curve
---@param point_count integer -- >= 2
---@param sampler fun(t:number):number,number
---@param segment_intersector fun(x1:number, y1:number, x2:number, y2:number, ...):number?,number?,number?,number?
---@return number[] -- (x1, y1, x2, y2, ...)
---@package
---@private
function M._intersection_point_pcurve(point_count, sampler, segment_intersector, ...)
	assert(point_count >= 2, "bad argument (point_count must be >= 2)")
	local t = 0.0
	local dt = 1.0 / (point_count - 1.0)
	local prev_x, prev_y = sampler(t)
	t = dt
	local points = {}
	for i = 1, point_count - 1 do
		local x, y = sampler(t)
		local ix1, iy1, ix2, iy2 = segment_intersector(prev_x, prev_y, x, y, ...)
		if type(ix1) == "table" then
			for j = 1, #ix1, 2 do
				table_insert(points, ix1[j])
				table_insert(points, ix1[j + 1])
			end
		elseif ix1 and iy1 then
			table_insert(points, ix1)
			table_insert(points, iy1)
			if ix2 and iy2 then
				table_insert(points, ix2)
				table_insert(points, iy2)
			end
		end
		prev_x, prev_y = x, y
		t = t + dt
	end
	return points
end

---Helper function to test intersection of segments of a polyline
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param segment_intersector fun(x1:number, y1:number, x2:number, y2:number, ...):boolean
---@package
---@private
function M._intersect_polyline(polyline, segment_intersector, ...)
	assert(#polyline >= 4, "bad argument (polyline must have at least 2 vertices)")
	local prev_x, prev_y = polyline[1], polyline[2]
	for i = 2, #polyline / 2 do
		local x, y = polyline[2*(i)-1], polyline[2*(i)]
		if segment_intersector(prev_x, prev_y, x, y, ...) then
			return true
		end
		prev_x, prev_y = x, y
	end
	return false
end

---Return true if any shape_points are inside polygon
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param shape_points number[] -- (x1, y1, x2, y2, ...)
---@package
---@private
function M._polygon_contains_points(polygon, shape_points)
	if M._contain_points(shape_points, M.point_in_polygon, polygon) then
		return true
	end
	return false
end

---Helper function to test intersection of segments with outline of a polygon
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param segment_intersector fun(x1:number, y1:number, x2:number, y2:number, ...):boolean
---@package
---@private
function M._intersect_polygon_segments(polygon, segment_intersector, ...)
	assert(#polygon >= 4, "bad argument (polygon must have at least 2 vertices)")
	-- Test segments for intersection
	local prev_x, prev_y = polygon[#polygon-1], polygon[#polygon]
	for i = 1, #polygon / 2 do
		local x, y = polygon[2*(i)-1], polygon[2*(i)]
		if segment_intersector(prev_x, prev_y, x, y, ...) then
			return true
		end
		prev_x, prev_y = x, y
	end
	return false
end

---Helper function to test point containment
---@param points number[] -- (x1, y1, x2, y2, ...)
---@param point_checker fun(x1:number, y1:number, ...):boolean
---@package
---@private
function M._contain_points(points, point_checker, ...)
	assert(#points >= 2, "bad argument (points must have at least 1 vertex)")
	for i = 1, #points, 2 do
		local x, y = points[i], points[i+1]
		if point_checker(x, y, ...) then
			return true
		end
	end
	return false
end

---Helper function to get intersection points in segments of a polyline
---@param polyline number[] -- (x1, y1, x2, y2, ...)
---@param segment_intersector fun(x1:number, y1:number, x2:number, y2:number, ...):number?,number?,number?,number?
---@return number[] -- (x1, y1, x2, y2, ...)
---@package
---@private
function M._intersection_point_polyline(polyline, segment_intersector, ...)
	assert(#polyline >= 4, "bad argument (polyline must have at least 2 vertices)")
	local prev_x, prev_y = polyline[1], polyline[2]
	local result = {}
	local result_index = 1
	for i = 2, #polyline / 2 do
		local x, y = polyline[2*(i)-1], polyline[2*(i)]
		local ix1, iy1, ix2, iy2 = segment_intersector(prev_x, prev_y, x, y, ...)
		if type(ix1) == "table" then
			for j = 1, #ix1, 2 do
				result[result_index] = ix1[j]
				result[result_index + 1] = ix1[j + 1]
				result_index = result_index + 2
			end
		elseif ix1 and iy1 then
			result[result_index] = ix1
			result[result_index + 1] = iy1
			result_index = result_index + 2
			if ix2 and iy2 then
				result[result_index] = ix2
				result[result_index + 1] = iy2
				result_index = result_index + 2
			end
		end
		prev_x, prev_y = x, y
	end
	return result
end


---Helper function to get intersection points in segments of a polygon
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param segment_intersector fun(x1:number, y1:number, x2:number, y2:number, ...):number?,number?,number?,number?
---@return number[] -- (x1, y1, x2, y2, ...)
---@package
---@private
function M._intersection_point_polygon(polygon, segment_intersector, ...)
	assert(#polygon >= 4, "bad argument (polygon must have at least 2 vertices)")
	local prev_x, prev_y = polygon[#polygon-1], polygon[#polygon]
	local result = {}
	local result_index = 1
	for i = 1, #polygon / 2 do
		local x, y = polygon[2*(i)-1], polygon[2*(i)]
		local ix1, iy1, ix2, iy2 = segment_intersector(prev_x, prev_y, x, y, ...)
		if type(ix1) == "table" then
			for j = 1, #ix1, 2 do
				result[result_index] = ix1[j]
				result[result_index + 1] = ix1[j + 1]
				result_index = result_index + 2
			end
		elseif ix1 and iy1 then
			result[result_index] = ix1
			result[result_index + 1] = iy1
			result_index = result_index + 2
			if ix2 and iy2 then
				result[result_index] = ix2
				result[result_index + 1] = iy2
				result_index = result_index + 2
			end
		end
		prev_x, prev_y = x, y
	end
	return result
end

-----------------------------------------------------------
-- Convexity
-----------------------------------------------------------

---@package
---@private
function M._quad_is_valid(x1, y1, x2, y2, x3, y3, x4, y4)
	return M.orientation(x1, y1, x2, y2, x3, y3) == M.orientation(x1, y1, x2, y2, x4, y4) and
		M.orientation(x3, y3, x4, y4, x1, y1) == M.orientation(x3, y3, x4, y4, x2, y2)
end

-----------------------------------------------------------
-- Randomness
-----------------------------------------------------------

---@param rng fun():number -- Random function returning value in [0,1]
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.random_point_in_aabb(rng, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	local width, height = max_x - min_x, max_y - min_y
	local random_x = rng() * width
	local random_y = rng() * height
	return min_x + random_x, min_y + random_y
end

---@param rng fun():number -- Random function returning value in [0,1]
---@param x number
---@param y number
---@param radius number
function M.random_point_in_circle(rng, x, y, radius)
	local r = radius * math_sqrt(rng())
	local theta = rng() * PI_MUL_2
	return x + (r * math_cos(theta)), y + (r * math_sin(theta))
end

---@param rng fun():number -- Random function returning value in [0,1]
---@param count integer
---@param polyline number[] -- (x1, y1, x2, y2, ...)
function M.random_points_in_polyline(rng, count, polyline)
	assert(#polyline >= 4, "bad argument (polyline must have at least 2 vertices)")
	-- Calculate cumulative length of each segment and build weighted array
	local total_length = 0
	local segment_lengths = {}
	local npoints = #polyline / 2
	local prev_x, prev_y = polyline[2*(1)-1], polyline[2*(1)]
	for i = 1, npoints - 1 do
		local x, y = polyline[2*(i + 1)-1], polyline[2*(i + 1)]
		local dx, dy = prev_x - x, prev_y - y
		local length = math_sqrt(dx * dx + dy * dy)
		prev_x, prev_y = x, y
		total_length = total_length + length
		table_insert(segment_lengths, total_length)
	end

	local result = {}
	for _ = 1, count do
		local random_length = rng() * total_length
		local i = 1
		for j = 1, #segment_lengths do
			if segment_lengths[j] >= random_length then
				i = j
				break
			end
		end
		local x1, y1 = polyline[2*(i)-1], polyline[2*(i)]
		local x2, y2 = polyline[2*(i + 1)-1], polyline[2*(i + 1)]
		local t = rng()
		local x = x1 + (x2 - x1) * t
		local y = y1 + (y2 - y1) * t
		table_insert(result, x)
		table_insert(result, y)
	end
	return result
end

---Generates random points within a polygon
---Notes:
---* Polygon is triangulated
---
---@see geo2d_module.random_point_in_triangle
---@param rng fun():number -- Random function returning value in [0,1]
---@param count integer
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@return number[]
function M.random_points_in_polygon(rng, count, polygon)
	assert(#polygon >= 6, "bad argument (polygon must have at least 3 vertices)")
	if #polygon == 6 then
		return { M.random_point_in_triangle(rng, polygon[1], polygon[2], polygon[3], polygon[4], polygon[5],
			polygon[6]) }
	end
	local triangles = M.triangles_from_polygon(polygon)
	assert(triangles, "bad argument (cannot triangulate polygon)")
	return M.random_points_in_triangles(rng, count, triangles)
end

---@param rng fun():number -- Random function returning value in [0,1]
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@param x4 number
---@param y4 number
---@return number, number
function M.random_point_in_quad(rng, x1, y1, x2, y2, x3, y3, x4, y4)
	local a, b = (2 * rng()) - 1, (2 * rng()) - 1
	local a1, a2 = 1 - a, 1 + a
	local b1, b2 = 1 - b, 1 + b
	local r1, r2, r3, r4 = a1 * b1, a2 * b1, a2 * b2, a1 * b2
	local px = ((r1 * x1) + (r2 * x2) + (r3 * x3) + (r4 * x4)) / 4
	local py = ((r1 * y1) + (r2 * y2) + (r3 * y3) + (r4 * y4)) / 4
	return px, py
end

---@param rng fun():number -- Random function returning value in [0,1]
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
function M.random_point_in_segment(rng, x1, y1, x2, y2)
	local t = rng()
	return x1 + (x2 - x1) * t, y1 + (y2 - y1) * t
end

---Generates random points within a set of triangles
---@param rng fun():number -- Random function returning value in [0,1]
---@param count integer -- Number of points to generate
---@param triangles number[] -- (x1, y1, x2, y2, x3, y3, ...)
---@return number[]
function M.random_points_in_triangles(rng, count, triangles)
	-- Calculate area of each triangle and select random triangle weighted by area
	local area_sums = {}
	local total_area = 0
	for i = 1, #triangles, 6 do
		local x1, y1, x2, y2, x3, y3 = triangles[i], triangles[i + 1], triangles[i + 2], triangles[i + 3],
			triangles[i + 4], triangles[i + 5]
		local area = math_abs(((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / 2)
		total_area = total_area + area
		table_insert(area_sums, total_area)
	end

	local result = {}
	for _ = 1, count do
		local random_area = rng() * total_area
		for i = 1, #area_sums do
			if area_sums[i] >= random_area then
				local index = 1 + (i - 1) * 6
				local px, py = M.random_point_in_triangle(rng, triangles[index], triangles[index + 1],
					triangles[index + 2], triangles[index + 3], triangles[index + 4], triangles[index + 5])
				table_insert(result, px)
				table_insert(result, py)
				break
			end
		end
	end
	return result
end

---@param rng fun():number -- Random function returning value in [0,1]
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number
function M.random_point_in_triangle(rng, x1, y1, x2, y2, x3, y3)
	local a, b = rng(), rng()
	if (a + b) > 1 then
		a, b = 1 - a, 1 - b
	end
	local c = 1 - a - b
	local px = (x1 * a) + (x2 * b) + (x3 * c)
	local py = (y1 * a) + (y2 * b) + (y3 * c)
	return px, py
end

-----------------------------------------------------------
-- Clipping
-----------------------------------------------------------

---@param x number
---@param y number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return boolean,boolean,boolean,boolean -- top, bottom, left, right
local function clip_point(x, y, min_x, min_y, max_x, max_y)
	local top, bottom, left, right = false, false, false, false
	if y < min_y then
		top = true
	elseif y > max_y then
		bottom = true
	end

	if x < min_x then
		left = true
	elseif x > max_x then
		right = true
	end

	return top, bottom, left, right
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
---@return number?, number?, number?, number?
function M.clip_segment_aabb(x1, y1, x2, y2, min_x, min_y, max_x, max_y)
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)

	local clip1_t, clip1_b, clip1_l, clip1_r = clip_point(x1, y1, min_x, min_y, max_x, max_y)
	local clip2_t, clip2_b, clip2_l, clip2_r = clip_point(x2, y2, min_x, min_y, max_x, max_y)
	local clip_t, clip_b, clip_l, clip_r = false, false, false, false

	local x, y = 0, 0
	while clip1_t or clip1_b or clip1_l or clip1_r or clip2_t or clip2_b or clip2_l or clip2_r do
		if (clip1_b and clip2_b) or (clip1_t and clip2_t) or (clip1_l and clip2_l) or (clip1_r and clip2_r) then
			return
		else
			if clip1_t or clip1_b or clip1_l or clip1_r then
				clip_t, clip_b, clip_l, clip_r = clip1_t, clip1_b, clip1_l, clip1_r
			else
				clip_t, clip_b, clip_l, clip_r = clip2_t, clip2_b, clip2_l, clip2_r
			end

			local dx, dy = x2 - x1, y2 - y1

			if clip_t then
				x = x1 + dx * (min_y - y1) / dy
				y = min_y
			elseif clip_b then
				x = x1 + dx * (max_y - y1) / dy
				y = max_y
			elseif clip_r then
				y = y1 + dy * (max_x - x1) / dx
				x = max_x
			elseif clip_l then
				y = y1 + dy * (min_x - x1) / dx
				x = min_x
			end

			if clip_l == clip1_l and clip_r == clip1_r and clip_t == clip1_t and clip_b == clip1_b then
				x1, y1 = x, y

				clip1_t, clip1_b, clip1_l, clip1_r = clip_point(x1, y1, min_x, min_y, max_x, max_y)
			else
				x2, y2 = x, y
				clip2_t, clip2_b, clip2_l, clip2_r = clip_point(x2, y2, min_x, min_y, max_x, max_y)
			end
		end
	end

	return x1, y1, x2, y2
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param cx number
---@param cy number
---@param r number
---@return number?, number?, number?, number?
function M.clip_segment_circle(x1, y1, x2, y2, cx, cy, r)
	local p1_in_circle = M.point_in_circle(x1, y1, cx, cy, r)
	local p2_in_circle = M.point_in_circle(x2, y2, cx, cy, r)
	if p1_in_circle and p2_in_circle then
		return x1, y1, x2, y2
	end
	local ix1, iy1, ix2, iy2 = M.intersection_point_segment_circle(x1, y1, x2, y2, cx, cy, r)
	if ix1 and iy1 then
		if ix2 and iy2 then
			return ix1, iy1, ix2, iy2
		else
			if p1_in_circle then
				return x1, y1, ix1, iy1
			elseif p2_in_circle then
				return x2, y2, ix1, iy1
			end
		end
	end
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param tri_x1 number
---@param tri_y1 number
---@param tri_x2 number
---@param tri_y2 number
---@param tri_x3 number
---@param tri_y3 number
---@return number?, number?, number?, number?
function M.clip_segment_triangle(x1, y1, x2, y2, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3)
	do
		-- Check for overlap with each triangle edge
		local are_points_equal_x1_y1_x1_y1 = are_points_equal(x1, y1, tri_x1, tri_y1)
		local are_points_equal_x1_y1_x2_y2 = are_points_equal(x1, y1, tri_x2, tri_y2)
		local are_points_equal_x1_y1_x3_y3 = are_points_equal(x1, y1, tri_x3, tri_y3)
		local are_points_equal_x2_y2_x1_y1 = are_points_equal(x2, y2, tri_x1, tri_y1)
		local are_points_equal_x2_y2_x2_y2 = are_points_equal(x2, y2, tri_x2, tri_y2)
		local are_points_equal_x2_y2_x3_y3 = are_points_equal(x2, y2, tri_x3, tri_y3)

		if (are_points_equal_x1_y1_x1_y1 and are_points_equal_x2_y2_x2_y2) or
			(are_points_equal_x1_y1_x2_y2 and are_points_equal_x2_y2_x3_y3) or
			(are_points_equal_x1_y1_x3_y3 and are_points_equal_x2_y2_x1_y1) or
			(are_points_equal_x1_y1_x2_y2 and are_points_equal_x2_y2_x1_y1) or
			(are_points_equal_x1_y1_x3_y3 and are_points_equal_x2_y2_x2_y2) or
			(are_points_equal_x1_y1_x1_y1 and are_points_equal_x2_y2_x3_y3) then
			return x1, y1, x2, y2
		end
	end

	if not M.intersect_segment_triangle(x1, y1, x2, y2, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3) then
		return
	end

	local npoints = 0
	local px1 = x1
	local py1 = y1
	local px2 = x2
	local py2 = y2

	local x, y = M.intersection_point_segment_segment(x1, y1, x2, y2, tri_x1, tri_y1, tri_x2, tri_y2)
	if x and y then
		px1 = x
		py1 = y
		npoints = npoints + 1
	end

	x, y = M.intersection_point_segment_segment(x1, y1, x2, y2, tri_x2, tri_y2, tri_x3, tri_y3)
	if x and y then
		if npoints == 0 then
			px1 = x
			py1 = y
		else
			px2 = x
			py2 = y
		end
		npoints = npoints + 1
	end

	if (npoints < 2) then
		x, y = M.intersection_point_segment_segment(x1, y1, x2, y2, tri_x3, tri_y3, tri_x1, tri_y1)
		if x and y then
			if npoints == 0 then
				px1 = x
				py1 = y
			else
				px2 = x
				py2 = y
			end
			npoints = npoints + 1
		end
	end

	if npoints == 1 then
		if M.point_in_triangle(x1, y1, tri_x1, tri_y1, tri_x2, tri_y2, tri_x3, tri_y3) then
			return px1, py1, x1, y1
		else
			return px1, py1, x2, y2
		end
	end

	return px1, py1, px2, py2
end

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param quad_x1 number
---@param quad_y1 number
---@param quad_x2 number
---@param quad_y2 number
---@param quad_x3 number
---@param quad_y3 number
---@param quad_x4 number
---@param quad_y4 number
---@return number?, number?, number?, number?
function M.clip_segment_quad(x1, y1, x2, y2, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4)
	do
		-- Check for overlap with each quad edge
		local are_points_equal_x1_y1_x1_y1 = are_points_equal(x1, y1, quad_x1, quad_y1)
		local are_points_equal_x1_y1_x2_y2 = are_points_equal(x1, y1, quad_x2, quad_y2)
		local are_points_equal_x1_y1_x3_y3 = are_points_equal(x1, y1, quad_x3, quad_y3)
		local are_points_equal_x1_y1_x4_y4 = are_points_equal(x1, y1, quad_x4, quad_y4)
		local are_points_equal_x2_y2_x1_y1 = are_points_equal(x2, y2, quad_x1, quad_y1)
		local are_points_equal_x2_y2_x2_y2 = are_points_equal(x2, y2, quad_x2, quad_y2)
		local are_points_equal_x2_y2_x3_y3 = are_points_equal(x2, y2, quad_x3, quad_y3)
		local are_points_equal_x2_y2_x4_y4 = are_points_equal(x2, y2, quad_x4, quad_y4)

		if (are_points_equal_x1_y1_x1_y1 and are_points_equal_x2_y2_x2_y2) or
			(are_points_equal_x1_y1_x2_y2 and are_points_equal_x2_y2_x3_y3) or
			(are_points_equal_x1_y1_x3_y3 and are_points_equal_x2_y2_x4_y4) or
			(are_points_equal_x1_y1_x4_y4 and are_points_equal_x2_y2_x1_y1) or
			(are_points_equal_x1_y1_x2_y2 and are_points_equal_x2_y2_x1_y1) or
			(are_points_equal_x1_y1_x3_y3 and are_points_equal_x2_y2_x2_y2) or
			(are_points_equal_x1_y1_x4_y4 and are_points_equal_x2_y2_x3_y3) or
			(are_points_equal_x1_y1_x1_y1 and are_points_equal_x2_y2_x4_y4) then
			return x1, y1, x2, y2
		end
	end

	if not M.intersect_segment_quad(x1, y1, x2, y2, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4) then
		return
	end

	local npoints = 0
	local px1 = x1
	local py1 = y1
	local px2 = x2
	local py2 = y2

	local x, y = M.intersection_point_segment_segment(x1, y1, x2, y2, quad_x1, quad_y1, quad_x2, quad_y2)
	if x and y then
		px1 = x
		py1 = y
		npoints = npoints + 1
	end

	x, y = M.intersection_point_segment_segment(x1, y1, x2, y2, quad_x2, quad_y2, quad_x3, quad_y3)
	if x and y then
		if npoints == 0 then
			px1 = x
			py1 = y
		else
			px2 = x
			py2 = y
		end
		npoints = npoints + 1
	end

	if (npoints < 2) then
		x, y = M.intersection_point_segment_segment(x1, y1, x2, y2, quad_x3, quad_y3, quad_x4, quad_y4)
		if x and y then
			if npoints == 0 then
				px1 = x
				py1 = y
			else
				px2 = x
				py2 = y
			end
			npoints = npoints + 1
		end
	end

	if (npoints < 2) then
		x, y = M.intersection_point_segment_segment(x1, y1, x2, y2, quad_x4, quad_y4, quad_x1, quad_y1)
		if x and y then
			if npoints == 0 then
				px1 = x
				py1 = y
			else
				px2 = x
				py2 = y
			end
			npoints = npoints + 1
		end
	end

	if npoints == 1 then
		if M.point_in_quad(x1, y1, quad_x1, quad_y1, quad_x2, quad_y2, quad_x3, quad_y3, quad_x4, quad_y4) then
			return px1, py1, x1, y1
		else
			return px1, py1, x2, y2
		end
	end

	return px1, py1, px2, py2
end

-----------------------------------------------------------
-- Polygon clipping
-----------------------------------------------------------

---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number, number -- (a, b, c)
local function half_plane_edge(x1, y1, x2, y2)
	local a = y2 - y1
	local b = x1 - x2
	local c = -a * x1 - b * y1
	return a, b, c
end

---@param edge_a number
---@param edge_b number
---@param edge_c number
---@param x number
---@param y number
---@return boolean
local function half_plane_edge_inside(edge_a, edge_b, edge_c, x, y)
	return 0.0 < (edge_a * x + edge_b * y + edge_c)
end

---@param edge_a number
---@param edge_b number
---@param edge_c number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number
local function half_plane_intersection_point(edge_a, edge_b, edge_c, x1, y1, x2, y2)
	local d = y2 - y1
	local e = x1 - x2
	local f = -d * x1 - e * y1
	local ratio = 1.0 / (e * edge_a - edge_b * d)
	return (edge_b * f - e * edge_c) * ratio, (d * edge_c - edge_a * f) * ratio
end

---@param edge_a number
---@param edge_b number
---@param edge_c number
---@param input_poly number[] -- (x1, y1, x2, y2, ...)
---@param clipped_poly number[] -- (x1, y1, x2, y2, ...)
local function clip_polygon_against_edge(edge_a, edge_b, edge_c, input_poly, clipped_poly)
	local npoints = #input_poly / 2
	if npoints < 2 then
		for i = 1, #input_poly do
			input_poly[i] = nil
		end
		for i = 1, #clipped_poly do
			clipped_poly[i] = nil
		end
		return
	end

	local prev_x, prev_y = input_poly[2*(npoints)-1], input_poly[2*(npoints)]
	local nclipped = #clipped_poly
	for i = 1, npoints do
		local curr_x, curr_y = input_poly[2*(i)-1], input_poly[2*(i)]
		local current_point_in = half_plane_edge_inside(edge_a, edge_b, edge_c, curr_x, curr_y)
		local previous_point_in = half_plane_edge_inside(edge_a, edge_b, edge_c, prev_x, prev_y)

		if current_point_in and previous_point_in then
			clipped_poly[nclipped + 1] = curr_x
			clipped_poly[nclipped + 2] = curr_y
			nclipped = nclipped + 2
		elseif not current_point_in and previous_point_in then
			local x, y = half_plane_intersection_point(edge_a, edge_b, edge_c, curr_x, curr_y, prev_x, prev_y)
			clipped_poly[nclipped + 1] = x
			clipped_poly[nclipped + 2] = y
			nclipped = nclipped + 2
		elseif current_point_in and not previous_point_in then
			local x, y = half_plane_intersection_point(edge_a, edge_b, edge_c, prev_x, prev_y, curr_x, curr_y)
			clipped_poly[nclipped + 1] = x
			clipped_poly[nclipped + 2] = y
			clipped_poly[nclipped + 3] = curr_x
			clipped_poly[nclipped + 4] = curr_y
			nclipped = nclipped + 4
		end

		prev_x, prev_y = curr_x, curr_y
	end

	for i = 1, #input_poly do
		input_poly[i] = nil
	end
end

---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param edge_list number[] -- (e1x1, e1y1, e1x2, e1y2, e2x1, e2y1, e2x2, e2y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
local function clip_polygon_from_edge_list(polygon, edge_list)
	assert(#polygon > 5, "bad argument (polygon must have at least 3 vertices)")
	assert(#edge_list > 11, "bad argument (edge_list must have at least 3 edges)")

	local clip_poly1 = {}
	local clip_poly2 = {}

	for i = 1, #polygon do
		table_insert(clip_poly1, polygon[i])
	end

	local nedges = #edge_list / 4
	for i = 1, nedges do
		local p1x, p1y = edge_list[4 * i - 3], edge_list[4 * i - 2]
		local p2x, p2y = edge_list[4 * i - 1], edge_list[4 * i]
		local edge_a, edge_b, edge_c = half_plane_edge(p1x, p1y, p2x, p2y)
		if i % 2 == 1 then
			clip_polygon_against_edge(edge_a, edge_b, edge_c, clip_poly1, clip_poly2)
		else
			clip_polygon_against_edge(edge_a, edge_b, edge_c, clip_poly2, clip_poly1)
		end
	end

	if nedges % 2 == 0 then
		return clip_poly1
	else
		return clip_poly2
	end
end

---Clip a polygon with an axis aligned bounding box
---
---Uses Sutherland-Hodgman clipping. For best results source polygon should be convex.
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param min_x number
---@param min_y number
---@param max_x number
---@param max_y number
function M.clip_polygon_aabb(polygon, min_x, min_y, max_x, max_y)
	assert(#polygon > 5, "bad argument (polygon must have at least 3 vertices)")
	_assert(min_x <= max_x and min_y <= max_y, "bad argument (min point (%.2f, %.2f) is larger than max point (%.2f, %.2f)", min_x, max_x, min_y, max_y)
	return M.clip_polygon_polygon(polygon, { min_x, max_y, max_x, max_y, max_x, min_y, max_x, min_y })
end

---Clip a polygon with a convex polygon
---
---Uses Sutherland-Hodgman clipping. For best results source polygon should be convex also.
---
---@see geo2d_module.convex_polygons_from_polygon
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param clipping_convex_polygon number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.clip_polygon_polygon(polygon, clipping_convex_polygon)
	assert(#polygon >= 6, "bad argument (polygon must have at least 3 vertices)")
	assert(#clipping_convex_polygon >= 6, "bad argument (clipping_convex_polygon must have at least 3 vertices)")

	local clip = clipping_convex_polygon
	local orientation = M.is_polygon_convex(clip)
	if orientation == nil then
		error("bad argument (clipping_convex_polygon must be convex)")
	end
	local counterclockwise = orientation == COUNTERCLOCKWISE

	-- Build edge list
	local edges = {} -- (e1x1, e1y1, e1x2, e1y2, e2x1, e2y1, e2x2, e2y2, ...)
	for j = 1, #clip, 2 do
		local x1, y1 = clip[j], clip[j + 1]
		local x2, y2 = (clip[j + 2] or clip[1]), (clip[j + 3] or clip[2])
		if counterclockwise then
			edges[#edges + 1] = x2
			edges[#edges + 1] = y2
			edges[#edges + 1] = x1
			edges[#edges + 1] = y1
		else
			edges[#edges + 1] = x1
			edges[#edges + 1] = y1
			edges[#edges + 1] = x2
			edges[#edges + 1] = y2
		end
	end

	return clip_polygon_from_edge_list(polygon, edges)
end

-----------------------------------------------------------
-- Polygon Misc
-----------------------------------------------------------

---Returns a polygon with vertices in counter clockwise order
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.polygon_ordered_from_points(points)
	assert(#points > 4, "bad argument (polygon must have at least 3 vertices)")
	local pts = {} -- (x1, y1, x2, y2, ...)
	local angles = {}
	for i = 1, #points do
		pts[i] = points[i]
		angles[i] = 0
	end
	local npoints = #points / 2

	do
		-- Find the lowest vertex
		local j = 1
		for i = 2, npoints do
			if pts[2*(i)] < pts[2*(j)] or (pts[2*(i)] == pts[2*(j)] and pts[2*(i)-1] < pts[2*(j)-1]) then
				j = i
			end
		end
		pts[1], pts[2], pts[2 * j - 1], pts[2 * j] = pts[2 * j - 1], pts[2 * j], pts[1], pts[2]
		angles[1], angles[j] = angles[j], angles[1]
	end

	-- Sort points around the anchor
	local anchor_x, anchor_y = pts[1], pts[2]
	for i = 2, npoints do
		angles[i] = M.cartesian_angle(pts[2*(i)-1] - anchor_x, pts[2*(i)] - anchor_y)
	end
	for i = 2, npoints do
		local p1x, p1y = pts[2*(i)-1], pts[2*(i)]
		for j = i + 1, npoints do
			local p2x, p2y = pts[2*(j)-1], pts[2*(j)]
			local less_than = false
			if angles[i] < angles[j] then
				less_than = true
			elseif angles[i] > angles[j] then
				less_than = false
			elseif are_points_equal(p1x, p1y, p2x, p2y) then
				less_than = false
			else
				local d1 = M.sq_distance(anchor_x, anchor_y, p1x, p1y)
				local d2 = M.sq_distance(anchor_x, anchor_y, p2x, p2y)
				less_than = d1 < d2
			end

			if not less_than then
				angles[i], angles[j] = angles[j], angles[i]
				pts[2 * i - 1], pts[2 * i], pts[2 * j - 1], pts[2 * j] = pts[2 * j - 1], pts[2 * j], pts[2 * i - 1],
					pts[2 * i]
			end
		end
	end

	return pts
end

---Returns the orientation if the polygon is convex, otherwise returns nil
---@param polygon number[]
---@return integer? -- -1 if CLOCKWISE, 1 if COUNTERCLOCKWISE
function M.is_polygon_convex(polygon)
	assert(#polygon > 5, "bad argument (polygon must have at least 3 vertices)")
	local poly_orien = COLLINEAR
	local npolygon = #polygon / 2
	for i = 1, npolygon do
		local prev = 1 + (i - 2) % npolygon
		local next = 1 + i % npolygon
		local ax, ay = polygon[2*(prev)-1], polygon[2*(prev)]
		local bx, by = polygon[2*(i)-1], polygon[2*(i)]
		local cx, cy = polygon[2*(next)-1], polygon[2*(next)]
		local edge_orien = M.orientation(ax, ay, bx, by, cx, cy)
		if edge_orien == COLLINEAR then
			-- Skip collinear edges
		elseif poly_orien ~= COLLINEAR and edge_orien ~= poly_orien then
			-- Orientation flipped, so polygon is not convex
			return
		else
			poly_orien = edge_orien
		end
	end
	return (poly_orien ~= COLLINEAR) and poly_orien or nil
end

-----------------------------------------------------------
-- Triangulation
-----------------------------------------------------------

---Return previous and next index in a circular array
---@param index integer
---@param length integer
local function prev_next_indices(index, length)
	local prev_index = 1 + (index - 2) % length
	local next_index = 1 + index % length
	return prev_index, next_index
end

---@param index integer
---@param polygon number[]
---@return number, number, number, number, number, number
local function vertex_triangle(index, polygon)
	local prev, next = prev_next_indices(index, #polygon / 2)
	local ax, ay = polygon[2*(prev)-1], polygon[2*(prev)]
	local bx, by = polygon[2*(index)-1], polygon[2*(index)]
	local cx, cy = polygon[2*(next)-1], polygon[2*(next)]
	return ax, ay, bx, by, cx, cy
end

---@param index integer
---@param polygon number[]
---@return boolean
local function vertex_is_ear(index, polygon)
	local ax, ay, bx, by, cx, cy = vertex_triangle(index, polygon)
	if M._robust_collinear(ax, ay, bx, by, cx, cy) then
		return false
	end

	local prev, next = prev_next_indices(index, #polygon / 2)
	for i = 1, #polygon / 2 do
		if i ~= prev and i ~= next and i ~= index then
			if M.point_in_triangle(polygon[2*(i)-1], polygon[2*(i)], ax, ay, bx, by, cx, cy) then
				return false
			end
		end
	end

	return true
end

---@param index integer
---@param polygon number[]
---@param orientation integer
---@return boolean
local function convex_vertex(index, polygon, orientation)
	return M.orientation(vertex_triangle(index, polygon)) == orientation
end

---@param index integer
---@param polygon number[]
---@return boolean
local function collinear_vertex(index, polygon)
	return M._robust_collinear(vertex_triangle(index, polygon))
end



---@param index integer
---@param indices integer[]
---@return integer, integer, integer
local function vertex_triangle_indexed(index, indices)
	local prev, next = prev_next_indices(index, #indices)
	return indices[prev], indices[index], indices[next]
end

---@param index integer
---@param indices integer[]
---@param points number[]
---@return boolean
local function vertex_is_ear_indexed(index, indices, points)
	local ai, bi, ci = vertex_triangle_indexed(index, indices)
	local ax, ay = points[2*(ai)-1], points[2*(ai)]
	local bx, by = points[2*(bi)-1], points[2*(bi)]
	local cx, cy = points[2*(ci)-1], points[2*(ci)]
	if M._robust_collinear(ax, ay, bx, by, cx, cy) then
		return false
	end

	local prev, next = prev_next_indices(index, #indices)
	for i = 1, #indices do
		if i ~= prev and i ~= next and i ~= index then
			local pi = indices[i]
			if M.point_in_triangle(points[2*(pi)-1], points[2*(pi)], ax, ay, bx, by, cx, cy) then
				return false
			end
		end
	end

	return true
end

---@param index integer
---@param indices integer[]
---@param points number[]
---@param orientation integer
---@return boolean
local function convex_vertex_indexed(index, indices, points, orientation)
	local ai, bi, ci = vertex_triangle_indexed(index, indices)
	local ax, ay = points[2*(ai)-1], points[2*(ai)]
	local bx, by = points[2*(bi)-1], points[2*(bi)]
	local cx, cy = points[2*(ci)-1], points[2*(ci)]
	return M.orientation(ax, ay, bx, by, cx, cy) == orientation
end

---Brute-force intersection test
---@param polygon number[]
function M.polygon_self_intersects(polygon)
	local n = #polygon / 2
	for i = 1, n do
		local next_i = 1 + i % n
		local i_x1, i_y1 = polygon[2*(i)-1], polygon[2*(i)]
		local i_x2, i_y2 = polygon[2*(next_i)-1], polygon[2*(next_i)]
		for j = 1, n do
			local next_j = 1 + j % n

			if i == j or i == next_j or next_i == j or next_i == next_j then
				-- Skip adjacent edges
			else
				local j_x1, j_y1 = polygon[2*(j)-1], polygon[2*(j)]
				local j_x2, j_y2 = polygon[2*(next_j)-1], polygon[2*(next_j)]
				if M.intersect_segment_segment(i_x1, i_y1, i_x2, i_y2, j_x1, j_y1, j_x2, j_y2) then
					return true
				end
			end
		end
	end
	return false
end

---@param polygon number[]
---@return integer -- -1 if CLOCKWISE, 1 if COUNTERCLOCKWISE
function M.polygon_orientation(polygon)
	local n = #polygon / 2
	assert(n >= 3, "bad argument (polygon requires 3 or more edges)")
	local area = 0
	for i = 1, n do
		local j = 1 + (i - 2) % n -- index before i
		area = area + (polygon[2*(j)-1] * polygon[2*(i)] - polygon[2*(i)-1] * polygon[2*(j)])
	end
	if area >= 0 then
		return COUNTERCLOCKWISE
	else
		return CLOCKWISE
	end
end

---Triangulate a polygon with 4 or more sides using the ear-clipping method
---Return a list of triangles
---@see geo2d_module.triangle_indices_from_polygon
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param prefer_equilateral_triangles? boolean -- Sorts the clipped ears for more equilateral triangles (slower but nicer)
---@return number[]? -- (tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri1_x3, tri1_y3, tri2_x1, tri2_y1, tri2_x2, tri2_y2, tri2_x3, tri2_y3, ...)
function M.triangles_from_polygon(polygon, prefer_equilateral_triangles)
	local indices = M.triangle_indices_from_polygon(polygon, prefer_equilateral_triangles)
	if not indices then
		return
	end

	local triangles = {}
	for i = 1, #indices, 3 do
		local ai, bi, ci = indices[i], indices[i + 1], indices[i + 2]
		local ax, ay = polygon[2*(ai)-1], polygon[2*(ai)]
		local bx, by = polygon[2*(bi)-1], polygon[2*(bi)]
		local cx, cy = polygon[2*(ci)-1], polygon[2*(ci)]
		triangles[#triangles + 1] = ax
		triangles[#triangles + 1] = ay
		triangles[#triangles + 1] = bx
		triangles[#triangles + 1] = by
		triangles[#triangles + 1] = cx
		triangles[#triangles + 1] = cy
	end
	return triangles
end

---Triangulate a polygon with 4 or more sides using the ear-clipping method
---Return a list of triangle indices, indexing into the points in polygon data
---@see geo2d_module.triangles_from_polygon
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@param prefer_equilateral_triangles? boolean -- Sorts the clipped ears for more equilateral triangles (slower but nicer)
---@return integer[]? -- (t1_i1, t1_i2, t1_i3, t2_i1, t2_i2, t2_i3, ...)
function M.triangle_indices_from_polygon(polygon, prefer_equilateral_triangles)
	assert(#polygon >= 8, "bad argument (polygon requires 4 or more edges)")
	assert(not M.polygon_self_intersects(polygon), "bad argument (polygon is self-intersecting)")

	local function triangle_niceness(x1, y1, x2, y2, x3, y3)
		local a = M.sq_distance(x1, y1, x2, y2)
		local b = M.sq_distance(x2, y2, x3, y3)
		local c = M.sq_distance(x3, y3, x1, y1)
		local max = math_max(math_max(a, b), c)
		local min = math_min(math_min(a, b), c)
		return min / max
	end

	local points = polygon -- Use polygon as points data
	local indices = {} ---@type integer[]
	for i = 1, #polygon/2 do
		table_insert(indices, i)
	end
	local poly_orien = M.polygon_orientation(polygon)
	local out = {}
	while #indices > 3 do

		local indices_idx = nil

		if prefer_equilateral_triangles then
			local potential_indexes = {}
			for i = 1, #indices do
				if convex_vertex_indexed(i, indices, points, poly_orien) and vertex_is_ear_indexed(i, indices, points) then
					potential_indexes[#potential_indexes + 1] = i
				end
			end

			-- find the ears that are least skinny
			if #potential_indexes == 1 then
				indices_idx = potential_indexes[1]
			elseif #potential_indexes > 1 then
				local max_niceness = -math_huge
				for i = 1, #potential_indexes do
					local ai, bi, ci = vertex_triangle_indexed(potential_indexes[i], indices)
					local ax, ay = points[2*(ai)-1], points[2*(ai)]
					local bx, by = points[2*(bi)-1], points[2*(bi)]
					local cx, cy = points[2*(ci)-1], points[2*(ci)]
					local niceness = triangle_niceness(ax, ay, bx, by, cx, cy)
					if niceness > max_niceness then
						max_niceness = niceness
						indices_idx = potential_indexes[i]
					end
				end
			end
		else
			for i = 1, #indices do
				if convex_vertex_indexed(i, indices, points, poly_orien) and vertex_is_ear_indexed(i, indices, points) then
					indices_idx = i
					break
				end
			end
		end

		if indices_idx then
			local ai, bi, ci = vertex_triangle_indexed(indices_idx, indices)
			table_insert(out, ai)
			table_insert(out, bi)
			table_insert(out, ci)
			table_remove(indices, indices_idx)
		end

		if not indices_idx then
			-- Cannot triangulate polygon
			return
		end
	end
	do
		local ai, bi, ci = vertex_triangle_indexed(1, indices)
		table_insert(out, ai)
		table_insert(out, bi)
		table_insert(out, ci)
	end

	return out
end

---Returns the orientation if the indexed polygon is convex, otherwise returns nil
---@param indices integer[]
---@param points number[]
---@return integer? -- -1 if CLOCKWISE, 1 if COUNTERCLOCKWISE
local function is_indexed_polygon_convex(indices, points)
	assert(#indices >= 3, "bad argument (indices must have at least 3 indexes)")
	local poly_orien = COLLINEAR
	for i = 1, #indices do
		local prev, next = prev_next_indices(i, #indices)
		local ai, bi, ci = indices[prev], indices[i], indices[next]
		local ax, ay = points[2*(ai)-1], points[2*(ai)]
		local bx, by = points[2*(bi)-1], points[2*(bi)]
		local cx, cy = points[2*(ci)-1], points[2*(ci)]
		local edge_orien = M.orientation(ax, ay, bx, by, cx, cy)
		if edge_orien == COLLINEAR then
			-- Skip collinear edges
		elseif poly_orien ~= COLLINEAR and edge_orien ~= poly_orien then
			-- Orientation flipped, so polygon is not convex
			return
		else
			poly_orien = edge_orien
		end
	end
	return (poly_orien ~= COLLINEAR) and poly_orien or nil
end

---Decompose a polygon into convex polygons (using Hertel-Mehlhorn algorithm)
---@param polygon number[] -- (x1, y1, x2, y2, ...)
---@return number[][]?
function M.convex_polygons_from_polygon(polygon)
	assert(#polygon >= 8, "bad argument (polygon requires 4 or more edges)")

	local tris = M.triangle_indices_from_polygon(polygon, true)
	if not tris then
		return
	end
	
	-- Construct mesh as a list of indexed polygons, starting with the set of triangles
	local mesh = {}
	for i=1,#tris,3 do
		mesh[#mesh+1] = {tris[i], tris[i+1], tris[i+2]}
	end

	-- Create list of index-pairs which are diagonals in the mesh
	local function is_diagonal(i, j)
		if i > j then return false end 
		local prev, next = prev_next_indices(i, #polygon/2)
		return j ~= prev and j ~= next
	end

	local diags = {}
	for i=1,#mesh do
		local tri = mesh[i]
		local a, b, c = tri[1], tri[2], tri[3]
		if is_diagonal(a, b) then
			diags[#diags+1] = {a, b}
		end
		if is_diagonal(b, c) then
			diags[#diags+1] = {b, c}
		end
		if is_diagonal(c, a) then
			diags[#diags+1] = {c, a}
		end
	end

	local function indexed_poly_contains_edge(poly, ai, bi)
		local has_a, has_b
		for _, i in ipairs(poly) do
			if i == ai then
				has_a = true
			elseif i == bi then
				has_b = true
			end
		end
		return has_a and has_b
	end

	---@return integer[]
	local function merge(poly1_index, poly2_index, ai, bi)
		local poly1 = mesh[poly1_index]
		local poly2 = mesh[poly2_index]

		local merged = {}

		local poly1_start
		for i=1,#poly1 do
			local j = 1 + i % #poly1
			if poly1[i] == ai and poly1[j] ~= bi then
				poly1_start = i
				break
			elseif poly1[i] == bi and poly1[j] ~= ai then
				poly1_start = i
				break
			end
		end

		-- Loop around poly1 starting at poly1_start until we reach ai or bi again
		merged[#merged+1] = poly1[poly1_start]
		local poly1_end = 1 + poly1_start % #poly1
		while poly1[poly1_end] ~= ai and poly1[poly1_end] ~= bi do
			merged[#merged+1] = poly1[poly1_end]
			poly1_end = 1 + poly1_end % #poly1
		end
		merged[#merged+1] = poly1[poly1_end]

		-- Loop around poly2 starting at poly1[poly1_end] until we reach ai or bi again
		local poly2_start
		for i=1,#poly2 do
			local j = 1 + i % #poly2
			if poly2[i] == poly1[poly1_end] then
				poly2_start = j
				break
			end
		end

		-- Loop around poly2 starting at poly2_start until we reach ai or bi again
		local poly2_end = poly2_start
		while poly2[poly2_end] ~= ai and poly2[poly2_end] ~= bi do
			merged[#merged+1] = poly2[poly2_end]
			poly2_end = 1 + poly2_end % #poly2
		end

		return merged
	end

	for _, diag in ipairs(diags) do
		-- if we can remove diag from mesh without making it concave then do so
		local ai, bi = diag[1], diag[2]
		local poly1_index, poly2_index
		for i=1,#mesh do
			if (indexed_poly_contains_edge(mesh[i], ai, bi)) then
				if poly1_index then
					poly2_index = i
					break
				else
					poly1_index = i
				end
			end
		end
		assert(poly1_index and poly2_index, "diagonal not found in mesh")
		local merged = merge(poly1_index, poly2_index, ai, bi)
		if is_indexed_polygon_convex(merged, polygon) then
			mesh[poly2_index] = merged
			table_remove(mesh, poly1_index)
		end
	end

	-- Construct final polygons
	local out = {}
	for _, poly in ipairs(mesh) do
		local new_poly = {}
		for _, i in ipairs(poly) do
			new_poly[#new_poly+1] = polygon[2*(i)-1]
			new_poly[#new_poly+1] = polygon[2*(i)]
		end
		out[#out+1] = new_poly
	end
	return out
end

-----------------------------------------------------------
-- Convex hull
-----------------------------------------------------------

---Calculate a convex hull of a set of points (Graham scan)
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number[] -- (x1, y1, x2, y2, ...)
function M.convex_hull(points)
	assert(#points >= 6, "bad argument (points requires 3 or more points to calculate convex hull)")

	local pts = {} -- (x1, y1, x2, y2, ...)
	local angles = {}
	local npoints = #points / 2
	for i = 1, npoints do
		pts[2 * i - 1] = points[2 * i - 1]
		pts[2 * i] = points[2 * i]
		angles[i] = 0
	end

	do
		local j = 1
		for i = 2, npoints do
			if pts[2*(i)] < pts[2*(j)] or (pts[2*(i)] == pts[2*(j)] and pts[2*(i)-1] < pts[2*(j)-1]) then
				j = i
			end
		end

		pts[1], pts[2], pts[2 * j - 1], pts[2 * j] = pts[2 * j - 1], pts[2 * j], pts[1], pts[2]
	end

	-- Sort points around the anchor
	local anchor_x, anchor_y = pts[1], pts[2]
	for i = 2, npoints do
		angles[i] = M.cartesian_angle(pts[2*(i)-1] - anchor_x, pts[2*(i)] - anchor_y)
	end

	for i = 2, npoints do
		local p1x, p1y = pts[2*(i)-1], pts[2*(i)]
		for j = i + 1, npoints do
			local p2x, p2y = pts[2*(j)-1], pts[2*(j)]
			local less_than = false
			if angles[i] < angles[j] then
				less_than = true
			elseif angles[i] > angles[j] then
				less_than = false
			elseif are_points_equal(p1x, p1y, p2x, p2y) then
				less_than = false
			else
				local d1 = M.sq_distance(anchor_x, anchor_y, p1x, p1y)
				local d2 = M.sq_distance(anchor_x, anchor_y, p2x, p2y)
				less_than = d1 < d2
			end

			if not less_than then
				angles[i], angles[j] = angles[j], angles[i]
				pts[2 * i - 1], pts[2 * i], pts[2 * j - 1], pts[2 * j] = pts[2 * j - 1], pts[2 * j], pts[2 * i - 1],
					pts[2 * i]
			end
		end
	end

	local hull = { pts[3], pts[4], pts[1], pts[2] }
	local nhull = 4

	do
		local j = 3
		while j <= npoints do
			if nhull > 2 then
				if M.orientation(hull[3], hull[4], hull[1], hull[2], pts[2*(j)-1], pts[2*(j)]) == COUNTERCLOCKWISE then
					table_insert(hull, 1, pts[2*(j)-1])
					table_insert(hull, 2, pts[2*(j)])
					j = j + 1
					nhull = nhull + 2
				else
					table_remove(hull, 2)
					table_remove(hull, 1)
					nhull = nhull - 2
				end
			else
				table_insert(hull, 1, pts[2*(j)-1])
				table_insert(hull, 2, pts[2*(j)])
				nhull = nhull + 2
				j = j + 1
			end
		end
	end

	return hull
end

-----------------------------------------------------------
-- Minimum bounding ball
-----------------------------------------------------------

---Randomise flat array of 2d points in-place from _point_ index `from` to index `to`
---
---Note: index is relative to point
---@param rng fun():number -- Random function returning value in [0,1]
---@param points number[] -- (x1, y1, x2, y2, ...)
---@param from integer
---@param to integer
---@return number[] -- (x1, y1, x2, y2, ...)
local function shuffle_points_in_place(rng, points, from, to)
	assert(from > 0 and from <= to and to <= #points / 2, "bad argument (invalid range)")
	local n = (to - from + 1)
	for i = from, to do
		local j = math_min(math_floor(rng() * n) + from, to)
		local ix, jx = 1 + (i - 1) * 2, 1 + (j - 1) * 2
		local iy, jy = ix + 1, jx + 1
		points[ix], points[iy], points[jx], points[jy] = points[jx], points[jy], points[ix], points[iy]
	end
	return points
end

-- Return the minimum circle that contains all 3 points
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@param x3 number
---@param y3 number
---@return number, number, number -- (x, y, r)
local function minimum_circle_from_3_points(x1, y1, x2, y2, x3, y3)
	local x, y, r = M.circle_from_two_points(x1, y1, x2, y2)
	if M.point_in_circle(x3, y3, x, y, r) then
		return x, y, r
	end
	x, y, r = M.circle_from_two_points(x1, y1, x3, y3)
	if M.point_in_circle(x2, y2, x, y, r) then
		return x, y, r
	end
	x, y, r = M.circle_from_two_points(x2, y2, x3, y3)
	if M.point_in_circle(x1, y1, x, y, r) then
		return x, y, r
	end
	return M.circle_from_three_points(x1, y1, x2, y2, x3, y3)
end

---@param rng fun():number -- Random function returning value in [0,1]
---@param points number[] -- (x1, y1, x2, y2, ...)
---@param end_index number
---@param x1 number
---@param y1 number
---@param x2 number
---@param y2 number
---@return number, number, number -- (x, y, r)
local function minimum_circle_with_2_points(rng, points, end_index, x1, y1, x2, y2)
	shuffle_points_in_place(rng, points, 1, end_index)
	local x, y, r = M.circle_from_two_points(x1, y1, x2, y2)
	for i = 1, end_index * 2, 2 do
		if not M.point_in_circle(points[i], points[i + 1], x, y, r) then
			x, y, r = minimum_circle_from_3_points(x1, y1, x2, y2, points[i], points[i + 1])
		end
	end
	return x, y, r
end

---@param rng fun():number -- Random function returning value in [0,1]
---@param points number[] -- (x1, y1, x2, y2, ...)
---@param end_index number
---@param qx number
---@param qy number
---@return number, number, number -- (x, y, r)
local function minimum_circle_with_1_point(rng, points, end_index, qx, qy)
	shuffle_points_in_place(rng, points, 1, end_index)
	local x, y, r = M.circle_from_two_points(qx, qy, points[1], points[2])
	for i = 3, end_index * 2, 2 do
		if not M.point_in_circle(points[i], points[i + 1], x, y, r) then
			x, y, r = minimum_circle_with_2_points(rng, points, (i - 1) / 2, qx, qy, points[i], points[i + 1])
		end
	end
	return x, y, r
end

---Return a circle that encloses all points
---
---NB: Can improve performance by passing in convex hull
---
---@see geo2d_module.minimum_bounding_circle_ritter
---@see geo2d_module.convex_hull
---
---@param rng fun():number -- Random function returning value in [0,1]
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number, number, number -- (x, y, r)
function M.minimum_bounding_circle_randomized(rng, points)
	assert(#points > 0, "bad argument (points must have at least 1 point)")

	local npoints = #points / 2
	if npoints == 1 then
		return points[1], points[2], 0
	elseif npoints == 2 then
		return M.circle_from_two_points(points[1], points[2], points[3], points[4])
	elseif npoints == 3 then
		return minimum_circle_from_3_points(points[1], points[2], points[3], points[4], points[5], points[6])
	end

	-- Copy and randomise points
	local rpoints = {}
	for i = 1, #points do
		rpoints[i] = points[i]
	end
	shuffle_points_in_place(rng, rpoints, 1, npoints)

	local x, y, r = M.circle_from_two_points(rpoints[1], rpoints[2], rpoints[3], rpoints[4])
	for i = 5, #rpoints, 2 do
		if not M.point_in_circle(rpoints[i], rpoints[i + 1], x, y, r) then
			x, y, r = minimum_circle_with_1_point(rng, rpoints, (i - 1) / 2, rpoints[i], rpoints[i + 1])
		end
	end
	return x, y, r
end

---Return a non-minimal circle that encloses all points (in O(nd) time)
---
---NB: Can improve performance by passing in convex hull
---
---@see geo2d_module.minimum_bounding_circle_randomized
---@see geo2d_module.convex_hull
---
---@param points number[] -- (x1, y1, x2, y2, ...)
---@return number, number, number -- (x, y, r)
function M.minimum_bounding_circle_ritter(points)
	assert(#points > 0, "bad argument (points must have at least 1 point)")

	local npoints = #points / 2
	if npoints == 1 then
		return points[1], points[2], 0
	elseif npoints == 2 then
		return M.circle_from_two_points(points[1], points[2], points[3], points[4])
	elseif npoints == 3 then
		return minimum_circle_from_3_points(points[1], points[2], points[3], points[4], points[5], points[6])
	end

	local min_x_x, min_x_y, min_y_x, min_y_y = math_huge, math_huge, math_huge, math_huge
	local max_x_x, max_x_y, max_y_x, max_y_y = -math_huge, -math_huge, -math_huge, -math_huge

	for i = 1, #points, 2 do
		local x, y = points[i], points[i + 1]
		if x < min_x_x then min_x_x, min_x_y = x, y end
		if x > max_x_x then max_x_x, max_x_y = x, y end
		if y < min_y_y then min_y_x, min_y_y = x, y end
		if y > max_y_y then max_y_x, max_y_y = x, y end
	end

	local span_x = M.distance(max_x_x, max_x_y, min_x_x, min_x_y)
	local span_y = M.distance(max_y_x, max_y_y, min_y_x, min_y_y)

	local dia1_x, dia1_y, dia2_x, dia2_y = -math_huge, -math_huge, -math_huge, -math_huge

	if span_x > span_y then
		dia1_x, dia1_y, dia2_x, dia2_y = min_x_x, min_x_y, max_x_x, max_x_y
	else
		dia1_x, dia1_y, dia2_x, dia2_y = min_y_x, min_y_y, max_y_x, max_y_y
	end

	local cx, cy, cr = M.circle_from_two_points(dia1_x, dia1_y, dia2_x, dia2_y)
	local radius_sqr = cr * cr

	for i = 1, #points, 2 do
		local x, y = points[i], points[i + 1]
		local lay_dist = M.sq_distance(x, y, cx, cy)
		if lay_dist > radius_sqr then
			local dist = math_sqrt(lay_dist)
			cr = (cr + dist) * 0.5
			radius_sqr = cr * cr
			local difference = dist - cr
			local ratio = 1.0 / dist
			cx, cy, cr = (cr * cx + difference * x) * ratio, (cr * cy + difference * y) * ratio, cr
		end
	end

	return cx, cy, cr
end

return M
