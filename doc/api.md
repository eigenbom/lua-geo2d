# geo2d
2D Computational Geometry  

This library provides a set of functions for 2D computational geometry  
https://github.com/eigenbom/lua-geo2d  
Benjamin Porter  
Licensed under the MIT license  

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
* The file is also available in debug mode (lua51_debug\lua)  
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
	* aabb_from_circle  
	* aabb_from_pcurve  
	* aabb_from_points  
	* aabb_from_quad  
	* aabb_from_segment  
	* aabb_from_triangle  

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

## _contain_pcurve

```lua
function _contain_pcurve(point_count: integer, sampler: fun(t: number):number, number, point_checker: fun(x1: number, y1: number, ...any):boolean, ...any)
  -> boolean
```

Helper function to test points of a parametric curve

@*param* `point_count` — >= 2

## _contain_points

```lua
function _contain_points(points: number[], point_checker: fun(x1: number, y1: number, ...any):boolean, ...any)
  -> boolean
```

Helper function to test point containment

@*param* `points` — (x1, y1, x2, y2, ...)

## _differing_orientation

```lua
function _differing_orientation(x1: number, y1: number, x2: number, y2: number, p1x: number, p1y: number, p2x: number, p2y: number)
  -> boolean
```

## _intersect_cubic_bezier_shape

```lua
function _intersect_cubic_bezier_shape(point_count: integer, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, segment_intersector: fun(x1: number, y1: number, x2: number, y2: number, ...any):boolean, ...any)
  -> boolean
```

Helper function to test segments of a bezier curve for intersection

@*param* `point_count` — >= 2

## _intersect_pcurve

```lua
function _intersect_pcurve(point_count: integer, sampler: fun(t: number):number, number, segment_intersector: fun(x1: number, y1: number, x2: number, y2: number, ...any):boolean, ...any)
  -> boolean
```

Helper function to test segments of a parametric curve

@*param* `point_count` — >= 2

## _intersect_polygon_segments

```lua
function _intersect_polygon_segments(polygon: number[], segment_intersector: fun(x1: number, y1: number, x2: number, y2: number, ...any):boolean, ...any)
  -> boolean
```

Helper function to test intersection of segments with outline of a polygon

@*param* `polygon` — (x1, y1, x2, y2, ...)

## _intersect_polyline

```lua
function _intersect_polyline(polyline: number[], segment_intersector: fun(x1: number, y1: number, x2: number, y2: number, ...any):boolean, ...any)
  -> boolean
```

Helper function to test intersection of segments of a polyline

@*param* `polyline` — (x1, y1, x2, y2, ...)

## _intersect_quadratic_bezier_shape

```lua
function _intersect_quadratic_bezier_shape(point_count: integer, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, segment_intersector: fun(x1: number, y1: number, x2: number, y2: number, ...any):boolean, ...any)
  -> boolean
```

Helper function to test segments of a bezier curve for intersection

@*param* `point_count` — >= 2

## _intersection_point_pcurve

```lua
function _intersection_point_pcurve(point_count: integer, sampler: fun(t: number):number, number, segment_intersector: fun(x1: number, y1: number, x2: number, y2: number, ...any):number?, number?, number?, number?, ...any)
  -> number[]
```

Helper function to test segments of a bezier curve

@*param* `point_count` — >= 2

@*return* — (x1, y1, x2, y2, ...)

## _intersection_point_polygon

```lua
function _intersection_point_polygon(polygon: number[], segment_intersector: fun(x1: number, y1: number, x2: number, y2: number, ...any):number?, number?, number?, number?, ...any)
  -> number[]
```

Helper function to get intersection points in segments of a polygon

@*param* `polygon` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## _intersection_point_polyline

```lua
function _intersection_point_polyline(polyline: number[], segment_intersector: fun(x1: number, y1: number, x2: number, y2: number, ...any):number?, number?, number?, number?, ...any)
  -> number[]
```

Helper function to get intersection points in segments of a polyline

@*param* `polyline` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## _intersection_point_ray_segment_dist

```lua
function _intersection_point_ray_segment_dist(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, x1: number, y1: number, x2: number, y2: number)
  -> number?, number?, number?
```

Checks for intersection of a ray and line segment

NOTE: Returns `x, y, dist`, where `x,y` is the intersection point and `dist` is the distance from the ray origin to the intersection point

## _minimum_distance_from_point_to_line

```lua
function _minimum_distance_from_point_to_line(px: number, py: number, x1: number, y1: number, x2: number, y2: number)
  -> number
```

## _oriented_vertex_angle

```lua
function _oriented_vertex_angle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, orient: integer)
  -> degrees: number
```

## _point_on_ray_distance

```lua
function _point_on_ray_distance(px: number, py: number, ray_x: number, ray_y: number, ray_dx: number, ray_dy: number)
  -> number?
```

If the point (px,py) is on the ray (ray_x,ray_y)->(ray_dx,ray_dy), return the distance from origin to point

## _point_test_circumcircle

```lua
function _point_test_circumcircle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, px: number, py: number)
  -> status: integer
```

Checks whether (px,py) is inside, outside, or on the circle circumscribed by the three points (x1,y1), (x2,y2), (x3,y3)

@*return* `status` — (1 = inside, -1 = outside, 0 = cocircular)

## _polygon_contains_points

```lua
function _polygon_contains_points(polygon: number[], shape_points: number[])
  -> boolean
```

Return true if any shape_points are inside polygon

@*param* `polygon` — (x1, y1, x2, y2, ...)

@*param* `shape_points` — (x1, y1, x2, y2, ...)

## _quad_is_valid

```lua
function _quad_is_valid(x1: any, y1: any, x2: any, y2: any, x3: any, y3: any, x4: any, y4: any)
  -> boolean
```

## _robust_collinear

```lua
function _robust_collinear(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, epsilon?: number)
  -> boolean
```

 Are three points collinear

## _robust_orientation

```lua
function _robust_orientation(x1: number, y1: number, x2: number, y2: number, px: number, py: number)
  -> integer
```

## _robust_parallel

```lua
function _robust_parallel(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, epsilon?: number)
  -> boolean
```

## _robust_perpendicular

```lua
function _robust_perpendicular(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, epsilon?: number)
  -> boolean
```

## aabb_from_circle

```lua
function aabb_from_circle(x: number, y: number, radius: number)
  -> number * 4
```

@*return* — min_x, min_y, max_x, max_y

## aabb_from_pcurve

```lua
function aabb_from_pcurve(point_count: integer, pcurve: fun(t: number):number, number)
  -> number * 4
```

@*param* `point_count` — >= 2

## aabb_from_points

```lua
function aabb_from_points(points: number[])
  -> number * 4
```

@*param* `points` — (x1, y1, x2, y2, ...)

@*return* — min_x, min_y, max_x, max_y

## aabb_from_quad

```lua
function aabb_from_quad(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number * 4
```

@*return* — min_x, min_y, max_x, max_y

## aabb_from_segment

```lua
function aabb_from_segment(x1: number, y1: number, x2: number, y2: number)
  -> number * 4
```

@*return* — min_x, min_y, max_x, max_y

## aabb_from_triangle

```lua
function aabb_from_triangle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number * 4
```

@*return* — min_x, min_y, max_x, max_y

## aabb_intersection

```lua
function aabb_intersection(a_min_x: number, a_min_y: number, a_max_x: number, a_max_y: number, b_min_x: number, b_min_y: number, b_max_x: number, b_max_y: number)
  -> number?, number?, number?, number?
```

## aabb_union

```lua
function aabb_union(a_min_x: number, a_min_y: number, a_max_x: number, a_max_y: number, b_min_x: number, b_min_y: number, b_max_x: number, b_max_y: number)
  -> number * 4
```

## aabb_within_aabb

```lua
function aabb_within_aabb(a_min_x: number, a_min_y: number, a_max_x: number, a_max_y: number, b_min_x: number, b_min_y: number, b_max_x: number, b_max_y: number)
  -> boolean
```

Check if `a` is contained within `b`

## area_aabb

```lua
function area_aabb(min_x: number, min_y: number, max_x: number, max_y: number)
  -> number
```

## area_circle

```lua
function area_circle(radius: number)
  -> number
```

## area_polygon

```lua
function area_polygon(polygon: number[])
  -> number
```

Area of a polygon

@*param* `polygon` — (x1, y1, x2, y2, ...)

## area_quad

```lua
function area_quad(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number
```

## area_triangle

```lua
function area_triangle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number
```

## area_triangle_signed

```lua
function area_triangle_signed(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number
```

Calculate signed area of a triangle

## cartesian_angle

```lua
function cartesian_angle(x: number, y: number)
  -> degrees: number
```

## centroid_aabb

```lua
function centroid_aabb(min_x: number, min_y: number, max_x: number, max_y: number)
  -> number, number
```

## centroid_line

```lua
function centroid_line(x1: number, y1: number, x2: number, y2: number)
  -> number, number
```

## centroid_pcurve

```lua
function centroid_pcurve(point_count: integer, pcurve: fun(t: number):number, number)
  -> unknown, unknown
```

Samples pcurve at point_count points and returns the centroid

@*param* `point_count` — >= 2

## centroid_points

```lua
function centroid_points(points: number[])
  -> unknown, unknown
```

Weighted centroid of a set of points

@*param* `points` — (x1, y1, x2, y2, ...)

## centroid_polygon

```lua
function centroid_polygon(polygon: number[])
  -> unknown, unknown
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## centroid_quad

```lua
function centroid_quad(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number, number
```

## centroid_segment

```lua
function centroid_segment(x1: number, y1: number, x2: number, y2: number)
  -> number, number
```

## centroid_triangle

```lua
function centroid_triangle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number, number
```

## circle_from_three_points

```lua
function circle_from_three_points(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number, number, number
```

Return a circle passing through three points (circumcircle)

@*return* — x, y, radius

## circle_from_two_points

```lua
function circle_from_two_points(x1: number, y1: number, x2: number, y2: number)
  -> number, number, number
```

Return a circle passing through two points

@*return* — x, y, radius

## circle_within_aabb

```lua
function circle_within_aabb(x: number, y: number, radius: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> boolean
```

## circle_within_circle

```lua
function circle_within_circle(x1: number, y1: number, r1: number, x2: number, y2: number, r2: number)
  -> boolean
```

## clip_polygon_aabb

```lua
function clip_polygon_aabb(polygon: number[], min_x: number, min_y: number, max_x: number, max_y: number)
  -> number[]
```

Clip a polygon with an axis aligned bounding box

Uses Sutherland-Hodgman clipping. For best results source polygon should be convex.

@*param* `polygon` — (x1, y1, x2, y2, ...)

## clip_polygon_polygon

```lua
function clip_polygon_polygon(polygon: number[], clipping_convex_polygon: number[])
  -> number[]
```

Clip a polygon with a convex polygon

Uses Sutherland-Hodgman clipping. For best results source polygon should be convex also.

@*param* `polygon` — (x1, y1, x2, y2, ...)

@*param* `clipping_convex_polygon` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## clip_segment_aabb

```lua
function clip_segment_aabb(x1: number, y1: number, x2: number, y2: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number?, number?, number?, number?
```

## clip_segment_circle

```lua
function clip_segment_circle(x1: number, y1: number, x2: number, y2: number, cx: number, cy: number, r: number)
  -> number?, number?, number?, number?
```

## clip_segment_quad

```lua
function clip_segment_quad(x1: number, y1: number, x2: number, y2: number, quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number)
  -> number?, number?, number?, number?
```

## clip_segment_triangle

```lua
function clip_segment_triangle(x1: number, y1: number, x2: number, y2: number, tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number)
  -> number?, number?, number?, number?
```

## closest_and_furthest_points_on_segment_from_point

```lua
function closest_and_furthest_points_on_segment_from_point(x1: number, y1: number, x2: number, y2: number, px: number, py: number)
  -> number * 4
```

## closest_point_in_aabb_from_point

```lua
function closest_point_in_aabb_from_point(min_x: number, min_y: number, max_x: number, max_y: number, px: number, py: number)
  -> number, number
```

Return the closest point in the AABB to (px,py)

## closest_point_in_circle_from_circle

```lua
function closest_point_in_circle_from_circle(c1_x: number, c1_y: number, c1_r: number, c2_x: number, c2_y: number, c2_r: number)
  -> number, number
```

## closest_point_in_circle_from_point

```lua
function closest_point_in_circle_from_point(c_x: number, c_y: number, c_r: number, px: number, py: number)
  -> number, number
```

## closest_point_in_circle_from_segment

```lua
function closest_point_in_circle_from_segment(c_x: number, c_y: number, c_r: number, x1: number, y1: number, x2: number, y2: number)
  -> number, number
```

## closest_point_in_polygon_from_point

```lua
function closest_point_in_polygon_from_point(polygon: number[], px: number, py: number)
  -> number, number
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## closest_point_in_quad_from_point

```lua
function closest_point_in_quad_from_point(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, px: number, py: number)
  -> number, number
```

## closest_point_in_triangle_from_point

```lua
function closest_point_in_triangle_from_point(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, px: number, py: number)
  -> number, number
```

## closest_point_on_aabb_from_point

```lua
function closest_point_on_aabb_from_point(min_x: number, min_y: number, max_x: number, max_y: number, px: number, py: number)
  -> number, number
```

Return the closest point on the AABB outline to (px,py)

## closest_point_on_circle_from_circle

```lua
function closest_point_on_circle_from_circle(c1_x: number, c1_y: number, c1_r: number, c2_x: number, c2_y: number, c2_r: number)
  -> number, number
```

## closest_point_on_circle_from_point

```lua
function closest_point_on_circle_from_point(c_x: number, c_y: number, c_r: number, px: number, py: number)
  -> number, number
```

## closest_point_on_circle_from_segment

```lua
function closest_point_on_circle_from_segment(c_x: number, c_y: number, c_r: number, x1: number, y1: number, x2: number, y2: number)
  -> number, number
```

## closest_point_on_line_from_point

```lua
function closest_point_on_line_from_point(x1: number, y1: number, x2: number, y2: number, px: number, py: number)
  -> number, number
```

## closest_point_on_pcurve_from_point

```lua
function closest_point_on_pcurve_from_point(point_count: integer, pcurve: fun(t: number):number, number, px: number, py: number)
  -> number, number
```

@*param* `point_count` — >= 2

## closest_point_on_polygon_from_point

```lua
function closest_point_on_polygon_from_point(polygon: number[], px: number, py: number)
  -> number, number
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## closest_point_on_polyline_from_point

```lua
function closest_point_on_polyline_from_point(polyline: number[], px: number, py: number)
  -> number, number
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## closest_point_on_quad_from_point

```lua
function closest_point_on_quad_from_point(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, px: number, py: number)
  -> number, number
```

## closest_point_on_ray_from_point

```lua
function closest_point_on_ray_from_point(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, px: number, py: number)
  -> number, number
```

## closest_point_on_segment_from_point

```lua
function closest_point_on_segment_from_point(x1: number, y1: number, x2: number, y2: number, px: number, py: number)
  -> number, number
```

## closest_point_on_triangle_from_point

```lua
function closest_point_on_triangle_from_point(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, px: number, py: number)
  -> number, number
```

## collinear

```lua
function collinear(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, epsilon?: number)
  -> boolean
```

Check if three points are collinear

## convex_hull

```lua
function convex_hull(points: number[])
  -> number[]
```

Calculate a convex hull of a set of points (Graham scan)

@*param* `points` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## convex_polygons_from_polygon

```lua
function convex_polygons_from_polygon(polygon: number[])
  -> number[][]?
```

Decompose a polygon into convex polygons (using Hertel-Mehlhorn algorithm)

@*param* `polygon` — (x1, y1, x2, y2, ...)

## cubic_bezier_coefficients

```lua
function cubic_bezier_coefficients(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number * 6
```

Return the cofficients for a cubic bezier curve `(ax,ay)t^3 + (bx,by)t^2 + (cx,cy)t + (x1,y1)`

@*return* — ax, bx, cx, ay, by, cy

## distance

```lua
function distance(x1: number, y1: number, x2: number, y2: number)
  -> number
```

Distance between two points

## dot_product

```lua
function dot_product(x1: number, y1: number, x2: number, y2: number)
  -> number
```

## evaluate_cubic_bezier

```lua
function evaluate_cubic_bezier(t: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number, number
```

Evaluate the cubic bezier curve at position t from control points

@*param* `t` — [0,1]

## evaluate_cubic_equation

```lua
function evaluate_cubic_equation(t: number, x1: number, y1: number, ax: number, bx: number, cx: number, ay: number, by: number, cy: number)
  -> number, number
```

Evaluate cubic bezier at position t from inital point (x1, y1) and coefficients

## evaluate_quadratic_bezier

```lua
function evaluate_quadratic_bezier(t: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number, number
```

Evaluate quadratic bezier at position t from control points

@*param* `t` — [0,1]

## evaluate_quadratic_equation

```lua
function evaluate_quadratic_equation(t: number, x1: number, y1: number, ax: number, bx: number, ay: number, by: number)
  -> number, number
```

Evaluate quadratic bezier at position t from inital point (x1, y1) and coefficients

## horizontal_mirror

```lua
function horizontal_mirror(angle: number)
  -> number
```

Reflect angle

## intersect_aabb_aabb

```lua
function intersect_aabb_aabb(a_min_x: number, a_min_y: number, a_max_x: number, a_max_y: number, b_min_x: number, b_min_y: number, b_max_x: number, b_max_y: number, outline_only?: boolean)
  -> boolean
```

## intersect_aabb_circle

```lua
function intersect_aabb_circle(min_x: number, min_y: number, max_x: number, max_y: number, c_x: number, c_y: number, c_r: number, outline_only?: boolean)
  -> boolean
```

## intersect_aabb_line

```lua
function intersect_aabb_line(min_x: number, min_y: number, max_x: number, max_y: number, x1: number, y1: number, x2: number, y2: number)
  -> boolean
```

## intersect_aabb_pcurve

```lua
function intersect_aabb_pcurve(min_x: number, min_y: number, max_x: number, max_y: number, point_count: any, pcurve: fun(t: number):number, number, outline_only?: boolean)
  -> boolean
```

## intersect_aabb_polygon

```lua
function intersect_aabb_polygon(min_x: number, min_y: number, max_x: number, max_y: number, polygon: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersect_aabb_polyline

```lua
function intersect_aabb_polyline(min_x: number, min_y: number, max_x: number, max_y: number, polyline: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersect_circle_circle

```lua
function intersect_circle_circle(c1_x: number, c1_y: number, c1_r: number, c2_x: number, c2_y: number, c2_r: number, outline_only?: boolean)
  -> boolean
```

## intersect_circle_pcurve

```lua
function intersect_circle_pcurve(c_x: number, c_y: number, c_r: number, point_count: integer, pcurve: fun(t: number):number, number, outline_only?: boolean)
  -> boolean
```

@*param* `point_count` — >= 2

## intersect_circle_polygon

```lua
function intersect_circle_polygon(c_x: number, c_y: number, c_r: number, polygon: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersect_circle_polyline

```lua
function intersect_circle_polyline(c_x: number, c_y: number, c_r: number, polyline: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersect_line_circle

```lua
function intersect_line_circle(x1: number, y1: number, x2: number, y2: number, c_x: number, c_y: number, c_r: number)
  -> boolean
```

## intersect_line_line

```lua
function intersect_line_line(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> boolean
```

## intersect_line_pcurve

```lua
function intersect_line_pcurve(x1: number, y1: number, x2: number, y2: number, point_count: any, pcurve: fun(t: number):number, number)
  -> boolean
```

## intersect_line_polygon

```lua
function intersect_line_polygon(x1: number, y1: number, x2: number, y2: number, polygon: number[])
  -> boolean
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersect_line_polyline

```lua
function intersect_line_polyline(x1: number, y1: number, x2: number, y2: number, polyline: number[])
  -> boolean
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersect_line_quad

```lua
function intersect_line_quad(line_x1: number, line_y1: number, line_x2: number, line_y2: number, quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number)
  -> boolean
```

## intersect_line_triangle

```lua
function intersect_line_triangle(line_x1: number, line_y1: number, line_x2: number, line_y2: number, tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number)
  -> boolean
```

## intersect_pcurve_pcurve

```lua
function intersect_pcurve_pcurve(point_count1: integer, pcurve1: fun(t: number):number, number, point_count2: integer, pcurve2: fun(t: number):number, number)
  -> boolean
```

## intersect_pcurve_polygon

```lua
function intersect_pcurve_polygon(point_count1: integer, pcurve1: fun(t: number):number, number, polygon: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersect_polygon_polygon

```lua
function intersect_polygon_polygon(polygon1: number[], polygon2: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polygon1` — (x1, y1, x2, y2, ...)

@*param* `polygon2` — (x1, y1, x2, y2, ...)

## intersect_polyline_pcurve

```lua
function intersect_polyline_pcurve(polyline: number[], point_count: integer, pcurve: fun(t: number):number, number)
  -> boolean
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersect_polyline_polygon

```lua
function intersect_polyline_polygon(polyline: number[], polygon: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersect_polyline_polyline

```lua
function intersect_polyline_polyline(polyline1: number[], polyline2: number[])
  -> boolean
```

@*param* `polyline1` — (x1, y1, x2, y2, ...)

@*param* `polyline2` — (x1, y1, x2, y2, ...)

## intersect_quad_aabb

```lua
function intersect_quad_aabb(quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number, min_x: number, min_y: number, max_x: number, max_y: number, outline_only?: boolean)
  -> boolean
```

-@return boolean

## intersect_quad_circle

```lua
function intersect_quad_circle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, c_x: number, c_y: number, c_r: number, outline_only?: boolean)
  -> boolean
```

## intersect_quad_pcurve

```lua
function intersect_quad_pcurve(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, point_count: integer, pcurve: fun(t: number):number, number, outline_only?: boolean)
  -> boolean
```

@*param* `point_count` — >= 2

## intersect_quad_polygon

```lua
function intersect_quad_polygon(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, polygon: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersect_quad_polyline

```lua
function intersect_quad_polyline(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, polyline: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersect_quad_quad

```lua
function intersect_quad_quad(quad1_x1: number, quad1_y1: number, quad1_x2: number, quad1_y2: number, quad1_x3: number, quad1_y3: number, quad1_x4: number, quad1_y4: number, quad2_x1: number, quad2_y1: number, quad2_x2: number, quad2_y2: number, quad2_x3: number, quad2_y3: number, quad2_x4: number, quad2_y4: number, outline_only?: boolean)
  -> boolean
```

## intersect_ray_aabb

```lua
function intersect_ray_aabb(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> boolean
```

## intersect_ray_circle

```lua
function intersect_ray_circle(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, c_x: number, c_y: number, c_r: number)
  -> boolean
```

## intersect_ray_line

```lua
function intersect_ray_line(ray1_x: number, ray1_y: number, ray1_dx: number, ray1_dy: number, line_x1: number, line_y1: number, line_x2: number, line_y2: number)
  -> boolean
```

## intersect_ray_pcurve

```lua
function intersect_ray_pcurve(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> boolean
```

@*param* `point_count` — >= 2

## intersect_ray_polygon

```lua
function intersect_ray_polygon(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, polygon: number[])
  -> boolean
```

@*param* `polygon` — polygon with 3 or more vertices, as flat array of x,y coordinates [x1, y1, ..., xn, yn]

## intersect_ray_polyline

```lua
function intersect_ray_polyline(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, polyline: number[])
  -> boolean
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersect_ray_quad

```lua
function intersect_ray_quad(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> boolean
```

## intersect_ray_ray

```lua
function intersect_ray_ray(ray1_x: number, ray1_y: number, ray1_dx: number, ray1_dy: number, ray2_x: number, ray2_y: number, ray2_dx: number, ray2_dy: number)
  -> boolean
```

## intersect_ray_segment

```lua
function intersect_ray_segment(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, x1: number, y1: number, x2: number, y2: number)
  -> boolean
```

## intersect_ray_triangle

```lua
function intersect_ray_triangle(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number)
  -> boolean
```

## intersect_segment_aabb

```lua
function intersect_segment_aabb(x1: number, y1: number, x2: number, y2: number, min_x: number, min_y: number, max_x: number, max_y: number, outline_only?: boolean)
  -> boolean
```

## intersect_segment_circle

```lua
function intersect_segment_circle(x1: number, y1: number, x2: number, y2: number, c_x: number, c_y: number, c_r: number, outline_only?: boolean)
  -> boolean
```

## intersect_segment_line

```lua
function intersect_segment_line(seg_x1: number, seg_y1: number, seg_x2: number, seg_y2: number, line_x1: number, line_y1: number, line_x2: number, line_y2: number)
  -> boolean
```

## intersect_segment_pcurve

```lua
function intersect_segment_pcurve(x1: number, y1: number, x2: number, y2: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> boolean
```

@*param* `point_count` — >= 2

## intersect_segment_polygon

```lua
function intersect_segment_polygon(x1: number, y1: number, x2: number, y2: number, polygon: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersect_segment_polyline

```lua
function intersect_segment_polyline(x1: number, y1: number, x2: number, y2: number, polyline: number[])
  -> boolean
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersect_segment_quad

```lua
function intersect_segment_quad(seg_x1: number, seg_y1: number, seg_x2: number, seg_y2: number, quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number, outline_only?: boolean)
  -> boolean
```

## intersect_segment_segment

```lua
function intersect_segment_segment(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> boolean
```

## intersect_segment_segment_simple

```lua
function intersect_segment_segment_simple(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> boolean
```

Check segment-segment intersection (naive version)

## intersect_segment_triangle

```lua
function intersect_segment_triangle(x1: number, y1: number, x2: number, y2: number, t1x: number, t1y: number, t2x: number, t2y: number, t3x: number, t3y: number, outline_only?: boolean)
  -> boolean
```

## intersect_triangle_aabb

```lua
function intersect_triangle_aabb(tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number, min_x: number, min_y: number, max_x: number, max_y: number, outline_only?: boolean)
  -> boolean
```

## intersect_triangle_circle

```lua
function intersect_triangle_circle(tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number, c_x: number, c_y: number, c_r: number, outline_only?: boolean)
  -> boolean
```

## intersect_triangle_pcurve

```lua
function intersect_triangle_pcurve(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, point_count: integer, pcurve: fun(t: number):number, number, outline_only?: boolean)
  -> boolean
```

@*param* `point_count` — >= 2

## intersect_triangle_polygon

```lua
function intersect_triangle_polygon(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, polygon: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersect_triangle_polyline

```lua
function intersect_triangle_polyline(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, polyline: number[], outline_only?: boolean)
  -> boolean
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersect_triangle_quad

```lua
function intersect_triangle_quad(tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number, quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number, outline_only?: boolean)
  -> boolean
```

## intersect_triangle_triangle

```lua
function intersect_triangle_triangle(tri1_x1: number, tri1_y1: number, tri1_x2: number, tri1_y2: number, tri1_x3: number, tri1_y3: number, tri2_x1: number, tri2_y1: number, tri2_x2: number, tri2_y2: number, tri2_x3: number, tri2_y3: number, outline_only?: boolean)
  -> boolean
```

## intersection_point_aabb_aabb

```lua
function intersection_point_aabb_aabb(a_min_x: number, a_min_y: number, a_max_x: number, a_max_y: number, b_min_x: number, b_min_y: number, b_max_x: number, b_max_y: number)
  -> number?, number?, number?, number?, number?, number?, number?, number?
```

## intersection_point_aabb_circle

```lua
function intersection_point_aabb_circle(min_x: number, min_y: number, max_x: number, max_y: number, cx: number, cy: number, radius: number)
  -> number?, number?, number?, number?, number?, number?, number?, number?
```

## intersection_point_aabb_line

```lua
function intersection_point_aabb_line(min_x: number, min_y: number, max_x: number, max_y: number, x1: number, y1: number, x2: number, y2: number)
  -> number?, number?, number?, number?
```

## intersection_point_aabb_pcurve

```lua
function intersection_point_aabb_pcurve(min_x: number, min_y: number, max_x: number, max_y: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> number[]
```

@*param* `point_count` — >= 2

## intersection_point_aabb_polygon

```lua
function intersection_point_aabb_polygon(min_x: number, min_y: number, max_x: number, max_y: number, polygon: number[])
  -> number[]
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersection_point_aabb_polyline

```lua
function intersection_point_aabb_polyline(min_x: number, min_y: number, max_x: number, max_y: number, polyline: number[])
  -> number[]
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersection_point_circle_circle

```lua
function intersection_point_circle_circle(x1: number, y1: number, r1: number, x2: number, y2: number, r2: number)
  -> number?, number?, number?, number?
```

Calculate up to two intersection points of two circles
Special cases:
 * Returns nil if no intersection
 * Returns only one point if the circles overlap exactly

## intersection_point_circle_pcurve

```lua
function intersection_point_circle_pcurve(cx: number, cy: number, cr: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> number[]
```

@*param* `point_count` — >= 2

## intersection_point_circle_polygon

```lua
function intersection_point_circle_polygon(cx: number, cy: number, cr: number, polygon: number[])
  -> number[]
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersection_point_circle_polyline

```lua
function intersection_point_circle_polyline(cx: number, cy: number, cr: number, polyline: number[])
  -> number[]
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersection_point_line_circle

```lua
function intersection_point_line_circle(x1: number, y1: number, x2: number, y2: number, cx: number, cy: number, radius: number)
  -> number?, number?, number?, number?
```

## intersection_point_line_line

```lua
function intersection_point_line_line(line1_x1: number, line1_y1: number, line1_x2: number, line1_y2: number, line2_x1: number, line2_y1: number, line2_x2: number, line2_y2: number)
  -> number?, number?
```

## intersection_point_line_pcurve

```lua
function intersection_point_line_pcurve(x1: number, y1: number, x2: number, y2: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> number[]
```

@*param* `point_count` — >= 2

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_line_polygon

```lua
function intersection_point_line_polygon(x1: number, y1: number, x2: number, y2: number, polygon: number[])
  -> number[]
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_line_polyline

```lua
function intersection_point_line_polyline(x1: number, y1: number, x2: number, y2: number, polyline: number[])
  -> number[]
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_line_quad

```lua
function intersection_point_line_quad(line_x1: number, line_y1: number, line_x2: number, line_y2: number, quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number)
  -> number?, number?, number?, number?
```

## intersection_point_line_triangle

```lua
function intersection_point_line_triangle(line_x1: number, line_y1: number, line_x2: number, line_y2: number, tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number)
  -> number?, number?, number?, number?
```

## intersection_point_pcurve_pcurve

```lua
function intersection_point_pcurve_pcurve(point_count1: integer, pcurve1: fun(t: number):number, number, point_count2: integer, pcurve2: fun(t: number):number, number)
  -> number[]
```

## intersection_point_pcurve_polygon

```lua
function intersection_point_pcurve_polygon(point_count: integer, pcurve: fun(t: number):number, number, polygon: number[])
  -> number[]
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersection_point_polygon_polygon

```lua
function intersection_point_polygon_polygon(polygon1: number[], polygon2: number[])
  -> number[]
```

@*param* `polygon1` — (x1, y1, x2, y2, ...)

@*param* `polygon2` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_polyline_pcurve

```lua
function intersection_point_polyline_pcurve(polyline: number[], point_count: integer, pcurve: fun(t: number):number, number)
  -> number[]
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersection_point_polyline_polygon

```lua
function intersection_point_polyline_polygon(polyline: number[], polygon: number[])
  -> number[]
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

@*param* `polygon` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_polyline_polyline

```lua
function intersection_point_polyline_polyline(polyline1: number[], polyline2: number[])
  -> number[]
```

@*param* `polyline1` — (x1, y1, x2, y2, ...)

@*param* `polyline2` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_quad_aabb

```lua
function intersection_point_quad_aabb(quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number?, number?, number?, number?, number?, number?, number?, number?
```

## intersection_point_quad_circle

```lua
function intersection_point_quad_circle(quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number, cx: number, cy: number, radius: number)
  -> number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?
```

## intersection_point_quad_pcurve

```lua
function intersection_point_quad_pcurve(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> number[]
```

@*param* `point_count` — >= 2

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_quad_polygon

```lua
function intersection_point_quad_polygon(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, polygon: number[])
  -> number[]
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_quad_polyline

```lua
function intersection_point_quad_polyline(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, polyline: number[])
  -> number[]
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_quad_quad

```lua
function intersection_point_quad_quad(quad1_x1: number, quad1_y1: number, quad1_x2: number, quad1_y2: number, quad1_x3: number, quad1_y3: number, quad1_x4: number, quad1_y4: number, quad2_x1: number, quad2_y1: number, quad2_x2: number, quad2_y2: number, quad2_x3: number, quad2_y3: number, quad2_x4: number, quad2_y4: number)
  -> number * 16
```

## intersection_point_ray_aabb

```lua
function intersection_point_ray_aabb(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number?, number?, number?, number?
```

## intersection_point_ray_circle

```lua
function intersection_point_ray_circle(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, c_x: number, c_y: number, c_r: number)
  -> number?, number?, number?, number?
```

Returns the intersection points of a ray and a circle
The closest point is returned first

## intersection_point_ray_line

```lua
function intersection_point_ray_line(ray1_x: number, ray1_y: number, ray1_dx: number, ray1_dy: number, line_x1: number, line_y1: number, line_x2: number, line_y2: number)
  -> number?, number?
```

## intersection_point_ray_pcurve

```lua
function intersection_point_ray_pcurve(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> number[]
```

@*param* `point_count` — >= 2

## intersection_point_ray_polygon

```lua
function intersection_point_ray_polygon(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, polygon: number[])
  -> number[]?
```

@*param* `polygon` — polygon with 3 or more vertices, as flat array of x,y coordinates [x1, y1, ..., xn, yn]

@*return* — Sorted list of intersection points (x1, y1, x2, y2, ...)

## intersection_point_ray_polyline

```lua
function intersection_point_ray_polyline(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, polyline: number[])
  -> number[]
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersection_point_ray_quad

```lua
function intersection_point_ray_quad(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, px1: number, py1: number, px2: number, py2: number, px3: number, py3: number, px4: number, py4: number)
  -> number?, number?, number?, number?
```

## intersection_point_ray_ray

```lua
function intersection_point_ray_ray(ray1_x: number, ray1_y: number, ray1_dx: number, ray1_dy: number, ray2_x: number, ray2_y: number, ray2_dx: number, ray2_dy: number)
  -> number?, number?
```

## intersection_point_ray_segment

```lua
function intersection_point_ray_segment(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, x1: number, y1: number, x2: number, y2: number)
  -> number?, number?
```

Checks for intersection of a ray and line segment

## intersection_point_ray_triangle

```lua
function intersection_point_ray_triangle(ray_x: number, ray_y: number, ray_dx: number, ray_dy: number, px1: number, py1: number, px2: number, py2: number, px3: number, py3: number)
  -> number?, number?, number?, number?
```

## intersection_point_segment_aabb

```lua
function intersection_point_segment_aabb(x1: number, y1: number, x2: number, y2: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number?, number?, number?, number?
```

## intersection_point_segment_circle

```lua
function intersection_point_segment_circle(x1: number, y1: number, x2: number, y2: number, cx: number, cy: number, radius: number)
  -> number?, number?, number?, number?
```

## intersection_point_segment_line

```lua
function intersection_point_segment_line(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number?, number?
```

Checks for intersection of a segment and line

## intersection_point_segment_pcurve

```lua
function intersection_point_segment_pcurve(seg_x1: number, seg_y1: number, seg_x2: number, seg_y2: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> number[]
```

@*param* `point_count` — >= 2

## intersection_point_segment_polygon

```lua
function intersection_point_segment_polygon(seg_x1: number, seg_y1: number, seg_x2: number, seg_y2: number, polygon: number[])
  -> number[]
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## intersection_point_segment_polyline

```lua
function intersection_point_segment_polyline(seg_x1: number, seg_y1: number, seg_x2: number, seg_y2: number, polyline: number[])
  -> number[]
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## intersection_point_segment_quad

```lua
function intersection_point_segment_quad(seg_x1: number, seg_y1: number, seg_x2: number, seg_y2: number, quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number)
  -> number?, number?, number?, number?
```

## intersection_point_segment_segment

```lua
function intersection_point_segment_segment(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number?, number?, number?, number?
```

Calculate up to two intersection points of two line segments
Two points are returned if the line segments overlap

## intersection_point_segment_triangle

```lua
function intersection_point_segment_triangle(seg_x1: number, seg_y1: number, seg_x2: number, seg_y2: number, tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number)
  -> number?, number?, number?, number?
```

## intersection_point_triangle_aabb

```lua
function intersection_point_triangle_aabb(tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number?, number?, number?, number?, number?, number?, number?, number?
```

## intersection_point_triangle_circle

```lua
function intersection_point_triangle_circle(tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number, cx: number, cy: number, radius: number)
  -> number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?
```

## intersection_point_triangle_pcurve

```lua
function intersection_point_triangle_pcurve(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> number[]
```

@*param* `point_count` — >= 2

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_triangle_polygon

```lua
function intersection_point_triangle_polygon(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, polygon: number[])
  -> number[]
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_triangle_polyline

```lua
function intersection_point_triangle_polyline(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, polyline: number[])
  -> number[]
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## intersection_point_triangle_quad

```lua
function intersection_point_triangle_quad(tri_x1: number, tri_y1: number, tri_x2: number, tri_y2: number, tri_x3: number, tri_y3: number, quad_x1: number, quad_y1: number, quad_x2: number, quad_y2: number, quad_x3: number, quad_y3: number, quad_x4: number, quad_y4: number)
  -> number?, number?, number?, number?, number?, number?, number?, number?
```

## intersection_point_triangle_triangle

```lua
function intersection_point_triangle_triangle(tri1_x1: number, tri1_y1: number, tri1_x2: number, tri1_y2: number, tri1_x3: number, tri1_y3: number, tri2_x1: number, tri2_y1: number, tri2_x2: number, tri2_y2: number, tri2_x3: number, tri2_y3: number)
  -> number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?, number?
```

## is_point_collinear

```lua
function is_point_collinear(x1: number, y1: number, x2: number, y2: number, px: number, py: number, robust?: boolean)
  -> boolean
```

Check if the point (px,py) is collinear to the line segment (x1,y1)->(x2,y2)

## is_polygon_convex

```lua
function is_polygon_convex(polygon: number[])
  -> integer?
```

Returns the orientation if the polygon is convex, otherwise returns nil

@*return* — -1 if CLOCKWISE, 1 if COUNTERCLOCKWISE

## is_tangent

```lua
function is_tangent(x1: number, y1: number, x2: number, y2: number, cx: number, cy: number, cr: number)
  -> boolean
```

 Check if a line lies tangent to a circle

## length_pcurve

```lua
function length_pcurve(point_count: integer, pcurve: fun(t: number):number, number)
  -> unknown
```

@*param* `point_count` — >= 2

## length_polyline

```lua
function length_polyline(polyline: number[])
  -> unknown
```

@*param* `polyline` — (x1, y1, x2, y2, ...)

## length_segment

```lua
function length_segment(x1: number, y1: number, x2: number, y2: number)
  -> number
```

## minimum_bounding_circle_randomized

```lua
function minimum_bounding_circle_randomized(rng: fun():number, points: number[])
  -> number, number, number
```

Return a circle that encloses all points

NB: Can improve performance by passing in convex hull


@*param* `rng` — Random function returning value in [0,1]

@*param* `points` — (x1, y1, x2, y2, ...)

@*return* — (x, y, r)
See:
  * [minimum_bounding_circle_ritter]()
  * [convex_hull]()

## minimum_bounding_circle_ritter

```lua
function minimum_bounding_circle_ritter(points: number[])
  -> number, number, number
```

Return a non-minimal circle that encloses all points (in O(nd) time)

NB: Can improve performance by passing in convex hull


@*param* `points` — (x1, y1, x2, y2, ...)

@*return* — (x, y, r)
See:
  * [minimum_bounding_circle_randomized]()
  * [convex_hull]()

## mirror_aabb

```lua
function mirror_aabb(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number * 4
```

## mirror_point

```lua
function mirror_point(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x: number, y: number)
  -> number, number
```

## mirror_points

```lua
function mirror_points(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, points: number[])
  -> number[]
```

@*param* `points` — (x1, y1, x2, y2, ...)

## mirror_points_in_place

```lua
function mirror_points_in_place(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, points: number[])
  -> number[]
```

@*param* `points` — (x1, y1, x2, y2, ...)

## mirror_quad

```lua
function mirror_quad(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number * 8
```

## mirror_ray

```lua
function mirror_ray(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x: number, y: number, dir_x: number, dir_y: number)
  -> number * 4
```

@*return* — (x,y,dir_x,dir_y)

## mirror_segment

```lua
function mirror_segment(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x1: number, y1: number, x2: number, y2: number)
  -> number * 4
```

## mirror_triangle

```lua
function mirror_triangle(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number * 6
```

## normalize

```lua
function normalize(x: number, y: number)
  -> number, number
```

Normalize a vector

@*return* — x, y

## normalize_angle

```lua
function normalize_angle(angle: number)
  -> number
```

Normalize angle in degrees to range [0,360]

## orientation

```lua
function orientation(x1: number, y1: number, x2: number, y2: number, px: number, py: number)
  -> integer
```

Determine if (px, py) lies on the left or right side of the line (x1,y1)->(x2,y2)

@*return* — 1 if left, -1 if right, 0 if collinear

## parallel

```lua
function parallel(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, epsilon?: number)
  -> boolean
```

Check if the lines (x1,y1)->(x2,y2) and (x3,y3)->(x4,y4) are parallel

## pcurve_from_cubic_bezier

```lua
function pcurve_from_cubic_bezier(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> fun(t: number):number, number
```

Returns a parametric curve function (t)->(x,y) from the control points of a cubic bezier curve

## pcurve_from_cubic_hermite

```lua
function pcurve_from_cubic_hermite(x1: number, y1: number, dx1: number, dy1: number, x2: number, y2: number, dx2: number, dy2: number)
  -> fun(t: number):number, number
```

Returns a parametric curve function (t)->(x,y) from the a start point (x1, y1) with velocity (dx1, dy1) and end point (x2, y2) with velocity (dx2, dy2)

## pcurve_from_quadratic_bezier

```lua
function pcurve_from_quadratic_bezier(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> fun(t: number):number, number
```

Returns a parametric curve function (t)->(x,y) from the control points of a quadratic bezier curve

## pcurve_within_aabb

```lua
function pcurve_within_aabb(point_count: integer, pcurve: fun(t: number):number, number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> boolean
```

@*param* `point_count` — >= 2

## perimeter_aabb

```lua
function perimeter_aabb(min_x: number, min_y: number, max_x: number, max_y: number)
  -> number
```

## perimeter_circle

```lua
function perimeter_circle(radius: number)
  -> number
```

## perimeter_polygon

```lua
function perimeter_polygon(polygon: number[])
  -> unknown
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## perimeter_quad

```lua
function perimeter_quad(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number
```

## perimeter_triangle

```lua
function perimeter_triangle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number
```

## perpendicular

```lua
function perpendicular(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, epsilon?: number)
  -> boolean
```

Check if the lines (x1,y1)->(x2,y2) and (x3,y3)->(x4,y4) are perpendicular

## point_in_aabb

```lua
function point_in_aabb(px: number, py: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> boolean
```

## point_in_circle

```lua
function point_in_circle(px: number, py: number, cx: number, cy: number, cr: number)
  -> boolean
```

Checks whether (px,py) is inside the circle defined by the three points

## point_in_circumcircle

```lua
function point_in_circumcircle(px: number, py: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> boolean
```

## point_in_focus_area

```lua
function point_in_focus_area(px: number, py: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> boolean
```

Test if a point is in the "focus area" of a triangle

Example: `a` is in the focus area, `b` is not
```txt
    /1
   /   a
  3
   \
 b  \2
```

## point_in_polygon

```lua
function point_in_polygon(px: number, py: number, polygon: number[])
  -> boolean
```

@*param* `polygon` — (x1, y1, x2, y2, ...)

## point_in_quad

```lua
function point_in_quad(px: number, py: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> boolean
```

## point_in_triangle

```lua
function point_in_triangle(px: number, py: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> boolean
```

## point_of_reflection

```lua
function point_of_reflection(sx1: number, sy1: number, sx2: number, sy2: number, p1x: number, p1y: number, p2x: number, p2y: number)
  -> number?, number?
```

Given a segment (sx1,sy1)->(sx2,sy2) and two points (p1x, p1y) and (p2x, p2y)
calculate the point of reflection (x, y) that lies on the segment
```txt
    p1   p2
     \   /
      \ /
  s1---*---s2
```

## point_on_aabb

```lua
function point_on_aabb(px: number, py: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> boolean
```

## point_on_circle

```lua
function point_on_circle(px: any, py: any, cx: any, cy: any, radius: any)
  -> boolean
```

## point_on_line

```lua
function point_on_line(px: number, py: number, x1: number, y1: number, x2: number, y2: number)
  -> boolean
```

## point_on_quad

```lua
function point_on_quad(px: any, py: any, x1: any, y1: any, x2: any, y2: any, x3: any, y3: any, x4: any, y4: any)
  -> boolean
```

## point_on_ray

```lua
function point_on_ray(px: number, py: number, ray_x: number, ray_y: number, ray_dx: number, ray_dy: number)
  -> boolean
```

## point_on_segment

```lua
function point_on_segment(px: number, py: number, x1: number, y1: number, x2: number, y2: number)
  -> boolean
```

## point_on_triangle

```lua
function point_on_triangle(px: number, py: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> boolean
```

## points_within_aabb

```lua
function points_within_aabb(points: number[], min_x: number, min_y: number, max_x: number, max_y: number)
  -> boolean
```

@*param* `points` — (x1, y1, x2, y2, ...)

## polygon_ordered_from_points

```lua
function polygon_ordered_from_points(points: number[])
  -> number[]
```

Returns a polygon with vertices in counter clockwise order

@*param* `points` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2, ...)

## polygon_orientation

```lua
function polygon_orientation(polygon: number[])
  -> integer
```

@*return* — -1 if CLOCKWISE, 1 if COUNTERCLOCKWISE

## polygon_self_intersects

```lua
function polygon_self_intersects(polygon: number[])
  -> boolean
```

Brute-force intersection test

## polyline_from_cubic_bezier

```lua
function polyline_from_cubic_bezier(point_count: integer, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number[]
```

Creates a polyline (x1, y1, x2, y2, ...) by sampling a cubic bezier curve

@*param* `point_count` — >= 2

@*return* — (x1, y1, x2, y2, ...)

## polyline_from_cubic_hermite

```lua
function polyline_from_cubic_hermite(point_count: integer, x1: number, y1: number, dx1: number, dy1: number, x2: number, y2: number, dx2: number, dy2: number)
  -> number[]
```

Creates a polyline (x1, y1, x2, y2, ...) by sampling a cubic hermite curve with  a start point (x1, y1) with velocity (dx1, dy1) and end point (x2, y2) with velocity (dx2, dy2)

@*param* `point_count` — >= 2

@*return* — (x1, y1, x2, y2, ...)

## polyline_from_quadratic_bezier

```lua
function polyline_from_quadratic_bezier(point_count: integer, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number[]
```

Creates a polyline (x1, y1, x2, y2, ...) by sampling a quadratic bezier curve

@*param* `point_count` — >= 2

@*return* — (x1, y1, x2, y2, ...)

## project_aabb_onto_axis

```lua
function project_aabb_onto_axis(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number * 4
```

@*return* — (x1, y1, x2, y2)

## project_circle_onto_axis

```lua
function project_circle_onto_axis(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x: number, y: number, r: number)
  -> number * 4
```

@*return* — (x1, y1, x2, y2)

## project_pcurve_onto_axis

```lua
function project_pcurve_onto_axis(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, point_count: integer, pcurve: fun(t: number):number, number)
  -> number * 4
```

@*param* `point_count` — >= 2

@*return* — (x1, y1, x2, y2)

## project_point_onto_axis

```lua
function project_point_onto_axis(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x: number, y: number)
  -> number, number
```

## project_points_onto_axis

```lua
function project_points_onto_axis(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, points: number[])
  -> number * 4
```

@*param* `points` — (x1, y1, x2, y2, ...)

@*return* — (x1, y1, x2, y2)

## project_quad_onto_axis

```lua
function project_quad_onto_axis(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number * 4
```

@*return* — (x1, y1, x2, y2)

## project_ray_onto_axis

```lua
function project_ray_onto_axis(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x: number, y: number, dir_x: number, dir_y: number)
  -> number * 4
```

Project ray onto axis, returning a point (x,y) and a projected direction (dir_x, dir_y) which may not be normalized

@*return* — (x, y, proj_dir_x, proj_dir_y)

## project_segment_onto_axis

```lua
function project_segment_onto_axis(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x1: number, y1: number, x2: number, y2: number)
  -> number * 4
```

@*return* — (x1, y1, x2, y2)

## project_triangle_onto_axis

```lua
function project_triangle_onto_axis(axis_x1: number, axis_y1: number, axis_x2: number, axis_y2: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number * 4
```

@*return* — (x1, y1, x2, y2)

## quad_within_aabb

```lua
function quad_within_aabb(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> boolean
```

## quadrant_by_angle

```lua
function quadrant_by_angle(angle: number)
  -> quadrant: integer
```

Return the quadrant of an angle
```
 2|1
 -+-
 3|4
```
Note: The quadrant is 0 for angles outside of range [0,360]

## quadrant_by_point

```lua
function quadrant_by_point(x: number, y: number)
  -> quadrant: integer
```

Return the quadrant of a point
```
 2|1
 -+-
 3|4
```
Note: The quadrant is 0 for point (0,0)

## quadratic_bezier_coefficients

```lua
function quadratic_bezier_coefficients(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number * 4
```

Return the cofficients for a quadratic bezier curve `(ax,ay)t^2 + (bx,by)t + (x1,y1)`

@*return* — ax, bx, ay, by

## random_point_in_aabb

```lua
function random_point_in_aabb(rng: fun():number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number, number
```

@*param* `rng` — Random function returning value in [0,1]

## random_point_in_circle

```lua
function random_point_in_circle(rng: fun():number, x: number, y: number, radius: number)
  -> number, number
```

@*param* `rng` — Random function returning value in [0,1]

## random_point_in_quad

```lua
function random_point_in_quad(rng: fun():number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number, number
```

@*param* `rng` — Random function returning value in [0,1]

## random_point_in_segment

```lua
function random_point_in_segment(rng: fun():number, x1: number, y1: number, x2: number, y2: number)
  -> number, number
```

@*param* `rng` — Random function returning value in [0,1]

## random_point_in_triangle

```lua
function random_point_in_triangle(rng: fun():number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number, number
```

@*param* `rng` — Random function returning value in [0,1]

## random_points_in_polygon

```lua
function random_points_in_polygon(rng: fun():number, count: integer, polygon: number[])
  -> number[]
```

Generates random points within a polygon
Notes:
* Polygon is triangulated

@*param* `rng` — Random function returning value in [0,1]

@*param* `polygon` — (x1, y1, x2, y2, ...)

## random_points_in_polyline

```lua
function random_points_in_polyline(rng: fun():number, count: integer, polyline: number[])
  -> table
```

@*param* `rng` — Random function returning value in [0,1]

@*param* `polyline` — (x1, y1, x2, y2, ...)

## random_points_in_triangles

```lua
function random_points_in_triangles(rng: fun():number, count: integer, triangles: number[])
  -> number[]
```

Generates random points within a set of triangles

@*param* `rng` — Random function returning value in [0,1]

@*param* `count` — Number of points to generate

@*param* `triangles` — (x1, y1, x2, y2, x3, y3, ...)

## rotate

```lua
function rotate(angle: number, x: number, y: number)
  -> number, number
```

Rotate point

## rotate_aaab

```lua
function rotate_aaab(angle: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number * 4
```

## rotate_aabb_around_origin

```lua
function rotate_aabb_around_origin(angle: number, min_x: number, min_y: number, max_x: number, max_y: number, ox: number, oy: number)
  -> number * 4
```

## rotate_around_origin

```lua
function rotate_around_origin(angle: number, x: number, y: number, ox: number, oy: number)
  -> number, number
```

Rotate point around origin

## rotate_point

```lua
function rotate_point(angle: number, x: number, y: number)
  -> number, number
```

## rotate_point_around_origin

```lua
function rotate_point_around_origin(angle: number, x: number, y: number, ox: number, oy: number)
  -> number, number
```

## rotate_points

```lua
function rotate_points(angle: number, points: number[])
  -> number[]
```

@*param* `points` — (x1, y1, x2, y2, ...)

## rotate_points_around_origin

```lua
function rotate_points_around_origin(angle: number, points: number[], ox: number, oy: number)
  -> number[]
```

@*param* `points` — (x1, y1, x2, y2, ...)

## rotate_points_in_place

```lua
function rotate_points_in_place(angle: number, points: number[])
  -> number[]
```

@*param* `points` — (x1, y1, x2, y2, ...)

## rotate_points_in_place_around_origin

```lua
function rotate_points_in_place_around_origin(angle: number, points: number[], ox: number, oy: number)
  -> number[]
```

@*param* `points` — (x1, y1, x2, y2, ...)

## rotate_quad

```lua
function rotate_quad(angle: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number * 8
```

## rotate_quad_around_origin

```lua
function rotate_quad_around_origin(angle: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number, ox: number, oy: number)
  -> number * 8
```

## rotate_ray

```lua
function rotate_ray(angle: number, x: number, y: number, dir_x: number, dir_y: number)
  -> number * 4
```

## rotate_ray_around_origin

```lua
function rotate_ray_around_origin(angle: number, x: number, y: number, dir_x: number, dir_y: number, ox: number, oy: number)
  -> number * 4
```

## rotate_segment

```lua
function rotate_segment(angle: number, x1: number, y1: number, x2: number, y2: number)
  -> number * 4
```

## rotate_segment_around_origin

```lua
function rotate_segment_around_origin(angle: number, x1: number, y1: number, x2: number, y2: number, ox: number, oy: number)
  -> number * 4
```

## rotate_triangle

```lua
function rotate_triangle(angle: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number * 6
```

## rotate_triangle_around_origin

```lua
function rotate_triangle_around_origin(angle: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, ox: number, oy: number)
  -> number * 6
```

## scale_aabb

```lua
function scale_aabb(sx: number, sy: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number * 4
```

## scale_point

```lua
function scale_point(sx: number, sy: number, x: number, y: number)
  -> number, number
```

## scale_points

```lua
function scale_points(sx: number, sy: number, points: number[])
  -> table
```

@*param* `points` — (x1, y1, x2, y2, ...)

## scale_points_in_place

```lua
function scale_points_in_place(sx: number, sy: number, points: number[])
  -> number[]
```

@*param* `points` — (x1, y1, x2, y2, ...)

## scale_quad

```lua
function scale_quad(sx: number, sy: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number * 8
```

## scale_segment

```lua
function scale_segment(sx: number, sy: number, x1: number, y1: number, x2: number, y2: number)
  -> number * 4
```

## scale_triangle

```lua
function scale_triangle(sx: number, sy: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number * 6
```

## segment_within_aabb

```lua
function segment_within_aabb(x1: number, y1: number, x2: number, y2: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> boolean
```

## sq_distance

```lua
function sq_distance(x1: number, y1: number, x2: number, y2: number)
  -> number
```

Squared distance between two points

## translate_aabb

```lua
function translate_aabb(dx: number, dy: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> number * 4
```

## translate_points

```lua
function translate_points(dx: number, dy: number, points: number[])
  -> number[]
```

@*param* `points` — (x1, y1, x2, y2, ...)

## translate_points_in_place

```lua
function translate_points_in_place(dx: number, dy: number, points: number[])
  -> number[]
```

@*param* `points` — (x1, y1, x2, y2, ...)

## translate_quad

```lua
function translate_quad(dx: number, dy: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, x4: number, y4: number)
  -> number * 8
```

## translate_segment

```lua
function translate_segment(dx: number, dy: number, x1: number, y1: number, x2: number, y2: number)
  -> number * 4
```

## translate_triangle

```lua
function translate_triangle(dx: number, dy: number, x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> number * 6
```

## triangle_indices_from_polygon

```lua
function triangle_indices_from_polygon(polygon: number[], prefer_equilateral_triangles?: boolean)
  -> integer[]?
```

Triangulate a polygon with 4 or more sides using the ear-clipping method
Return a list of triangle indices, indexing into the points in polygon data

@*param* `polygon` — (x1, y1, x2, y2, ...)

@*param* `prefer_equilateral_triangles` — Sorts the clipped ears for more equilateral triangles (slower but nicer)

@*return* — (t1_i1, t1_i2, t1_i3, t2_i1, t2_i2, t2_i3, ...)

## triangle_within_aabb

```lua
function triangle_within_aabb(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number, min_x: number, min_y: number, max_x: number, max_y: number)
  -> boolean
```

## triangles_from_polygon

```lua
function triangles_from_polygon(polygon: number[], prefer_equilateral_triangles?: boolean)
  -> number[]?
```

Triangulate a polygon with 4 or more sides using the ear-clipping method
Return a list of triangles

@*param* `polygon` — (x1, y1, x2, y2, ...)

@*param* `prefer_equilateral_triangles` — Sorts the clipped ears for more equilateral triangles (slower but nicer)

@*return* — (tri1_x1, tri1_y1, tri1_x2, tri1_y2, tri1_x3, tri1_y3, tri2_x1, tri2_y1, tri2_x2, tri2_y2, tri2_x3, tri2_y3, ...)

## vertex_angle

```lua
function vertex_angle(x1: number, y1: number, x2: number, y2: number, x3: number, y3: number)
  -> degrees: number
```

## vertical_mirror

```lua
function vertical_mirror(angle: number)
  -> number
```

Reflect angle