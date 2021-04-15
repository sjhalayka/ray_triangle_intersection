#include "primitives.h"
#include "mesh.h"


vector<triangle> triangles;
vector<vec3> face_normals;
vector<vec3> vertices;
vector<vec3> vertex_normals;









/*
bool rayTriangleIntersect(
	const vec3& orig, const vec3& dir,
	const vec3& v0, const vec3& v1, const vec3& v2,
	float& t)
{
	// compute plane's normal
	vec3 v0v1 = v1 - v0;
	vec3 v0v2 = v2 - v0;
	// no need to normalize
	vec3 N = cross(v0v1, v0v2); // N 

	// Step 1: finding P

	// check if ray and plane are parallel ?
	float NdotRayDirection = dot(N, dir);
	if (fabs(NdotRayDirection) < 0.00001) // almost 0 
		return false; // they are parallel so they don't intersect ! 

	// compute d parameter using equation 2
	float d = dot(N, v0);

	// compute t (equation 3)
	t = (dot(N, orig) + d) / NdotRayDirection;
	// check if the triangle is in behind the ray
	if (t < 0) return false; // the triangle is behind 

	// compute the intersection point using equation 1
	vec3 P = orig + t * dir;

	// Step 2: inside-outside test
	vec3 C; // vector perpendicular to triangle's plane 

	// edge 0
	vec3 edge0 = v1 - v0;
	vec3 vp0 = P - v0;
	C = cross(edge0, vp0);
	if (dot(N, C) < 0) return false; // P is on the right side 

	// edge 1
	vec3 edge1 = v2 - v1;
	vec3 vp1 = P - v1;
	C = cross(edge1, vp1);
	if (dot(N, C) < 0)  return false; // P is on the right side 

	// edge 2
	vec3 edge2 = v0 - v2;
	vec3 vp2 = P - v2;
	C = cross(edge2, vp2);
	if (dot(N, C) < 0) return false; // P is on the right side; 

	return true; // this ray hits the triangle 
}
*/




bool RayIntersectsTriangle(const vec3 rayOrigin,
	const vec3 rayVector,
	const vec3 v0, const vec3 v1, const vec3 v2,
	vec3& outIntersectionPoint)
{
	const float EPSILON = 0.0000001f;
	vec3 vertex0 = v0;// inTriangle->vertex0;
	vec3 vertex1 = v1;// inTriangle->vertex1;
	vec3 vertex2 = v2;// inTriangle->vertex2;
	vec3 edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	h = cross(rayVector, edge2);
	a = dot(edge1, h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.
	f = 1.0f / a;
	s = rayOrigin - vertex0;
	u = f * dot(s, h);
	if (u < 0.0f || u > 1.0f)
		return false;
	q = cross(s, edge1);
	v = f * dot(rayVector, q);
	if (v < 0.0f || u + v > 1.0f)
		return false;

	// At this stage we can compute t to find out where the intersection point is on the line.

	float t = f * dot(edge2, q);

	if (t > EPSILON) // ray intersection
	{
		outIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}


int main(void)
{
	if (false == read_triangles_from_binary_stereo_lithography_file(triangles, "fractal.stl"))
	{
		cout << "Error: Could not properly read file fractal.stl" << endl;
		return 2;
	}

//	get_vertices_and_normals_from_triangles(triangles, face_normals, vertices, vertex_normals);

	vec3 pos = vec3(10, 0, 0);
	vec3 look_at_ray = vec3(-1, 0, 0);

	size_t intersection_count = 0;

	for (size_t i = 0; i < triangles.size(); i++)
	{
		vec3 v0 = vec3(triangles[i].vertex[0].x, triangles[i].vertex[0].y, triangles[i].vertex[0].z);
		vec3 v1 = vec3(triangles[i].vertex[1].x, triangles[i].vertex[1].y, triangles[i].vertex[1].z);
		vec3 v2 = vec3(triangles[i].vertex[2].x, triangles[i].vertex[2].y, triangles[i].vertex[2].z);

		vec3 outIntersectionPoint;

		bool found_intersection = RayIntersectsTriangle(pos,
			look_at_ray,
			v0, v1, v2,
			outIntersectionPoint);

		if (found_intersection)
		{
			intersection_count++;
		}
	}

	cout << intersection_count << endl;

	return 0;
}