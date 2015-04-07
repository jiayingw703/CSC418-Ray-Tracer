/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include <vector>
#include "scene_object.h"

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSquare, which is
	// defined on the xy-plane, with vertices (0.5, 0.5, 0), 
	// (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
	// (0, 0, 1).
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.

	// Transform ray into object space
	// The origin of ray in object space
	Point3D p = worldToModel * ray.origin;

	// The direction of ray in object space
	Vector3D v = worldToModel * ray.dir;

	// Since square lies on xy-plane, intersection point must have z = 0
	double t = (0 - p[2]) / v[2];

	// t must be positive
	if (t > 0)
	{
		// Calculate the intersection of ray and xy-plane
		Point3D point = p + t * v;
		double x = point[0];
		double y = point[1];

		// Determine if the point is inside the unit square
		if (x < 0.5 && x > -0.5 && y < 0.5 && y > -0.5)
		{
			// Check if there's already a valid intersection, update if this one is closer
			if (ray.intersection.none || t < ray.intersection.t_value)
			{
				ray.intersection.t_value = t;
				ray.intersection.point = modelToWorld * point;
				// (Ma) x (Mb) = (det M)M^-T(a x b)
				ray.intersection.normal = transNorm(worldToModel, Vector3D(0.0, 0.0, 1.0));
				ray.intersection.normal.normalize();
				ray.intersection.none = false;
				updateTextureCoordinate(ray, worldToModel);
				return true;
			}
		}
	}
	return false;
}

void UnitSquare::updateTextureCoordinate(Ray3D& ray, const Matrix4x4& worldToModel) {
	Point3D point = worldToModel * ray.intersection.point;
	ray.intersection.uvCoord = Point3D(point[0] + 0.5, 0.5 - point[1], 0);
}

bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSphere, which is centred 
	// on the origin.  
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.

	// Transform ray into object space

	// The origin of ray in object space
	Point3D p = worldToModel * ray.origin;

	// The direction of ray in object space
	Vector3D v = worldToModel * ray.dir;

	// Center of sphere (0, 0, 0)
	Point3D center(0.0, 0.0, 0.0);

	// Calculate quadratic coefficient
	double A = v.dot(v);
	double B = (p - center).dot(v);
	double C = (p - center).dot(p - center) - 1;
	double D = B * B - A * C;

	// D < 0 then no intersection
	if (D >= 0)
	{
		// Calculate lambda1 and lambda2
		double lambda[2] = {
			// Lambda 1
			-B / A + std::sqrt(D) / A,
			// Lambda 2
			-B / A - std::sqrt(D) / A,
		};

#define LAMBDA1 lambda[0]
#define LAMBDA2 lambda[1]

		double t = -1.0;

		// Select the positive min t value
		if (LAMBDA1 > 0 && LAMBDA2 < 0)
		{
			t = LAMBDA1;
		}
		else if (LAMBDA2 > 0)
		{
			t = LAMBDA2;
		}

		if (t > 0)
		{
			// Calculate visible intersection point
			Point3D point = p + t * v;

			// Check if there's already a valid intersection, update if this one is closer
			if (ray.intersection.none || t < ray.intersection.t_value)
			{
				ray.intersection.t_value = t;
				ray.intersection.point = modelToWorld * point;
				ray.intersection.normal = transNorm(worldToModel, point - center);
				ray.intersection.normal.normalize();
				ray.intersection.none = false;
				updateTextureCoordinate(ray, worldToModel);
				return true;
			}
		}
	}
	return false;
}

void UnitSphere::updateTextureCoordinate(Ray3D& ray, const Matrix4x4& worldToModel) {
	
	Point3D point = worldToModel * ray.intersection.point;
	
	double theta = acos(point[1]);
	double phi = atan2(point[2], point[0]);
	phi = phi < 0 ? phi + 2 * M_PI : phi;
	double u = 0.5 * (1.0 - phi * M_1_PI);
	double v = 1.0 - theta * M_1_PI;
	ray.intersection.uvCoord = Point3D(u, v, 0);
}

bool UnitCylinder::intersect(Ray3D& ray, const Matrix4x4& worldToModel, const Matrix4x4& modelToWorld) {
	// Unit cylinder, defined by
	// x^2 + z^2 = 1, -0.5 <= y <= 0.5

	// Transform ray into object space
	// The origin of ray in object space
	Point3D p = worldToModel * ray.origin;

	// The direction of ray in object space
	Vector3D v = worldToModel * ray.dir;

	// Calculate quadratic coefficient
	// x^2 + z^2 = 1
	// x = p[0] + t * v[0], z = p[2] + t * v[2]
	double A = v[0] * v[0] + v[2] * v[2];
	double B = v[0] * p[0] + v[2] * p[2];
	double C = p[0] * p[0] + p[2] * p[2] - 1;
	double D = B * B - A * C;

	// Intersection
	Intersection intersection;
	intersection.none = true;
	intersection.t_value = DBL_MAX;
	
	if (D >= 0)
	{
		// Calculate lambda1 and lambda2
		double lambda[2] = {
			// Lambda 1
			-B / A + std::sqrt(D) / A,
			// Lambda 2
			-B / A - std::sqrt(D) / A,
		};

		double t = -1.0;

		// Validate roots
		if (LAMBDA1 > 0 && LAMBDA2 < 0)
		{
			t = LAMBDA1;
		}
		else if (LAMBDA2 > 0)
		{
			t = LAMBDA2;
		}

		if (t > 0)
		{
			// Calculate intersection with quadratic wall
			Point3D point = p + t * v;

			// Check if intersect with cylinder
			if (point[1] < 0.5 && point[1] > -0.5)
			{
				// Update intersection
				intersection.t_value = t;
				intersection.point = modelToWorld * point;
				intersection.normal = transNorm(worldToModel, Vector3D(point[0], 0, point[2]));
				intersection.normal.normalize();
				intersection.none = false;
			}
		}
	}

	// Compute intersection with top & bottom caps
	// Bottom cap, has y = -0.5; norm (0, -1, 0)
	// Top cap,    has y =  0.5; norm (0, 1, 0)

	double t[2] = {
		// t of bottom cap
		(-0.5 - p[1]) / v[1],

		// t of top cap
		( 0.5 - p[1]) / v[1]
	};

	Vector3D norm[2] = {
		// normal of bottom cap
		Vector3D(0, -1, 0),

		// normal of top cap
		Vector3D(0, 1, 0)
	};

	for (int i = 0; i < 2; i++)
	{
		// Calculate intersection
		Point3D point = p + t[i] * v;

		// x^2 + z^2 < 1
		if (point[0] * point[0] + point[2] * point[2] < 1 && t[i] > 0
			&& t[i] < intersection.t_value)
		{
			intersection.t_value = t[i];
			intersection.point = modelToWorld * point;
			intersection.normal = transNorm(worldToModel, norm[i]);
			intersection.normal.normalize();
			intersection.none = false;
		}
	}

	if (!intersection.none && (ray.intersection.none || intersection.t_value < ray.intersection.t_value))
	{
		ray.intersection = intersection;
		return true;
	}

	return false;
}

void UnitCylinder::updateTextureCoordinate(Ray3D& ray, const Matrix4x4& worldToModel) {
	throw "Not yet implemented";
}