/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <algorithm>
#include "light_source.h"

Colour PointLight::shade(Ray3D& ray, ShadeMode mode) {
	// TODO: implement this function to fill in values for ray.col 
	// using phong shading.  Make sure your vectors are normalized, and
	// clamp colour values to 1.0.
	//
	// It is assumed at this point that the intersection information in ray 
	// is available.  So be sure that traverseScene() is called on the ray 
	// before this function.  

	// Material at intersection
	Material *mat = ray.intersection.mat;

	// Light source direction from the intersection
	Vector3D L = _pos - ray.intersection.point;
	L.normalize();

	// Surface normal at intersection
	Vector3D N = ray.intersection.normal;
	N.normalize();

	if (mat->transmissive && N.dot(L) < 0)
	{
		N = -N;
	}

	// Estimation of the mirror reflection direction
	Vector3D R = 2 * N.dot(L) * N - L;
	R.normalize();

	// Viewing direction, which is opposite to the ray direction
	Vector3D V = -ray.dir;
	V.normalize();

	
	// Ambient light intensity
	Colour Ia = mat->ambient * _col_ambient;

	// Diffuse reflection intensity
	Colour Id = mat->diffuse * (std::max(0.0, N.dot(L)) * _col_diffuse);

	// Specular reflection intensity
	Colour Is = mat->specular * (std::max(0.0, std::pow(R.dot(V), mat->specular_exp)) * _col_specular);


	// Different shade mode
	Colour color;
	switch (mode)
	{
	case SceneSignature:
		color = mat->diffuse;
		break;
	case WithoutSpeular:
		color = Ia + Id;
		break;
	case WithSpecular:
		color = Ia + Id + Is;
		break;
	default:
		break;
	}
	
	// Multiply by it's texture color
	color = color *  mat->getTextureColor(ray);
	return color;
}

