/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <random>  
#include <ctime>

#include <omp.h>

Raytracer::Raytracer() : _root(new SceneDagNode()) {

}

Raytracer::~Raytracer() {
	// Clean up

	// Delete light sources
	for (int i = 0; i < _lightSource.size(); i++)
	{
		delete _lightSource[i];
	}

	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material *mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;
}

void Raytracer::addLightSource( LightSource* light ) {
	_lightSource.push_back(light);
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray, Matrix4x4 &_modelToWorld, Matrix4x4 &_worldToModel ) {
	SceneDagNode *childPtr;

	// Applies transformation of the current node to the global
	// transformation matrices.
	Matrix4x4 modelToWorld = _modelToWorld*node->trans;
	Matrix4x4 worldToModel = node->invtrans*_worldToModel; 
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, worldToModel, modelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray, modelToWorld, worldToModel);
		childPtr = childPtr->next;
	}

	// Removes transformation of the current node from the global
	// transformation matrices.
// 	_worldToModel = node->trans*_worldToModel;
// 	_modelToWorld = _modelToWorld*node->invtrans;
}

void Raytracer::computeShading( Ray3D& ray ) {

	Colour color(0, 0, 0);

	for (int i = 0; i < _lightSource.size(); i++)
	{
		LightSource *curLight = _lightSource[i];
		Colour curShade = curLight->shade(ray, _shadeMode);

		if (_shadowFlag)
		{
			/*
			Implementation of soft shadows, by sampling the light direction with
			random jitter, then compute average shadow shade
			*/
			double theta = 2 * M_PI * erand();
			Vector3D unitOffset(cos(theta), 0, sin(theta));
			Point3D randLightPos = curLight->get_position() + erand() * unitOffset;

			// Shadow direction
			Vector3D s_dir = randLightPos - ray.intersection.point;

			double lightDistance = s_dir.length();

			s_dir.normalize();

			// Shadow origin
			Point3D s_origin = ray.intersection.point + EPS * s_dir;

			// Shadow ray
			Ray3D s_ray(s_origin, s_dir);

			// Traverse scene using shadow ray
			traverseScene(_root, s_ray, _modelToWorld, _worldToModel);

			// Check if it's a shadow ray
			if (!s_ray.intersection.none && s_ray.intersection.t_value < lightDistance)
			{
				continue;
			}
		}
		color += curShade;
	}
	color.clamp();
	ray.col = color;
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray, int max_depth ) {
	Colour col(0.0, 0.0, 0.0); 

	traverseScene(_root, ray, _modelToWorld, _worldToModel); 
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {

		computeShading(ray);
		col = ray.col;

		// You'll want to call shadeRay recursively (with a different ray, 
		// of course) here to implement reflection/refraction effects.  

		// Continue iterate rays?
		if (max_depth > 0)
		{	
			// Add reflection
			// Calculate the primary reflection direction

			// Normal at intersection surface
			Vector3D N = ray.intersection.normal;

			// Incidence direction
			Vector3D I = -ray.dir;
			
			double R = 1.0;

			// Refraction
			if (ray.intersection.mat->transmissive)
			{	
				double n1 = 1.0;
				double n2 = 1.0;

				// Refraction color
				Colour refrColor(0, 0, 0);

				
				if (I.dot(N) > 0)
				{
					// Entering transmissive object
					n2 = ray.intersection.mat->refrac_idx;
				}
				else
				{
					// Leaving transmissive object
					n1 = ray.intersection.mat->refrac_idx;
					N = -N;
				}
				
				// Compute refraction direction
				double n = n1 / n2;
				double cosI = N.dot(I);
				double sinT2 = n * n * (1.0 - cosI * cosI);
				if (sinT2 <= 1)
				{
					double cosT = std::sqrt(1.0 - sinT2);
					Vector3D refr_dir = n * I - (n * cosI + cosT) * N;
					refr_dir.normalize();

					Point3D refr_origin = ray.intersection.point + EPS * refr_dir;
					Ray3D refr_ray(refr_origin, refr_dir);

					double Rs = (n1 * cosI - n2 * cosT) / (n1 * cosI + n2 * cosT);
					double Rp = (n1 * cosT - n2 * cosI) / (n1 * cosT + n2 * cosI);
					R = (Rs * Rs + Rp * Rp) / 2.0;

					refrColor = (1.0 - R) * ray.intersection.mat->getTextureColor(ray) * shadeRay(refr_ray, max_depth - 1);
				}

				// Add refraction
				col += refrColor;
			}

			// Primary reflection direction
			Vector3D refl_dir = 2 * N.dot(I) * N - I;
			refl_dir.normalize();

			// Reflection origin offset a little to avoid same intersection
			Point3D refl_origin = ray.intersection.point + EPS * refl_dir;

			// Reflection color
			Colour reflColor(0, 0, 0);

			// Glossy reflection
			Vector3D u = refl_dir.cross(N);
			u.normalize();
			Vector3D v = refl_dir.cross(u);
			v.normalize();

			// Generate random ray direction
			Ray3D refl_ray;
			refl_ray.origin = refl_origin;

			// Generate cosine weighted random hemisphere samples
			double theta = acos(pow(erand(), 1.0 / (ray.intersection.mat->specular_exp + 1)));
			double phi = 2 * M_PI * erand();
			double x = sin(theta) * cos(phi);
			double y = sin(theta) * sin(phi);
			double z = cos(theta);

			refl_ray.dir = x * u + y * v + z * refl_dir;
			refl_ray.dir.normalize();

			reflColor += shadeRay(refl_ray, max_depth - 1);

			// Add reflection
			col += R * ray.intersection.mat->specular * ray.intersection.mat->getTextureColor(ray) * reflColor;
		}
	}

	col.clamp();
	return col; 
}	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
	Vector3D up, double fov, int max_depth, int samples, double aperture, double focalLength, ShadeMode mode, bool shadowFlag, double gamma, char* fileName) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	_shadeMode = mode;
	_shadowFlag = shadowFlag;
	double factor = (height / 2.0) / tan(fov * M_PI / 360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	// Set up origin in view space
	Point3D origin(0, 0, 0);
	
	// Anti-Aliasing by super-sampling
	// Divide a single pixel into, e.g. 4 x 4 sub-pixels
	// Sqrt of total super-samples
	int sqrtSuperSamples = 4;

	// Total samples in a single pixel
	int superSamples = sqrtSuperSamples * sqrtSuperSamples;

	// Reciprocals
	double subPixWidth = 1.0 / sqrtSuperSamples;
	double sampleWeight = 1.0 / (superSamples * samples);

	// Enable OpenMP for parallel execution
#pragma omp parallel for schedule(dynamic, 1)
	for (int y = 0; y < _scrHeight; y++)
	{
		// Update progress
		fprintf(stderr, "\rRendering (%d ssp) %5.2f%%", superSamples * samples, 100.0 * y / (_scrHeight - 1));

		for (int x = 0; x < _scrWidth; x++)
		{
			// buffer offset
			int i = y * width + x;

			// Color at pixel (x, y)
			Colour color;

			// Loop over all samples and average all shades
			for (int sy = 0; sy < sqrtSuperSamples; sy++)
			{
				for (int sx = 0; sx < sqrtSuperSamples; sx++)
				{
					for (int s = 0; s < samples; s++)
					{
						double dy = (sy + erand()) * subPixWidth;
						double dx = (sx + erand()) * subPixWidth;

						// Sets up ray origin and direction in view space, 
						// image plane is at z = -1.
						Point3D imagePlane(
							(-width / 2.0 + x + dx) / factor,
							(-height / 2.0 + y + dy) / factor,
							-1.0
							);

						Vector3D directDir = imagePlane - origin;
						directDir.normalize();

						// TODO: Convert ray to world space and call 
						// shadeRay(ray) to generate pixel colour. 	

						// t value to focal plane
						double t_focal = -focalLength / directDir[2];

						// Focal point
						Point3D focalPoint = origin + t_focal * directDir;

						// Cast origin
						double phi = 2 * M_PI * erand();
						double r = aperture * sqrt(erand()) / 2.0;

						// Randomly choose cast ray origin on lens 
						Point3D castOrigin(r * cos(phi), r * sin(phi), 0);

						// Cast direction
						Vector3D castDir = focalPoint - castOrigin;

						// Construct cast ray
						Ray3D castRay;
						castRay.origin = viewToWorld * castOrigin;
						castRay.dir = viewToWorld * castDir;
						castRay.dir.normalize();
						
						// Assign pixel color as the average of all sub-pixels
						color += shadeRay(castRay, max_depth);
					}
				}
			}
			color *= sampleWeight;

			// Clamp the RGB value
			color.clamp();

			// Update pixel buffer with gamma correction (2.2)
			_rbuffer[i] = static_cast<unsigned char>(pow(color[0], 1 / gamma) * MAXRGB + 0.5);
			_gbuffer[i] = static_cast<unsigned char>(pow(color[1], 1 / gamma) * MAXRGB + 0.5);
			_bbuffer[i] = static_cast<unsigned char>(pow(color[2], 1 / gamma) * MAXRGB + 0.5);
		}
	}

	// Flush buffer, save image to file
	flushPixelBuffer(fileName);
}


int main(int argc, char* argv[])
{
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 320;
	int height = 240;

	int scene = 2;

	// Maximum trace depth
	int max_depth = 6; 

	// samples per pixel (ssp)
	int samples = 50;

	if (argc == 5) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
		samples = atoi(argv[3]) / 16;
		scene = atoi(argv[4]);
	}
	else {
		printf("Usage: raytracer width height samples scene( 0 for part A, generate 6 pictures. 1 for extended raytracer scene 1. 2 for extended raytracer scene 2\n");
		exit(-1);
	}

	// Loading texture map
	Texture worldmap("earth_physical.bmp");
	Texture checkerboard("checkerboard.bmp");

	// Define materials for shading
	Material gold(Colour(0.3, 0.3, 0.3), 
		Colour(0.75164, 0.60648, 0.22648),
		Colour(0.628281, 0.555802, 0.366065),
		51.2, 1, false, NULL);

	Material jade(Colour(0, 0, 0), 
		Colour(0.54, 0.89, 0.63),
		Colour(0.316228, 0.316228, 0.316228),
		12.8, 1, false, NULL);

	Material red(Colour(0.1, 0.0, 0.0), 
		Colour(0.4, 0.4, 0.4),
		Colour(0.6, 0.05, 0.05),
		1, 1, false, NULL);

	Material blue(Colour(0.0, 0.0, 0.1), 
		Colour(0.4, 0.4, 0.4),
		Colour(0.05, 0.05, 0.6),
		1, 1, false, NULL);

	Material white(Colour(0.01, 0.01, 0.01), 
		Colour(0.5, 0.5, 0.5),
		Colour(0.5, 0.5, 0.5),
		1, 1, false, NULL);

	Material earth(Colour(0.1, 0.1, 0.1), 
		Colour(0.8, 0.8, 0.8),
		Colour(0.1, 0.1, 0.1),
		10, 1, false, &worldmap);

	Material silver(Colour(0.19125, 0.19125, 0.19125), 
		Colour(0.50754, 0.50754, 0.50754),
		Colour(0.508273, 0.508273, 0.508273),
		100, 1.3, false, NULL);

	Material glass(Colour(0.001, 0.001, 0.001), 
		Colour(0.0, 0.0, 0.0),
		Colour(0.999, 0.999, 0.999),
		10000, 1.5, true, NULL);

	Material mirror(Colour(0.001, 0.001, 0.001),
		Colour(0.0, 0.0, 0.0),
		Colour(0.999, 0.999, 0.999),
		10000, 1.5, false, NULL);

	Material glossyMirror(Colour(0.01, 0.01, 0.01), 
		Colour(0.1, 0.1, 0.1),
		Colour(0.9, 0.9, 0.9),
		1000, 1.5, false, NULL);

	Material board(Colour(0.01, 0.01, 0.01),
		Colour(0.09, 0.09, 0.09),
		Colour(0.9, 0.9, 0.9),
		10000, 1.5, false, &checkerboard);


	// Load scene
	switch (scene)
	{
	case 0:
	{
		

		// Defines a point light source.
		raytracer.addLightSource(new PointLight(Point3D(0, 0, 5),
			Colour(0.9, 0.9, 0.9)));

		// Add a unit square into the scene with material mat.
		SceneDagNode* sphere = raytracer.addObject(new UnitSphere(), &gold);
		SceneDagNode* plane = raytracer.addObject(new UnitSquare(), &jade);

		// Apply some transformations to the unit square.
		double factor1[3] = { 1.0, 2.0, 1.0 };
		double factor2[3] = { 6.0, 6.0, 6.0 };
		raytracer.translate(sphere, Vector3D(0, 0, -5));
		raytracer.rotate(sphere, 'x', -45);
		raytracer.rotate(sphere, 'z', 45);
		raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

		raytracer.translate(plane, Vector3D(0, 0, -7));
		raytracer.rotate(plane, 'z', 45);
		raytracer.scale(plane, Point3D(0, 0, 0), factor2);

		// Render the scene, feel free to make the image smaller for
		// testing purposes.	

		char *filenames[3][2] = {
				{ "sig1.bmp", "sig2.bmp" },
				{"diffuse1.bmp", "diffuse2.bmp"},
				{"phong1.bmp", "phong2.bmp"}
		};

		for (int i = 0; i < 3; i++)
		{
			Point3D eye(0, 0, 1);
			Vector3D view(0, 0, -1);
			Vector3D up(0, 1, 0);
			double fov = 60;
			raytracer.render(width, height, eye, view, up, fov, static_cast<ShadeMode>(i), false, filenames[i][0]);

			// Render it from a different point of view.
			Point3D eye2(4, 2, 1);
			Vector3D view2(-4, -2, -6);
			raytracer.render(width, height, eye2, view2, up, fov, static_cast<ShadeMode>(i), false, filenames[i][1]);
		}
		break;
	}
	case 1:
	{
		// Scene 1

		// Camera parameters.
		Point3D eye(0, 2, 10);
		Vector3D view(0, 0, -1);
		Vector3D up(0, 1, 0);
		double fov = 60;
		double aperture = 0.6;
		double focalLength = 12;

		// Defines a point light source.
		raytracer.addLightSource(new PointLight(Point3D(0, 6, 3),
			Colour(0.2, 0.2, 0.2), Colour(0.8, 0.8, 0.8), Colour(0.8, 0.8, 0.8)));

		// Construct scene
		SceneDagNode* floor = raytracer.addObject(new UnitSquare(), &glossyMirror);
		SceneDagNode* ceiling = raytracer.addObject(new UnitSquare(), &white);
		SceneDagNode* leftWall = raytracer.addObject(new UnitSquare(), &blue);
		SceneDagNode* rightWall = raytracer.addObject(new UnitSquare(), &red);
		SceneDagNode* backWall = raytracer.addObject(new UnitSquare(), &white);
		SceneDagNode* cylinder = raytracer.addObject(new UnitCylinder(), &gold);
		SceneDagNode* earthSphere = raytracer.addObject(new UnitSphere(), &earth);
		SceneDagNode* mirrorSphere = raytracer.addObject(new UnitSphere(), &mirror);
		SceneDagNode* glassSphere = raytracer.addObject(new UnitSphere(), &glass);

		// Apply transformations
		double wallScale[3] = { 100.0, 100.0, 100.0 };
		raytracer.translate(floor, Vector3D(0, -3, 0));
		raytracer.rotate(floor, 'x', -90);
		raytracer.scale(floor, Point3D(0, 0, 0), wallScale);

		raytracer.translate(backWall, Vector3D(0, 0, -7));
		raytracer.scale(backWall, Point3D(0, 0, 0), wallScale);

		raytracer.translate(leftWall, Vector3D(-7, 0, 0));
		raytracer.rotate(leftWall, 'y', 90);
		raytracer.scale(leftWall, Point3D(0, 0, 0), wallScale);

		raytracer.translate(rightWall, Vector3D(7, 0, 0));
		raytracer.rotate(rightWall, 'y', -90);
		raytracer.scale(rightWall, Point3D(0, 0, 0), wallScale);

		raytracer.translate(ceiling, Vector3D(0, 7, 0));
		raytracer.rotate(ceiling, 'x', 90);
		raytracer.scale(ceiling, Point3D(0, 0, 0), wallScale);

		double cylinderScale[3] = { 1.5, 2.0, 1.5 };
		raytracer.translate(cylinder, Vector3D(-4, -2, -4));
		raytracer.scale(cylinder, Point3D(0, 0, 0), cylinderScale);

		double sphereScale[3] = { 2.0, 2.0, 2.0 };
		raytracer.translate(earthSphere, Vector3D(3, 3, -3));
		raytracer.rotate(earthSphere, 'y', -25);
		raytracer.rotate(earthSphere, 'z', -23.5);
		raytracer.scale(earthSphere, Point3D(0, 0, 0), sphereScale);

		raytracer.translate(mirrorSphere, Vector3D(-4, 0.9, -4));
		raytracer.scale(mirrorSphere, Point3D(0, 0, 0), sphereScale);

		raytracer.translate(glassSphere, Vector3D(1, -0.9, -0.5));
		raytracer.scale(glassSphere, Point3D(0, 0, 0), sphereScale);


		raytracer.render(width, height, eye, view, up, fov, max_depth, samples, aperture, focalLength, WithSpecular, true, 2.2, "extended_raytracing_scene_1.bmp");
		break;
	}
	case 2:
	{
		// Scene 2

		// Camera parameters.
		Point3D eye(0, 4, 3);
		Vector3D view(0, -0.2, -1);
		Vector3D up(0, 1, 0);
		double fov = 90;
		double aperture = 0.6;
		double focalLength = 5.5;

		raytracer.addLightSource(new PointLight(Point3D(0, 5, 4),
			Colour(0.1, 0.1, 0.1),
			Colour(0.9, 0.9, 0.9),
			Colour(0.9, 0.9, 0.9)));

		raytracer.addLightSource(new PointLight(Point3D(-4, 4, 3),
			Colour(0.0, 0.0, 0.0),
			Colour(0.9, 0.9, 0.9),
			Colour(0.9, 0.9, 0.9)));

		SceneDagNode* ground = raytracer.addObject(new UnitSquare(), &board);
		double groundScale[3] = { 30, 30, 30 };
		raytracer.rotate(ground, 'x', -90);
		raytracer.scale(ground, Point3D(), groundScale);

		SceneDagNode* redSphere = raytracer.addObject(new UnitSphere(), &white);
		double sphereScale[3] = { 2.0, 2.0, 2.0 };
		raytracer.translate(redSphere, Vector3D(4, 2, -2));
		raytracer.scale(redSphere, Point3D(), sphereScale);

		SceneDagNode* earthSphere = raytracer.addObject(new UnitSphere(), &earth);
		raytracer.translate(earthSphere, Vector3D(0, 2, -4));
		raytracer.scale(earthSphere, Point3D(), sphereScale);

		SceneDagNode* mirrorSphere = raytracer.addObject(new UnitSphere(), &mirror);
		raytracer.translate(mirrorSphere, Vector3D(-6, 2, -6));
		raytracer.scale(mirrorSphere, Point3D(), sphereScale);

		SceneDagNode* glassSphere = raytracer.addObject(new UnitSphere(), &glass);
		raytracer.translate(glassSphere, Vector3D(0.4, 1.0, -0.5));

		SceneDagNode* cylinder = raytracer.addObject(new UnitCylinder(), &gold);
		double cylinderScale[3] = { 0.8, 4.0, 0.8 };
		raytracer.translate(cylinder, Vector3D(-3.0, 0.81, -2.0));
		raytracer.rotate(cylinder, 'y', 60);
		raytracer.rotate(cylinder, 'x', 90);
		raytracer.scale(cylinder, Point3D(), cylinderScale);

		raytracer.render(width, height, eye, view, up, fov, max_depth, samples, aperture, focalLength, WithSpecular, true, 2.2, "extended_raytracing_scene_2.bmp");
		break; 
	}
	}

	return 0;
}

