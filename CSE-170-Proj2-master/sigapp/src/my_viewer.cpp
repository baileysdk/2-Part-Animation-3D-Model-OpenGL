# include "my_viewer.h"

# include <sigogl/ui_button.h>
# include <sigogl/ui_radio_button.h>
# include <sig/sn_primitive.h>
# include <sig/sn_transform.h>
# include <sig/sn_manipulator.h>

# include <sigogl/ws_run.h>

#include <random>
#include <functional>
#include <string>
//rotations
float IR_chap = 90.0f;
float R_chap = 0.0f;
float R_torso = 0.0f;
float R_rLeg = 0.0f;
float R_lLeg = 0.0f;
float R_arm = 0.0f;
float R_leg = 0.0f;
float rot = 0.0f;
float degrees = (float)GS_PI / 180;
//movement vectors
GsVec forward(0.0f, -3.0f, 0.0f);
GsVec backward(0.0f, 3.0f, 0.0f);
GsVec upward(0.0f, 0.0f, 3.0f);
GsVec downward(0.0f, 0.0f, -3.0f);
//swapping booleans
bool fixedCam = true;
bool foot = true;

bool completed = false;

GsVec direction(0.0f, -1.0f, 0.0f);

/*
	rootg()->add(...) order
	// Objects
	0.  bezPath
	1.	splash
	2.  torso_manip
	3.  _torus
	4.  _normals
	// Primitives
	5.  floor
	6.	pool
	7.  obj1
	8.  obj2
	9.  obj3
	// Control Points
	10.	base1
	11.	base2
	12.	base3
	13.	base4
*/

static float genPts(int i) {
	// Randomly Generate Intermediary Points
	
	std::random_device dev;
	std::mt19937 rng(dev());
	std::uniform_int_distribution<int> dx(-500, 500);
	std::uniform_int_distribution<int> dy( 0, 500);

	float p;
	if (i % 2 == 0) {
		p = float( dx(rng) );
	}
	else {
		p = float( dx(rng) );
	}
	
	return p;
}

void MyViewer::setPoly() {
	// Starting Point		: 0,    0
	// Intermediary Points  : random
	// Ending Point			: 0, -500
	poly = new SnPolyEditor;
	GsPolygon& P = poly->polygons()->push();

	float points[12];

	points[0] = 0.0f; points[1] = 0.0f; // Starting Point 0,0
	for (int i = 2; i < 10; i++) {
		points[i] = genPts(i);
	}

	//points = genPts(points);
	points[10] = 0.0f; points[11] = -500.0f; // Final Point 0,-500

	P.setpoly(points, 6); // Set Coordinates of Control Points (float* input)
}

// Functions from Lab 6 to Calc Bezier Curve
static int fact(int x) { // Calculate factorial values of input
	return (x <= 1) ? 1 : x * fact(x - 1);
}
static int bin(int n, int i) { // Calculate C(n,i) = n! / i!(n-i)! based on input
	if (i > n) { return 0; }
	else return fact(n) / (fact(i) * fact(n - i));
}
static GsPnt2 eval_bezier(float t, const GsArray<GsPnt2>& P)
{
	// Bezier of order n for given n = P.size()-1 control points, t in [0,1]
	// Nth Order Bezier Formula
	//		f(t) = sum (i=0 -> n) { P[i] * B i,n (t) }
	//		B(i,n)(t) = C(n,i) * t^i * (1-t)^n-1
	//		C(n,i) = n! / i!(n-i)!

	int n = P.size() - 1;

	GsPnt2 point(0, 0);
	int count;
	float b = 0.0f;

	for (count = 0; count <= n; count++) {
		b = bin(n, count) * pow(t, count) * pow(1.0f - t, n - count);
		point += P[count] * b;
	}

	return point;
}

void MyViewer::genBezier() {
	// Bezier Curve to have Path follow
	bezPath->init();
	GsPolygon& P = poly->polygon(0);

	bezPath->begin_polyline();
	for (float t = 0.0f; t < 1.0f; t += 0.01f) {
		bezPath->push(eval_bezier(t, P));
	}
	bezPath->end_polyline();

	SnPrimitive* p;
	GsPnt pt;
	//Generate numbers
	std::random_device dev;
	std::mt19937 rng(dev());;
	//gsout<<bezPath->V.size();
	// Generate "Bases"
	for (int i = 1; i < 5; i++) {
		//float x = P[i].x; float y = P[i].y;
		//x += (P[i - 1].x + P[i +1 ].x) / 2; y += (P[i - 1].y + P[i + 1].y) / 2;
		std::uniform_int_distribution<int> dpt(((i-1) * bezPath->V.size() / 4), (-10 + (i * bezPath->V.size() / 4)));
		pt = bezPath->V[dpt(rng)] - GsPnt(0,0,112);

		p = new SnPrimitive(GsPrimitive::Cylinder, 40.0f, 30.0f, 5.0f);
		p->prim().material.diffuse = GsColor::darkblue;
		p->prim().material.shininess = false;
		add_model(p, pt,90.0f);
	}
}

MyViewer::MyViewer ( int x, int y, int w, int h, const char* l ) : WsViewer(x,y,w,h,l)
{
	//checks
	_animating=false;
	_caming = false;
	_turning = false;
	_walking = false;

	//torus
	_torus_t = 1;
	_torus_n = 10;
	_torus_R = 200.0f;
	_torus_r = 20.0f;
	_flat = false;
	//translation matrices
	T_total.translation(GsVec(0.0f, 0.0f, 0.0f));
	T_torso.translation(GsVec(0.0f, 0.0f, 0.0f));
	T_chap.translation(GsVec(-2.2f, 0.0f, 38.4823f));
	T_lArm.translation(GsVec(43.4432f, -1.5f, 19.57585f));
	T_rArm.translation(GsVec(-25.5751f, -2.0f, 17.07585f));
	T_lLeg.translation(GsVec(7.9005f, -4.476375f, -30.46005f));
	T_rLeg.translation(GsVec(-12.7005f, -4.476375f, -30.46005f));
	T_splash.translation(GsVec(0.0f, -650.0f, -200.0f));
	//manipulators
	torso_manip = new SnManipulator;
	chap_manip = new SnManipulator;
	lArm_manip = new SnManipulator;
	rArm_manip = new SnManipulator;
	lLeg_manip = new SnManipulator;
	rLeg_manip = new SnManipulator;
	splash_manip = new SnManipulator;
	
	//scale
	S_chap.scaling(0.25f);
	S_Arm.scaling(1.2f);
	S_splash.scaling(0.25f);

	//determine rot
	X_chap.rotx(IR_chap * degrees);
	X_splash.rotx(90.0f * degrees);
	Z_torso.rotx(R_torso * degrees);
	Z_lLeg.rotx(R_lLeg * degrees);
	Z_rLeg.rotx(R_rLeg * degrees);

	// Bezier Path to follow
	rootg()->add( bezPath = new SnLines );
	bezPath->color(GsColor::magenta);
	bezPath->line_width(3.0f);

	setPoly();
	
	pathWalk = false;

	//build window
	build_ui ();
	build_scene ();
}

void MyViewer::build_ui ()
{
	UiPanel *p;
	UiManager* uim = WsWindow::uim();
	p = uim->add_panel ( "", UiPanel::HorizLeft );
	p->add ( new UiButton ( "Exit", EvExit ) ); p->top()->separate();
}

void MyViewer::add_model ( SnShape* s, GsVec p )
{
	// This method demonstrates how to add some elements to our scene graph: lines,
	// and a shape, and all in a group under a SnManipulator.
	// Therefore we are also demonstrating the use of a manipulator to allow the user to
	// change the position of the object with the mouse. If you do not need mouse interaction,
	// you can just use a SnTransform to apply a transformation instead of a SnManipulator.
	// You would then add the transform as 1st element of the group, and set g->separator(true).
	// Your scene graph should always be carefully designed according to your application needs.

	SnManipulator* manip = new SnManipulator;
	GsMat m;
	m.translation ( p );
	manip->initial_mat ( m );

	SnGroup* g = new SnGroup;
	SnLines* l = new SnLines;
	l->color(GsColor::orange);
	g->add(s);
	g->add(l);
	manip->child(g);
	manip->visible(false); // call this to turn off mouse interaction

	rootg()->add(manip);
}

void MyViewer::add_model(SnShape* s, GsVec p, float rot)
{

	SnManipulator* manip = new SnManipulator;
	GsMat m;
	GsMat rotx;
	m.translation(p);
	rotx.rotx(rot * degrees);
	manip->initial_mat(m*rotx);

	SnGroup* g = new SnGroup;
	SnLines* l = new SnLines;
	l->color(GsColor::orange);
	g->add(s);
	g->add(l);
	manip->child(g);
	manip->visible(false); // call this to turn off mouse interaction

	rootg()->add(manip);
}

void MyViewer::build_scene ()
{
	//splash
	GsModel* splash;
	splash = new GsModel();
	splash->load("../arm/splash.obj");
	SnModel* sn_splash;
	sn_splash = new SnModel(splash);
	sn_splash->color(GsColor::cyan);
	SnGroup* splash_sng = new SnGroup;

	splash_manip->visible(false);
	splash_manip->initial_mat(T_splash * X_splash * S_splash);

	splash_sng->add(sn_splash);
	splash_manip->child(splash_sng);
	rootg()->add(splash_manip);

	//torso
	GsModel* torso;
	torso = new GsModel();
	torso->load("../arm/torso.obj");
	SnModel* sn_torso;
	sn_torso = new SnModel(torso);
	SnGroup* torso_sng = new SnGroup;

	torso_manip->visible(false);
	torso_manip->initial_mat(T_torso * Z_torso);

	torso_sng->add(sn_torso);
	torso_manip->child(torso_sng);
	rootg()->add(torso_manip);
	
	//chappie head
	GsModel* chap;
	chap = new GsModel();
	chap->load("../arm/head.OBJ");
	SnModel* sn_chap;
	sn_chap = new SnModel(chap);
	SnGroup* chap_sng = new SnGroup;

	chap_manip->visible(false);
	chap_manip->initial_mat(T_chap /* X_chap /* S_chap*/);

	chap_sng->add(sn_chap);
	chap_manip->child(chap_sng);
	torso_sng->add(chap_manip);

	//right arm
	GsModel* rArm;
	rArm = new GsModel();
	rArm->load("../arm/rArm.obj");
	SnModel* sn_rArm;
	sn_rArm = new SnModel(rArm);
	SnGroup* rArm_sng = new SnGroup;

	rArm_manip->visible(false);
	rArm_manip->initial_mat(T_rArm * Z_rArm * S_Arm);

	rArm_sng->add(sn_rArm);
	rArm_manip->child(rArm_sng);
	torso_sng->add(rArm_manip);

	//left arm
	GsModel* lArm;
	lArm = new GsModel();
	lArm->load("../arm/lArm.obj");
	SnModel* sn_lArm;
	sn_lArm = new SnModel(lArm);
	SnGroup* lArm_sng = new SnGroup;

	lArm_manip->visible(false);
	lArm_manip->initial_mat(T_lArm * Z_lArm * S_Arm);

	lArm_sng->add(sn_lArm);
	lArm_manip->child(lArm_sng);
	torso_sng->add(lArm_manip);

	//right leg
	GsModel* rLeg;
	rLeg = new GsModel();
	rLeg->load("../arm/rLeg.obj");
	SnModel* sn_rLeg;
	sn_rLeg = new SnModel(rLeg);
	SnGroup* rLeg_sng = new SnGroup;

	rLeg_manip->visible(false);
	rLeg_manip->initial_mat(T_rLeg * Z_rLeg);

	rLeg_sng->add(sn_rLeg);
	rLeg_manip->child(rLeg_sng);
	torso_sng->add(rLeg_manip);

	//left leg
	GsModel* lLeg;
	lLeg = new GsModel();
	lLeg->load("../arm/lLeg.obj");
	SnModel* sn_lLeg;
	sn_lLeg = new SnModel(lLeg);
	SnGroup* lLeg_sng = new SnGroup;

	lLeg_manip->visible(false);
	lLeg_manip->initial_mat(T_lLeg * Z_lLeg);

	lLeg_sng->add(sn_lLeg);
	lLeg_manip->child(lLeg_sng);
	torso_sng->add(lLeg_manip);
	
	//torus
	_torus = new SnModel;
	_torus->color(GsColor::red);   // using node color (instead of model materials)
	_torus->visible(true);
	add_model(_torus,GsVec(0.0f,0.0f,-100.0f));

	_normals = new SnLines;
	_normals->visible(false);
	rootg()->add(_normals);

	make_torus(_flat);
	texture_torus(_torus_t);
	//primitives
	SnPrimitive* p;

	p = new SnPrimitive(GsPrimitive::Box, 500.0f, 500.0f, 0.3f);
	p->prim().material.diffuse = GsColor::brown;
	p->prim().material.shininess = false;
	add_model(p, GsVec(0.0f, 0.0f, -121.0f));

	p = new SnPrimitive(GsPrimitive::Cylinder, 20.0f,20.0f,10.0f);
	p->prim().material.diffuse = GsColor::red;
	p->prim().material.shininess = true;
	add_model(p, GsVec(50.0f, 50.0f, -100.0f));

	p = new SnPrimitive(GsPrimitive::Ellipsoid, 10.0f, 2.0f, 5.0f);
	p->prim().material.diffuse = GsColor::green;
	p->prim().material.shininess = true;
	add_model(p, GsVec(-70.0f, 50.0f, -100.0f));

	p = new SnPrimitive(GsPrimitive::Box, 20.0f, 20.0f, 20.0f);
	p->prim().material.diffuse = GsColor::darkblue;
	p->prim().material.shininess = true;
	add_model(p, GsVec(50.0f, -50.0f, -100.0f));

	p = new SnPrimitive(GsPrimitive::Box, 500.0f, 500.0f, 0.3f);
	p->prim().material.diffuse = GsColor::cyan;
	p->prim().material.shininess = false;
	add_model(p, GsVec(0.0f, -500.0f, -121.0f));

	genBezier();
	
}

void MyViewer::texture_torus(int file) {
	//	1. Access GsModel
	GsModel& m = *_torus->model();
	//	2. Create a “material group”: 
	GsModel::Group& g = *m.G.push();
	g.fi = 0;										// The group starts at first face,
	g.fn = m.F.size();								// convers all faces,
	g.dmap = new GsModel::Texture;					// and will be textured,
	g.dmap->fname.set("../arm/1.png");	// with this image
	// 3. Make sure the number of materials matches the number of groups:
	m.M.push().init();			// Only the diffuse component will come from the texture,
	m.M.top() = GsMaterial();	// so add here any material properties you’d like
	//	4. Now you can add texture coordinates to be used per vertex:
	int nv = m.V.size();
	m.T.size(nv);				// set same size as m.V array 
	double inc = GS_2PI / _torus_n;
	double theta = 0;
	double phi = 0;

	for (int i = 0; i < nv; i += 4) {	// Compute your coordinates and put them in T[i]
		m.T[i].set((theta) / GS_PI, (phi) / GS_PI);
		m.T[i + 1].set((theta + inc) / GS_PI, (phi) / GS_PI);
		m.T[i + 2].set((theta) / GS_PI, (phi + inc) / GS_PI);
		m.T[i + 3].set((theta + inc) / GS_PI, (phi + inc) / GS_PI);

		theta += inc;
		phi += inc;
	}
	// 5. Set parameters to enable texturing:
	m.set_mode(GsModel::Smooth, GsModel::PerGroupMtl);
	m.textured = true;



}

inline GsVec evaluate_torus_point(double R, double r, double theta, double phi)
{
	return GsVec((R + r * cos(theta)) * cos(phi), (R + r * cos(theta)) * sin(phi), r * sin(theta));
}

inline GsVec torus_axis_point(double R, double r, double theta, double phi)
{
	return GsVec(R * cos(phi), R * sin(phi), 0.0);
}

void MyViewer::make_torus(bool flat)
{
	GsModel* m = _torus->model();  // access model definition class in the node
	m->init();
	double inc = GS_2PI / _torus_n;
	GsVec axisp;

	// (Note: this implementation can still be improved to remove all duplicate vertices!)
	for (double theta = 0; theta <= GS_2PI; theta = theta + inc)
	{
		m->V.push() = evaluate_torus_point(_torus_R, _torus_r, theta, 0);
		m->V.push() = evaluate_torus_point(_torus_R, _torus_r, theta + inc, 0);

		if (!flat)
		{
			axisp = torus_axis_point(_torus_R, _torus_r, theta, 0);
			m->N.push() = normalize(m->V.top(1) - axisp);
			m->N.push() = normalize(m->V.top() - axisp);
		}

		for (double phi = 0; phi <= GS_2PI; phi = phi + inc)
		{
			m->V.push() = evaluate_torus_point(_torus_R, _torus_r, theta, phi + inc);
			m->V.push() = evaluate_torus_point(_torus_R, _torus_r, theta + inc, phi + inc);
			int i = m->V.size() - 4;
			m->F.push() = GsModel::Face(i, i + 2, i + 3);
			m->F.push() = GsModel::Face(i + 3, i + 1, i);

			if (!flat)
			{
				axisp = torus_axis_point(_torus_R, _torus_r, theta, phi + inc);
				m->N.push() = normalize(m->V.top(1) - axisp);
				m->N.push() = normalize(m->V.top() - axisp);
			}
		}
	}

	if (flat)
	{
		for (int i = 0; i < _torus->model()->F.size(); i++)
		{
			const GsVec& a = m->V[m->F[i].a];
			const GsVec& b = m->V[m->F[i].b];
			const GsVec& c = m->V[m->F[i].c];
			m->N.push() = cross(b - a, c - a);
			m->N.top().normalize();
		}
		_torus->model()->set_mode(GsModel::Flat, GsModel::NoMtl);
	}
	else
	{
		m->set_mode(GsModel::Smooth, GsModel::NoMtl);
	}

	// update normals if they are visible:
	if (_normals->visible()) make_normals(_flat);
}

void MyViewer::make_normals(bool flat)
{
	float my_scale_factor = 0.15f;
	SnLines* l = _normals;
	l->init();
	l->line_width(2.0f);
	l->color(GsColor::darkgreen);
	if (flat)
	{
		for (int i = 0; i < _torus->model()->F.size(); i++)
		{
			const GsVec& a = _torus->model()->V[_torus->model()->F[i].a];
			const GsVec& b = _torus->model()->V[_torus->model()->F[i].b];
			const GsVec& c = _torus->model()->V[_torus->model()->F[i].c];
			GsVec fcenter = (a + b + c) / 3.0f;
			l->push(fcenter, fcenter + normalize(cross(b - a, c - a)) * my_scale_factor);
		}
	}
	else
	{
		for (int i = 0; i < _torus->model()->V.size(); i++)
		{
			l->push(_torus->model()->V[i], _torus->model()->V[i] + _torus->model()->N[i] * my_scale_factor);
		}
	}
}

void MyViewer::set_camera () {

	if (_caming) { fixedCam = !fixedCam; return; } // avoid recursive calls
	_caming = true;
	GsMat camRot;
	camRot.roty(degrees / 5.0f);

	if (fixedCam == true) {
		camera().eye = GsVec(0.0, 1.0, 1.0);
		camera().center = GsVec(0.0, 150.0, 0.0);
		camera().up = GsVec(0.0, 1.0, 0.0);
		camera().translate(GsVec(0.0, -1000.0, 0.0));
		render();
		ws_check();
	}
	else {
		view_all();
		camera().eye = GsVec(0.0, 0.0, 1000.0);
		camera().translate(GsVec(0.0, -1000.0, 0.0));
		camera().center = (GsVec(0.0, -500.0, 0.0));
		render();
		ws_check();

		/*double lt, t0 = gs_time();
		do { 
			lt = (gs_time() - t0);
		//camera().eye.z += 0.001f;
		camera().center = GsVec(0.0,0.0,0.0);
		//camera().up.z += 0.001f;
			camera().rotate(camRot);
			render();
			ws_check();
			message().setf("localtime = % f", lt);
		} while (lt <= 2.5f);*/
	}

	_caming = false;
}

void MyViewer::turn_animation (bool c) {
	if (_turning) return;
	_turning = true;

	int s = 30;
	double frdt = 1.0 / (s); // delta time to reach given number of frames per second
	double t = 0, lt = 0, t0 = gs_time();
	R_rLeg = 0.0f;
	R_lLeg = 0.0f;
	R_chap = 0.0f;

	double i = 0.0;

	if (c) {
		do // run for a while:
		{
			while (t - lt < frdt) { ws_check(); t = gs_time() - t0; } // wait until it is time for next frame
			lt = t;
			if (i < 3) { // "rotate" torso
				R_rLeg += 6.0f; Z_rLeg.rotz(R_rLeg * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
				R_lLeg += 6.0f; Z_lLeg.rotz(R_lLeg * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
				Z_chap.roty(-R_rLeg * degrees); chap_manip->initial_mat(T_chap /* X_chap */ * Z_chap /* S_chap*/);
				redraw();
				i++;
			}
			else if (i < (6)) { // rotate legs
				R_rLeg -= 6.0f; Z_rLeg.rotz(R_rLeg * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
				R_lLeg -= 6.0f; Z_lLeg.rotz(R_lLeg * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
				Z_chap.roty(-R_rLeg * degrees); chap_manip->initial_mat(T_chap /* X_chap */ * Z_chap /* S_chap*/);
				redraw();
				i++;
			}
			render(); // notify it needs redraw
			ws_check(); // redraw now
		} while (i < (6));
	}
	else {
		do // run for a while:
		{
			while (t - lt < frdt) { ws_check(); t = gs_time() - t0; } // wait until it is time for next frame
			lt = t;
			if (i < 3) { // "rotate" torso/head
				R_rLeg -= 6.0f; Z_rLeg.rotz(R_rLeg * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
				R_lLeg -= 6.0f; Z_lLeg.rotz(R_lLeg * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
				Z_chap.roty(-R_rLeg * degrees); chap_manip->initial_mat(T_chap /* X_chap */ * Z_chap /* S_chap*/);
				redraw();
				i++;
			}
			else if (i < (6)) { // rotate legs/head
				R_rLeg += 6.0f; Z_rLeg.rotz(R_rLeg * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
				R_lLeg += 6.0f; Z_lLeg.rotz(R_lLeg * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
				Z_chap.roty(-R_rLeg * degrees); chap_manip->initial_mat(T_chap /* X_chap */ * Z_chap /* S_chap*/);
				redraw();
				i++;
			}
			render(); // notify it needs redraw
			ws_check(); // redraw now
		} while (i < (6));

	}
	_turning = false;
}

void MyViewer::walk_animation(bool f) {
	if (_walking) return;
	_walking = true;

	R_leg = 0.0f;
	int s = 30;
	double frdt = 1.0 / (s); // delta time to reach given number of frames per second
	double t = 0, lt = 0, t0 = gs_time();

	double i = 0.0;
	int foot;
	if (f) foot = +1; else foot = -1;

	do // run for a while:
	{
		while (t - lt < frdt) { ws_check(); t = gs_time() - t0; } // wait until it is time for next frame
		lt = t;
		if (i < 3) { // move limbs forward
			rot += foot*3.0f; Z_rLeg.rotx(rot * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			Z_lLeg.rotx(-rot * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
			Z_rArm.rotx(-rot * degrees); rArm_manip->initial_mat(T_rArm * Z_rArm);
			Z_lArm.rotx(rot * degrees); lArm_manip->initial_mat(T_lArm * Z_lArm);
			redraw();
			i++;
		}
		else if (i < (6)) { // move limbs backward
			rot -= foot*3.0f; Z_rLeg.rotx(rot * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			Z_lLeg.rotx(-rot * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
			Z_rArm.rotx(-rot * degrees); rArm_manip->initial_mat(T_rArm * Z_rArm);
			Z_lArm.rotx(rot * degrees); lArm_manip->initial_mat(T_lArm * Z_lArm);
			redraw();
			i++;
		}
		render(); // notify it needs redraw
		ws_check(); // redraw now
	} while (i < (6));
	_walking = false;
}

void MyViewer::walk_the_path() {
	if (_animating) return;
	if (completed) { // reset position if the path animation already occurred
		T_torso.translation( GsVec(0.0f, 0.0f, 0.0f) );

		rot = 0.0f;
		Z_torso.rotz(rot * degrees); torso_manip->initial_mat(T_torso * Z_torso);
		Z_rLeg.rotx(rot * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
		Z_lLeg.rotx(-rot * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
		Z_rArm.rotx(-rot * degrees); rArm_manip->initial_mat(T_rArm * Z_rArm);
		Z_lArm.rotx(rot * degrees); lArm_manip->initial_mat(T_lArm * Z_lArm);

		setPoly(); // Reset the random control points
		// Need to remove old control points before drawing new Bezier
		rootg()->remove(10);	rootg()->remove(11);	rootg()->remove(12); rootg()->remove(13);
		genBezier(); // Redraw Bezier with new Control Points

		completed = false;
		return;
	}

	if (pathWalk) return;
	pathWalk = true;

	int s = 20;
	double frdt = 1.0 / s; // 20 FPS
	double t = 0, lt = 0, t0 = gs_time();

	int i = 1;
	int j = 0;

	int f; 
	if (foot) f = +1; else f = -1;

	do {
		while (t - lt < frdt) { ws_check(); t = gs_time() - t0; }
		lt = t;

		GsVec motion = bezPath->V[i] - bezPath->V[i-1];

		T_torso.lcombtrans(motion); // Translate on a Vector from one point of the Bezier Path to the next 

		Z_torso.rot(direction, bezPath->V[i]); // Rotate the Body to Face the direction of the Path

		torso_manip->initial_mat(T_torso * Z_torso);

		i++;

		// Can't call walk_animation() or else it is slow & choppy
		// *Repeated limb motion here*
		if (j < 3) { // move limbs forward
			rot += f * 3.0f; Z_rLeg.rotx(rot * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			Z_lLeg.rotx(-rot * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
			Z_rArm.rotx(-rot * degrees); rArm_manip->initial_mat(T_rArm * Z_rArm);
			Z_lArm.rotx(rot * degrees); lArm_manip->initial_mat(T_lArm * Z_lArm);
			redraw();
			j++;
		}
		else if (j < (6)) { // move limbs backward
			rot -= f * 3.0f; Z_rLeg.rotx(rot * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			Z_lLeg.rotx(-rot * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
			Z_rArm.rotx(-rot * degrees); rArm_manip->initial_mat(T_rArm * Z_rArm);
			Z_lArm.rotx(rot * degrees); lArm_manip->initial_mat(T_lArm * Z_lArm);
			redraw();
			j++;
		}
		else if (j == 6) { j = 0; }

		render();
		ws_check();
	} while ( i < bezPath->V.size() );
	pathWalk = false;
	completed = do_the_dive();
}

bool MyViewer::do_the_dive() {

	// avoid recursive calls
	_animating = true;

	int ind = gs_random(0, rootg()->size() - 1); // pick one child
	SnManipulator* manip = rootg()->get<SnManipulator>(ind); // access one of the manipulators
	GsMat m = manip->mat();

	// set angles
	GsArray<float> rT(5, 5);
	rT[0] = 10.0f;
	rT[1] = 10.0f;
	rT[2] = 180.0f;
	rT[3] = 40.0f;
	rT[4] = 20.0f;
	GsArray<float> rA(5,5);
	rA[0] = 20.0f;
	rA[1] = 150.0f;
	rA[2] = 30.0f;
	rA[3] = 80.0f;
	rA[4] = 40.0f;
	GsArray<float> rL(5, 5);
	rL[0] = 20.0f;
	rL[1] = 30.0f;
	rL[2] = 20.0f;
	rL[3] = 120.0f;
	rL[4] = 20.0f;
	GsArray<float> rH(5, 5);
	rH[0] = 20.0f;
	rH[1] = 20.0f;
	rH[2] = 20.0f;
	rH[3] = 20.0f;
	rH[4] = 20.0f;
	R_torso = rT[0]; Z_torso.rotz(R_torso * degrees); 
	R_arm = rA[0]; Z_lArm.rotx(R_arm * degrees); Z_rArm.rotx(R_arm * degrees);
	R_leg = rL[0]; Z_lLeg.rotx(R_leg * degrees); Z_rLeg.rotx(R_leg * degrees);
	R_chap = rH[0]; X_chap.rotx(R_chap * degrees);
	redraw();
	
	// calculate interpolated angle
	int s = 20;
	GsArray<float> rTi(5,5);
	GsArray<float> rAi(5, 5);
	GsArray<float> rLi(5, 5);
	GsArray<float> rHi(5, 5);
	for (int i = 0; i < rT.size()-1; i++) {
		rTi[i] = (rT[i] + R_torso) / (s);
		rAi[i] = (rA[i] + R_arm) / (s);
		rLi[i] = (rL[i] + R_leg) / (s);
		rHi[i] = (rH[i] + R_torso) / (s);
	}


	double frdt = 1.0 / (s); // delta time to reach given number of frames per second
	double t = 0, lt = 0, t0 = gs_time();

	double i = 0.0;

	do // run for a while:
	{
		while (t - lt < frdt) { ws_check(); t = gs_time() - t0; } // wait until it is time for next frame
		lt = t;
		if (i < s) { // pose
			R_leg -= rLi(1); Z_lLeg.rotx(R_leg * degrees); Z_rLeg.rotx(R_leg * degrees);
			lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
			rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			R_torso += rTi(1); Z_torso.rotx(R_torso*degrees); torso_manip->initial_mat(T_torso * Z_torso);
			redraw();
			i++;
		}
		else if (i < (2.0*s)) { // raise arms
			R_arm -= rAi(1); Z_lArm.rotx(R_arm * degrees); Z_rArm.rotx(R_arm * degrees);
			lArm_manip->initial_mat(T_lArm * Z_lArm * S_Arm);
			rArm_manip->initial_mat(T_rArm * Z_rArm * S_Arm);
			redraw();
			i++;
		}
		else if (i < (3.0*s)) { // jump into air
			R_leg += rLi(2); Z_lLeg.rotx(R_leg * degrees); Z_rLeg.rotx(R_leg * degrees);
			lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
			rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			T_torso.lcombtrans(4*upward); T_torso.lcombtrans(forward); torso_manip->initial_mat(T_torso * Z_torso);
			redraw();
			i++;
		}
		else if (i < (3.5*s)) { // bend at apex
			R_torso += rTi(2); Z_torso.rotx(R_torso * degrees); T_torso.lcombtrans(forward); torso_manip->initial_mat(T_torso * Z_torso);
			R_leg -= rLi(3); Z_lLeg.rotx(R_leg * degrees); Z_rLeg.rotx(R_leg * degrees);
			lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
			rLeg_manip->initial_mat(T_rLeg * Z_rLeg); 
			R_arm += rAi(3); Z_lArm.rotx(R_arm * degrees); Z_rArm.rotx(R_arm * degrees);
			lArm_manip->initial_mat(T_lArm * Z_lArm * S_Arm);
			rArm_manip->initial_mat(T_rArm * Z_rArm * S_Arm);
			redraw();
			i++;
		}
		else if (i < (4.0 * s)) { // dive into water
			R_leg += rLi(2); Z_lLeg.rotx(R_leg * degrees); Z_rLeg.rotx(R_leg * degrees);
			lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
			rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			R_arm -= rAi(3); Z_lArm.rotx(R_arm * degrees); Z_rArm.rotx(R_arm * degrees);
			lArm_manip->initial_mat(T_lArm * Z_lArm * S_Arm);
			rArm_manip->initial_mat(T_rArm * Z_rArm * S_Arm);
			R_torso += rTi(3); Z_torso.rotx(R_torso * degrees);
			T_torso.lcombtrans(7 * downward); T_torso.lcombtrans(forward); torso_manip->initial_mat(T_torso * Z_torso);
			redraw();
			i++;
		}
		else if (i < (4.5*s)) { // dive into water
			R_leg += rLi(2); Z_lLeg.rotx(R_leg * degrees); Z_rLeg.rotx(R_leg * degrees);
			lLeg_manip->initial_mat(T_lLeg * Z_lLeg);
			rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			R_arm -= rAi(3); Z_lArm.rotx(R_arm * degrees); Z_rArm.rotx(R_arm * degrees);
			lArm_manip->initial_mat(T_lArm * Z_lArm * S_Arm);
			rArm_manip->initial_mat(T_rArm * Z_rArm * S_Arm);
			R_torso += rTi(3); Z_torso.rotx(R_torso * degrees);
			T_torso.lcombtrans(7*downward); T_torso.lcombtrans(forward); torso_manip->initial_mat(T_torso * Z_torso);
			T_splash.lcombtrans(3 * upward); splash_manip->initial_mat(T_splash * X_splash * S_splash);
			redraw();
			i++;
		}
		else if (i < (5.0 * s)) { // finish splash
			T_torso.lcombtrans(7 * downward); T_torso.lcombtrans(forward); torso_manip->initial_mat(T_torso * Z_torso);
			T_splash.lcombtrans(3 * downward); splash_manip->initial_mat(T_splash * X_splash * S_splash);
			redraw();
			i++;
		}
		render(); // notify it needs redraw
		ws_check(); // redraw now
	} while (i < (5.0*s));
	//gsout << T_torso;
	//T_torso.lcombtrans(21 * downward); T_torso.lcombtrans(forward); torso_manip->initial_mat(T_torso * Z_torso); redraw();
	_animating = false;
	return true;
}

int MyViewer::handle_keyboard ( const GsEvent &e )
{
	int ret = WsViewer::handle_keyboard ( e ); // 1st let system check events
	if ( ret ) return ret;

	switch (e.key) {
	case GsEvent::KeyEsc: gs_exit(); return 1;

		//moving forward/backward
	case GsEvent::KeyUp: {T_torso.lcombtrans(forward);
		torso_manip->initial_mat(T_torso * Z_torso); walk_animation(foot); foot = !foot; redraw(); return 1; }
	case GsEvent::KeyDown: {T_torso.lcombtrans(backward);
		torso_manip->initial_mat(T_torso * Z_torso); walk_animation(!foot); foot = !foot; redraw(); return 1; }

		//turning left/right
	case GsEvent::KeyLeft: {R_torso +=8; Z_torso.rotz(R_torso * degrees); 
		forward.rotz(8.0f * degrees); backward.rotz(8.0f * degrees); 
		torso_manip->initial_mat(T_torso * Z_torso); turn_animation(false); redraw(); return 1; }
	case GsEvent::KeyRight: {R_torso -= 8.0f; Z_torso.rotz(R_torso * degrees); 
		forward.rotz(-8.0f * degrees); backward.rotz(-8.0f * degrees); 
		torso_manip->initial_mat(T_torso * Z_torso); turn_animation(true); redraw(); return 1; }
		case ' ': {fixedCam = !fixedCam; set_camera(); }

		//legs turned in/out
		case 'q': {R_rLeg -= 6.0f; Z_rLeg.rotz(R_rLeg * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			R_lLeg += 6.0f; Z_lLeg.rotz(R_lLeg * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg); redraw(); return 1; }
		case 'a': {R_rLeg += 6.0f; Z_rLeg.rotz(R_rLeg * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			R_lLeg -= 6.0f; Z_lLeg.rotz(R_lLeg * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg); redraw(); return 1; }
		
		//legs rotated opposite, backwards/forwards
		case 'w': {R_leg += 3.0f; Z_rLeg.rotx(R_leg * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			Z_lLeg.rotx(-R_leg * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg); redraw(); return 1; }
		case 's': {R_leg -= 3.0f; Z_rLeg.rotx(R_leg * degrees); rLeg_manip->initial_mat(T_rLeg * Z_rLeg);
			Z_lLeg.rotx(-R_leg * degrees); lLeg_manip->initial_mat(T_lLeg * Z_lLeg); redraw(); return 1; }

		//arms rotated opposite, forwards/backwards
		case 'e': { R_arm += 3.0f; Z_rArm.rotx(-R_arm * degrees); rArm_manip->initial_mat(T_rArm * Z_rArm);
			Z_lArm.rotx(R_arm * degrees); lArm_manip->initial_mat(T_lArm * Z_lArm); redraw(); return 1; }
		case 'd': { R_arm -= 3.0f; Z_rArm.rotx(-R_arm * degrees); rArm_manip->initial_mat(T_rArm * Z_rArm);
			Z_lArm.rotx(R_arm * degrees); lArm_manip->initial_mat(T_lArm * Z_lArm); redraw(); return 1; }
		//head turned left,right
		case 'r': {R_chap += 3.0f; Z_chap.roty(R_chap * degrees); chap_manip->initial_mat(T_chap /* X_chap */ * Z_chap /* S_chap*/);
			redraw(); return 1; }
		case 'f': {R_chap -= 3.0f; Z_chap.roty(R_chap * degrees); chap_manip->initial_mat(T_chap /* X_chap */ * Z_chap /* S_chap*/);
			redraw(); return 1; }

		
		case 'p': { walk_the_path(); redraw(); return 1; }
		case 'o': {return do_the_dive();}
				 /*case 'q': ru += 1; Zu.rotz(ru * degrees); u_manip->initial_mat(Tu * Zu * Y); redraw(); return 0;
		case 'a': ru -= 1; Zu.rotz(ru * degrees); u_manip->initial_mat(Tu * Zu * Y); redraw(); return 0;
		case 'w': rl -= 1; Zl.rotx(rl * degrees); l_manip->initial_mat(Tl * Zl); redraw(); return 0;
		case 's': rl += 1; Zl.rotx(rl * degrees); l_manip->initial_mat(Tl * Zl); redraw(); return 0;
		case 'e': rh -= 1; Zh.rotx(rh * degrees); h_manip->initial_mat(Th * Zh); redraw(); return 0;
		case 'd': rh += 1; Zh.rotx(rh * degrees); h_manip->initial_mat(Th * Zh); redraw(); return 0;
		*/
	}

	return 0;
}

int MyViewer::uievent ( int e )
{
	switch ( e ){
		case EvExit: gs_exit();
	}
	return WsViewer::uievent(e);
}
