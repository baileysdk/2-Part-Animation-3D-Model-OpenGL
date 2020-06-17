# pragma once

# include <sig/sn_poly_editor.h>
# include <sig/sn_lines2.h>
# include <sig/sn_manipulator.h>

# include <sigogl/ui_button.h>
# include <sigogl/ws_viewer.h>
# include <sig/sn_model.h>
# include <sig/sn_lines.h>
# include <sig/gs_light.h>

// Viewer for this example:
class MyViewer : public WsViewer
{  protected :
	enum MenuEv { EvExit };
	bool _animating;
	bool _caming;
	bool _turning;
	bool _walking;
	int _torus_n, _torus_t;
	float _torus_R, _torus_r;
	SnModel* _torus;
	SnLines* _normals;
	bool _flat;
   public :
	SnManipulator* torso_manip;
	SnManipulator* chap_manip;
	SnManipulator* lArm_manip;
	SnManipulator* rArm_manip;
	SnManipulator* lLeg_manip;
	SnManipulator* rLeg_manip;
	SnManipulator* splash_manip;
	//body translation
	GsMat T_total;
	GsMat T_splash;
	GsMat T_torso;
	GsMat T_chap;
	GsMat T_lArm;
	GsMat T_rArm;
	GsMat T_lLeg;
	GsMat T_rLeg;
	//scale
	GsMat S_chap;
	GsMat S_Arm;
	GsMat S_splash;
	//x-rot
	GsMat X_chap;
	GsMat X_splash;
	//z-rot
	GsMat Z_chap;
	GsMat Z_torso;
	GsMat Z_lArm;
	GsMat Z_rArm;
	GsMat Z_lLeg;
	GsMat Z_rLeg;

	SnLines* bezPath;
	SnPolyEditor* poly;
	bool pathWalk;

	MyViewer ( int x, int y, int w, int h, const char* l );
	void set_camera();
	void turn_animation(bool c);
	void walk_animation(bool f);
	void walk_the_path();
	bool do_the_dive();
	void setPoly();
	void genBezier();
	void build_ui ();
	void add_model ( SnShape* s, GsVec p );
	void add_model(SnShape* s, GsVec p, float rot);
	void build_scene ();
	void texture_torus(int file);
	void make_torus(bool flat);
	void make_normals(bool flat);
	void run_animation ();
	virtual int handle_keyboard ( const GsEvent &e ) override;
	virtual int uievent ( int e ) override;
};

