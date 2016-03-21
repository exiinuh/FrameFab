#pragma once

#ifndef RENDERINGWIDGET_H
#define RENDERINGWIDGET_H

#include <iostream>
#include <algorithm>

#include <QtWidgets/QMenu>
#include <QtWidgets/QAction>
#include <QGLWidget>
#include <QEvent>
#include <QKeyEvent>
#include <QColorDialog>
#include <QFileDialog>
#include <QTextCodec>

#include <gl\GLU.h>
#include <glut.h>

#include "mainwindow.h"
#include "ArcBall.h"
#include "globalFunctions.h"
#include "FiberPrint\FiberPrintPlugIn.h"
#include "WireFrame\WireFrame.h"

enum OperationMode
{
	NORMAL,
	CHOOSEBASE,
	CHOOSECEILING,
};


class MainWindow;
class CArcBall;
class Mesh3D;

class RenderingWidget : public QGLWidget
{
	Q_OBJECT

public:
	RenderingWidget(QWidget *parent, MainWindow* mainwindow = 0);
	~RenderingWidget();

public:
	void	InitDrawData();
	void	InitCapturedData();
	void	InitFiberData();

protected:
	void	initializeGL();
	void	resizeGL(int w, int h);
	void	paintGL();
	void	timerEvent(QTimerEvent *e);

	// mouse events
	void	mousePressEvent(QMouseEvent *e);
	void	mouseMoveEvent(QMouseEvent *e);
	void	mouseReleaseEvent(QMouseEvent *e);
	void	mouseDoubleClickEvent(QMouseEvent *e);
	void	wheelEvent(QWheelEvent *e);

public:
	void	keyPressEvent(QKeyEvent *e);
	void	keyReleaseEvent(QKeyEvent *e);

signals:
	void	ChooseBasePressed(bool);
	void	ChooseCeilingPressed(bool);

	void	SetOrderSlider(int);
	void	SetMaxOrderSlider(int);

	void	meshInfo(int, int);
	void	operatorInfo(QString);
	void	modeInfo(QString);
	void	CapturedVert(int, int);
	void	CapturedEdge(int, double);

	void	Error(QString);

	void	Reset();

private:
	void	CoordinatesTransform(QPoint p, double *x, double *y, double *z);
	bool	CaptureVertex(QPoint mouse);
	bool	CaptureEdge(QPoint mouse);

	void	Render();
	void	SetLight();

public slots:
	void	SetBackground();
	void	ScaleFrame(double scale);

	void	ReadFrame();
	void	WriteFrame(QString filename);
	void	WriteFrame(bool bVert, bool bLine, 
						bool bBase, bool bCeiling, bool bCut,
						int min_layer, int max_layer, QString filename);
	void	ImportFrame();
	void	ExportFrame(int min_layer, int max_layer, 
						QString vert_path, QString line_path);

	void	CheckDrawPoint(bool bv);
	void	CheckEdgeMode(int type);
	void	CheckLight(bool bv);
	void	CheckDrawAxes(bool bv);

	void	SwitchToNormal();
	void	SwitchToChooseBase();
	void	SwitchToChooseCeiling();

private:
	void	DrawAxes(bool bv);
	void	DrawPoints(bool bv);
	void	DrawEdge(bool bv);
	void	DrawHeat(bool bv);
	void	DrawBulk(bool bv);
	void	DrawOrder(bool bv);

public slots:
	void	FiberPrintAnalysis(double radius, double density, double g,
								double youngs_modulus, double shear_modulus,
								double Dt_tol, double Dr_tol,
								double penalty, double pri_tol, double dual_tol,
								double gamma, double Wl, double Wp);

	void	SimplifyFrame();
	void	RefineFrame();
	void DebugFrame();
	void	ProjectBound(double len);
	void	ModifyProjection(double len);

	void	RotateXY();
	void	RotateXZ();
	void	RotateYZ();

	void	PrintOrder(int order);
	void	PrintNextStep();
	void	PrintNextLayer();

public:
	MainWindow		*ptr_mainwindow_;
	CArcBall		*ptr_arcball_;
	WireFrame		*ptr_frame_;

	// eye
	GLfloat			eye_distance_;
	point			eye_goal_;
	vec				eye_direction_;
	QPoint			current_position_;

	// Render information
	OperationMode	op_mode_;
	bool			is_draw_point_;
	bool			is_draw_edge_;
	bool			is_draw_heat_;
	bool			is_draw_bulk_;
	bool			is_draw_order_;
	bool			has_lighting_;
	bool			is_draw_axes_;

	// Fiber
	FiberPrintPlugIn	*ptr_fiberprint_;

	vector<WF_vert*>	captured_verts_;
	vector<bool>		is_captured_vert_;
	vector<WF_edge*>	captured_edges_;
	vector<bool>		is_captured_edge_;

	vector<WF_vert*>	base_;
	vector<WF_edge*>	ceiling_;

	float				scale_;
	int					print_order_;
};

#endif // RENDERINGWIDGET_H
